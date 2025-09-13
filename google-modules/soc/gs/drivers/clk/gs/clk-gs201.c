// SPDX-License-Identifier: GPL-2.0-only
/*
 * Common Clock Framework support for GS201 SoC.
 *
 * Copyright (c) 2019-2021 Google LLC
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/google/cal-if.h>
#include <dt-bindings/clock/gs201.h>

#include "../../soc/google/cal-if/gs201/cmucal-vclk.h"
#include "../../soc/google/cal-if/gs201/cmucal-node.h"
#include "../../soc/google/cal-if/gs201/cmucal-qch.h"
#include "../../soc/google/cal-if/gs201/clkout_gs201.h"
#include "composite.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/arm-smccc.h>

#define PAD_CTRL_CLKOUT0 0x18063e80
#define PAD_CTRL_CLKOUT1 0x18063e84

static const phys_addr_t clkout_addresses[] = {
	0x1a000810,
	0x18000810,
	0x25a00810,
	0x1ca00810,
	0x1E080810,
	0x20c00810,
	0x20C00814,
	0x20c10810,
	0x20c20810,
	0x1a400810,
	0x1c200810,
	0x1b000810,
	0x1c000818,
	0x17000810,
	0x1c600810,
	0x1a800810,
	0x27f00810,
	0x27f00814,
	0x1d000810,
	0x11000810,
	0x11800810,
	0x14400810,
	0x1ac00810,
	0x1b400810,
	0x1b700810,
	0x1c800810,
	0x20800810,
	0x10010810,
	0x1e000810,
	0x20000810,
	0x1e800810,
	0x1f000810,
	0x1aa00810,
	0x10800810,
	0x10c00810,
	0x1bc00810,
	0x1cc00810
};
#endif

static struct samsung_clk_provider *gs201_clk_provider;
/*
 * list of controller registers to be saved and restored during a
 * suspend/resume cycle.
 */
/* fixed rate clocks generated outside the soc */
static struct samsung_fixed_rate gs201_fixed_rate_ext_clks[] = {
	FRATE(OSCCLK,
		"fin_pll",
		NULL,
		0,
		24576000),
};

/* HWACG VCLK */
static struct init_vclk gs201_apm_hwacg_vclks[] = {
	HWACG_VCLK(MUX_APM_FUNCSRC,
		MUX_CLKCMU_APM_FUNCSRC,
		"MUX_APM_FUNCSRC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(MUX_APM_FUNC,
		MUX_CLKCMU_APM_FUNC,
		"MUX_APM_FUNC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_aur_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_AUR_AURCTL,
		MUX_CLKCMU_AUR_AURCTL_USER,
		"UMUX_CLKCMU_AUR_AURCTL",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_AUR_NOC,
		MUX_CLKCMU_AUR_NOC_USER,
		"UMUX_CLKCMU_AUR_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_top_hwacg_vclks[] = {
	HWACG_VCLK(GATE_DFTMUX_CMU_CIS_CLK0,
		DFTMUX_CMU_QCH_CIS_CLK0,
		"GATE_DFTMUX_CMU_CIS_CLK0",
		NULL,
		0,
		VCLK_GATE | VCLK_QCH_DIS,
		NULL),
	HWACG_VCLK(GATE_DFTMUX_CMU_CIS_CLK1,
		DFTMUX_CMU_QCH_CIS_CLK1,
		"GATE_DFTMUX_CMU_CIS_CLK1",
		NULL,
		0,
		VCLK_GATE | VCLK_QCH_DIS,
		NULL),
	HWACG_VCLK(GATE_DFTMUX_CMU_CIS_CLK2,
		DFTMUX_CMU_QCH_CIS_CLK2,
		"GATE_DFTMUX_CMU_CIS_CLK2",
		NULL,
		0,
		VCLK_GATE | VCLK_QCH_DIS,
		NULL),
	HWACG_VCLK(GATE_DFTMUX_CMU_CIS_CLK3,
		DFTMUX_CMU_QCH_CIS_CLK3,
		"GATE_DFTMUX_CMU_CIS_CLK3",
		NULL,
		0,
		VCLK_GATE | VCLK_QCH_DIS,
		NULL),
	HWACG_VCLK(GATE_DFTMUX_CMU_CIS_CLK4,
		DFTMUX_CMU_QCH_CIS_CLK4,
		"GATE_DFTMUX_CMU_CIS_CLK4",
		NULL,
		0,
		VCLK_GATE | VCLK_QCH_DIS,
		NULL),
	HWACG_VCLK(GATE_DFTMUX_CMU_CIS_CLK5,
		DFTMUX_CMU_QCH_CIS_CLK5,
		"GATE_DFTMUX_CMU_CIS_CLK5",
		NULL,
		0,
		VCLK_GATE | VCLK_QCH_DIS,
		NULL),
	HWACG_VCLK(GATE_DFTMUX_CMU_CIS_CLK6,
		DFTMUX_CMU_QCH_CIS_CLK6,
		"GATE_DFTMUX_CMU_CIS_CLK6",
		NULL,
		0,
		VCLK_GATE | VCLK_QCH_DIS,
		NULL),
	HWACG_VCLK(GATE_DFTMUX_CMU_CIS_CLK7,
		DFTMUX_CMU_QCH_CIS_CLK7,
		"GATE_DFTMUX_CMU_CIS_CLK7",
		NULL,
		0,
		VCLK_GATE | VCLK_QCH_DIS,
		NULL),
};

static struct init_vclk gs201_nocl0_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_NOCL0_NOC,
		MUX_CLKCMU_NOCL0_NOC_USER,
		"UMUX_CLKCMU_NOCL0_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(MUX_NOCL0_NOC_OPTION1,
		MUX_CLK_NOCL0_NOC_OPTION1,
		"MUX_NOCL0_NOC_OPTION1",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_nocl1a_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_NOCL1A_NOC,
		MUX_CLKCMU_NOCL1A_NOC_USER,
		"UMUX_CLKCMU_NOCL1A_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_nocl1b_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_NOCL1B_NOC,
		MUX_CLKCMU_NOCL1B_NOC_USER,
		"UMUX_CLKCMU_NOCL1B_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_nocl2a_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_NOCL2A_NOC,
		MUX_CLKCMU_NOCL2A_NOC_USER,
		"UMUX_CLKCMU_NOCL2A_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_eh_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_EH_NOC,
		MUX_CLKCMU_EH_NOC_USER,
		"UMUX_CLKCMU_EH_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_EH_PLL_NOCL0,
		MUX_CLKCMU_EH_PLL_NOCL0_USER,
		"UMUX_CLKCMU_EH_PLL_NOCL0",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(MUX_EH_NOC,
		MUX_CLK_EH_NOC,
		"MUX_EH_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_g3d_hwacg_vclks[] = {
	HWACG_VCLK(GATE_GPU,
		GPU_QCH,
		"GATE_GPU",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_dpu_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_DPU_NOC,
		MUX_CLKCMU_DPU_NOC_USER,
		"UMUX_CLKCMU_DPU_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_DPUF_DMA,
		DPUF_QCH_DPU_DMA,
		"GATE_DPUF_DMA",
		"UMUX_CLKCMU_DPU_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_DPUF_DPP,
		DPUF_QCH_DPU_DPP,
		"GATE_DPUF_DPP",
		"UMUX_CLKCMU_DPU_NOC",
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_disp_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_DISP_NOC,
		MUX_CLKCMU_DISP_NOC_USER,
		"UMUX_CLKCMU_DISP_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_DPUB,
		DPUB_QCH,
		"GATE_DPUB",
		"UMUX_CLKCMU_DISP_NOC",
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_g2d_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_G2D_G2D,
		MUX_CLKCMU_G2D_G2D_USER,
		"UMUX_CLKCMU_G2D_G2D",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_G2D_MSCL,
		MUX_CLKCMU_G2D_MSCL_USER,
		"UMUX_CLKCMU_G2D_MSCL",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_G2D,
		G2D_QCH,
		"GATE_G2D",
		"UMUX_CLKCMU_G2D_G2D",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_JPEG,
		JPEG_QCH,
		"GATE_JPEG",
		"UMUX_CLKCMU_G2D_MSCL",
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_hsi0_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_HSI0_TCXO,
		MUX_CLKCMU_HSI0_TCXO_USER,
		"UMUX_CLKCMU_HSI0_TCXO",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(MUX_HSI0_USB20_REF,
		MUX_CLK_HSI0_USB20_REF,
		"MUX_HSI0_USB20_REF",
		NULL,
		0,
		0,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_HSI0_USB31DRD,
		MUX_CLKCMU_HSI0_USB31DRD_USER,
		"UMUX_CLKCMU_HSI0_USB31DRD",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_HSI0_USB20,
		MUX_CLKCMU_HSI0_USB20_USER,
		"UMUX_CLKCMU_HSI0_USB20",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(MUX_HSI0_USB31DRD,
		MUX_CLK_HSI0_USB31DRD,
		"MUX_HSI0_USB31DRD",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_HSI0_NOC,
		MUX_CLKCMU_HSI0_NOC_USER,
		"UMUX_CLKCMU_HSI0_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_HSI0_ALT,
		MUX_CLKCMU_HSI0_ALT_USER,
		"UMUX_CLKCMU_HSI0_ALT",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(MUX_HSI0_NOC,
		MUX_CLK_HSI0_NOC,
		"MUX_HSI0_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_HSI0_DPGTC,
		MUX_CLKCMU_HSI0_DPGTC_USER,
		"UMUX_CLKCMU_HSI0_DPGTC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_USB31DRD_SLV_LINK,
		USB31DRD_QCH_SLV_LINK,
		"GATE_USB31DRD_SLV_LINK",
		"MUX_HSI0_USB31DRD",
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_hsi1_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_HSI1_NOC,
		MUX_CLKCMU_HSI1_NOC_USER,
		"UMUX_CLKCMU_HSI1_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_HSI1_PCIE,
		MUX_CLKCMU_HSI1_PCIE_USER,
		"UMUX_CLKCMU_HSI1_PCIE",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_DBG_1,
		PCIE_GEN4_0_QCH_DBG_1,
		"GATE_PCIE_GEN4_0_DBG_1",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_AXI_1,
		PCIE_GEN4_0_QCH_AXI_1,
		"GATE_PCIE_GEN4_0_AXI_1",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_APB_1,
		PCIE_GEN4_0_QCH_APB_1,
		"GATE_PCIE_GEN4_0_APB_1",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_SCLK_1,
		PCIE_GEN4_0_QCH_SCLK_1,
		"GATE_PCIE_GEN4_0_SCLK_1",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_PCS_APB,
		PCIE_GEN4_0_QCH_PCS_APB,
		"GATE_PCIE_GEN4_0_PCS_APB",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_PMA_APB,
		PCIE_GEN4_0_QCH_PMA_APB,
		"GATE_PCIE_GEN4_0_PMA_APB",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_DBG_2,
		PCIE_GEN4_0_QCH_DBG_2,
		"GATE_PCIE_GEN4_0_DBG_2",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_AXI_2,
		PCIE_GEN4_0_QCH_AXI_2,
		"GATE_PCIE_GEN4_0_AXI_2",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_APB_2,
		PCIE_GEN4_0_QCH_APB_2,
		"GATE_PCIE_GEN4_0_APB_2",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_0_UDBG,
		PCIE_GEN4_0_QCH_UDBG,
		"GATE_PCIE_GEN4_0_UDBG",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_hsi2_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_HSI2_NOC,
		MUX_CLKCMU_HSI2_NOC_USER,
		"UMUX_CLKCMU_HSI2_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_HSI2_PCIE,
		MUX_CLKCMU_HSI2_PCIE_USER,
		"UMUX_CLKCMU_HSI2_PCIE",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_HSI2_UFS_EMBD,
		MUX_CLKCMU_HSI2_UFS_EMBD_USER,
		"UMUX_CLKCMU_HSI2_UFS_EMBD",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_HSI2_MMC_CARD,
		MUX_CLKCMU_HSI2_MMC_CARD_USER,
		"UMUX_CLKCMU_HSI2_MMC_CARD",
		NULL,
		0,
		VCLK_GATE,
		NULL),

	HWACG_VCLK(GATE_MMC_CARD,
		MMC_CARD_QCH,
		"GATE_MMC_CARD",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_1_AXI_1,
		PCIE_GEN4_1_QCH_AXI_1,
		"GATE_PCIE_GEN4_1_AXI_1",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_1_APB_1,
		PCIE_GEN4_1_QCH_APB_1,
		"GATE_PCIE_GEN4_1_APB_1",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_1_DBG_1,
		PCIE_GEN4_1_QCH_DBG_1,
		"GATE_PCIE_GEN4_1_DBG_1",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_1_PCS_APB,
		PCIE_GEN4_1_QCH_PCS_APB,
		"GATE_PCIE_GEN4_1_PCS_APB",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_1_PMA_APB,
		PCIE_GEN4_1_QCH_PMA_APB,
		"GATE_PCIE_GEN4_1_PMA_APB",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_1_AXI_2,
		PCIE_GEN4_1_QCH_AXI_2,
		"GATE_PCIE_GEN4_1_AXI_2",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_1_DBG_2,
		PCIE_GEN4_1_QCH_DBG_2,
		"GATE_PCIE_GEN4_1_DBG_2",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_1_APB_2,
		PCIE_GEN4_1_QCH_APB_2,
		"GATE_PCIE_GEN4_1_APB_2",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN4_1_UDBG,
		PCIE_GEN4_1_QCH_UDBG,
		"GATE_PCIE_GEN4_1_UDBG",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_UFS_EMBD,
		UFS_EMBD_QCH,
		"GATE_UFS_EMBD",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_UFS_EMBD_FMP,
		UFS_EMBD_QCH_FMP,
		"GATE_UFS_EMBD_FMP",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_csis_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_CSIS_NOC,
		MUX_CLKCMU_CSIS_NOC_USER,
		"UMUX_CLKCMU_CSIS_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_pdp_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_PDP_NOC,
		MUX_CLKCMU_PDP_NOC_USER,
		"UMUX_CLKCMU_PDP_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_PDP_VRA,
		MUX_CLKCMU_PDP_VRA_USER,
		"UMUX_CLKCMU_PDP_VRA",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_ipp_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_IPP_NOC,
		MUX_CLKCMU_IPP_NOC_USER,
		"UMUX_CLKCMU_IPP_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_g3aa_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_G3AA_G3AA,
		MUX_CLKCMU_G3AA_G3AA_USER,
		"UMUX_CLKCMU_G3AA_G3AA",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_itp_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_ITP_NOC,
		MUX_CLKCMU_ITP_NOC_USER,
		"UMUX_CLKCMU_ITP_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_dns_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_DNS_NOC,
		MUX_CLKCMU_DNS_NOC_USER,
		"UMUX_CLKCMU_DNS_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_tnr_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_TNR_NOC,
		MUX_CLKCMU_TNR_NOC_USER,
		"UMUX_CLKCMU_TNR_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_mcsc_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_MCSC_ITSC,
		MUX_CLKCMU_MCSC_ITSC_USER,
		"UMUX_CLKCMU_MCSC_ITSC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_MCSC_MCSC,
		MUX_CLKCMU_MCSC_MCSC_USER,
		"UMUX_CLKCMU_MCSC_MCSC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_gdc_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_GDC_SCSC,
		MUX_CLKCMU_GDC_SCSC_USER,
		"UMUX_CLKCMU_GDC_SCSC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_GDC_GDC0,
		MUX_CLKCMU_GDC_GDC0_USER,
		"UMUX_CLKCMU_GDC_GDC0",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_GDC_GDC1,
		MUX_CLKCMU_GDC_GDC1_USER,
		"UMUX_CLKCMU_GDC_GDC1",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_mfc_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_MFC_MFC,
		MUX_CLKCMU_MFC_MFC_USER,
		"UMUX_CLKCMU_MFC_MFC",
		NULL,
		0,
		0,
		NULL),
	HWACG_VCLK(GATE_MFC,
		MFC_QCH,
		"GATE_MFC",
		"UMUX_CLKCMU_MFC_MFC",
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_mif_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_MIF_DDRPHY2X,
		CLKMUX_MIF_DDRPHY2X,
		"UMUX_MIF_DDRPHY2X",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_misc_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_MISC_NOC,
		MUX_CLKCMU_MISC_NOC_USER,
		"UMUX_CLKCMU_MISC_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_MISC_SSS,
		MUX_CLKCMU_MISC_SSS_USER,
		"UMUX_CLKCMU_MISC_SSS",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_MCT,
		MCT_QCH,
		"GATE_MCT",
		"DOUT_CLK_MISC_NOCP",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_WDT_CL0,
		WDT_CLUSTER0_QCH,
		"GATE_WDT_CL0",
		"DOUT_CLK_MISC_NOCP",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_WDT_CL1,
		WDT_CLUSTER1_QCH,
		"GATE_WDT_CL1",
		"DOUT_CLK_MISC_NOCP",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PDMA0,
		PDMA0_QCH,
		"GATE0_PDMA",
		"DOUT_CLK_MISC_NOCP",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PDMA1,
		PDMA1_QCH,
		"GATE1_PDMA",
		"DOUT_CLK_MISC_NOCP",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK_QACTIVE(ATCLK,
		"ATCLK",
		"DOUT_CLK_MISC_BUSP",
		0,
		VCLK_QACTIVE,
		NULL,
		0x2b00c02c,
		0x1,
		0x1),
};

static struct init_vclk gs201_peric0_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_PERIC0_NOC,
		MUX_CLKCMU_PERIC0_NOC_USER,
		"UMUX_CLKCMU_PERIC0_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_PERIC0_USI0_UART,
		MUX_CLKCMU_PERIC0_USI0_UART_USER,
		"UMUX_CLKCMU_PERIC0_USI0_UART",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_USI1_USI,
		USI1_USI_QCH,
		"GATE_PERIC0_TOP0_USI1_USI",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_USI2_USI,
		USI2_USI_QCH,
		"GATE_PERIC0_TOP0_USI2_USI",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_USI3_USI,
		USI3_USI_QCH,
		"GATE_PERIC0_TOP0_USI3_USI",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_USI4_USI,
		USI4_USI_QCH,
		"GATE_PERIC0_TOP0_USI4_USI",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_USI5_USI,
		USI5_USI_QCH,
		"GATE_PERIC0_TOP0_USI5_USI",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_USI6_USI,
		USI6_USI_QCH,
		"GATE_PERIC0_TOP0_USI6_USI",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_USI7_USI,
		USI7_USI_QCH,
		"GATE_PERIC0_TOP0_USI7_USI",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_USI8_USI,
		USI8_USI_QCH,
		"GATE_PERIC0_TOP0_USI8_USI",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_I3C1,
		I3C1_QCH_SCLK,
		"GATE_PERIC0_TOP0_I3C1",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_I3C2,
		I3C2_QCH_SCLK,
		"GATE_PERIC0_TOP0_I3C2",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_I3C3,
		I3C3_QCH_SCLK,
		"GATE_PERIC0_TOP0_I3C3",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_I3C4,
		I3C4_QCH_SCLK,
		"GATE_PERIC0_TOP0_I3C4",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_I3C5,
		I3C5_QCH_SCLK,
		"GATE_PERIC0_TOP0_I3C5",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_I3C6,
		I3C6_QCH_SCLK,
		"GATE_PERIC0_TOP0_I3C6",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_I3C7,
		I3C7_QCH_SCLK,
		"GATE_PERIC0_TOP0_I3C7",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP0_I3C8,
		I3C8_QCH_SCLK,
		"GATE_PERIC0_TOP0_I3C8",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP1_USI0_UART,
		USI0_UART_QCH,
		"GATE_PERIC0_TOP1_USI0_UART",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC0_TOP1_USI14_USI,
		USI14_USI_QCH,
		"GATE_PERIC0_TOP1_USI14_USI",
		"UMUX_CLKCMU_PERIC0_NOC",
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_peric1_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_PERIC1_NOC,
		MUX_CLKCMU_PERIC1_NOC_USER,
		"UMUX_CLKCMU_PERIC1_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_USI0_USI,
		USI0_USI_QCH,
		"GATE_PERIC1_TOP0_USI0_USI",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_USI9_USI,
		USI9_USI_QCH,
		"GATE_PERIC1_TOP0_USI9_USI",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_USI10_USI,
		USI10_USI_QCH,
		"GATE_PERIC1_TOP0_USI10_USI",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_USI11_USI,
		USI11_USI_QCH,
		"GATE_PERIC1_TOP0_USI11_USI",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_USI12_USI,
		USI12_USI_QCH,
		"GATE_PERIC1_TOP0_USI12_USI",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_USI13_USI,
		USI13_USI_QCH,
		"GATE_PERIC1_TOP0_USI13_USI",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_I3C0,
		I3C0_QCH_SCLK,
		"GATE_PERIC1_TOP0_I3C0",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_PWM,
		PWM_QCH,
		"GATE_PERIC1_TOP0_PWM",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_USI15_USI,
		USI15_USI_QCH,
		"GATE_PERIC1_TOP0_USI15_USI",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PERIC1_TOP0_USI16_USI,
		USI16_USI_QCH,
		"GATE_PERIC1_TOP0_USI16_USI",
		"UMUX_CLKCMU_PERIC1_NOC",
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_tpu_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_TPU_TPU,
		MUX_CLKCMU_TPU_TPU_USER,
		"UMUX_CLKCMU_TPU_TPU",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_TPU_TPUCTL,
		MUX_CLKCMU_TPU_TPUCTL_USER,
		"UMUX_CLKCMU_TPU_TPUCTL",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_TPU_NOC,
		MUX_CLKCMU_TPU_NOC_USER,
		"UMUX_CLKCMU_TPU_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_TPU_UART,
		MUX_CLKCMU_TPU_UART_USER,
		"UMUX_CLKCMU_TPU_UART",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(MUX_TPU_TPU,
		MUX_CLK_TPU_TPU,
		"MUX_TPU_TPU",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(MUX_TPU_TPUCTL,
		MUX_CLK_TPU_TPUCTL,
		"MUX_TPU_TPUCTL",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

static struct init_vclk gs201_bo_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_BO_NOC,
		MUX_CLKCMU_BO_NOC_USER,
		"UMUX_CLKCMU_BO_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

/* Special VCLK */
static struct init_vclk gs201_apm_vclks[] = {
	VCLK(DOUT_CLK_APM_BOOST,
		DIV_CLK_APM_BOOST,
		"DOUT_CLK_APM_BOOST",
		0,
		0,
		NULL),
	VCLK(DOUT_CLK_APM_USI0_UART,
		DIV_CLK_APM_USI0_UART,
		"DOUT_CLK_APM_USI0_UART",
		0,
		0,
		NULL),
	VCLK(DOUT_CLK_APM_USI1_UART,
		DIV_CLK_APM_USI1_UART,
		"DOUT_CLK_APM_USI1_UART",
		0,
		0,
		NULL),
	VCLK(DOUT_CLK_APM_USI0_USI,
		DIV_CLK_APM_USI0_USI,
		"DOUT_CLK_APM_USI0_USI",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_aur_vclks[] = {
	VCLK(DOUT_CLK_AUR_NOCP,
		DIV_CLK_AUR_NOCP,
		"DOUT_CLK_AUR_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_nocl0_vclks[] = {
	VCLK(DOUT_CLK_NOCL0_NOCP,
		DIV_CLK_NOCL0_NOCP,
		"DOUT_CLK_NOCL0_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_nocl1a_vclks[] = {
	VCLK(DOUT_CLK_NOCL1A_NOCP,
		DIV_CLK_NOCL1A_NOCP,
		"DOUT_CLK_NOCL1A_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_nocl1b_vclks[] = {
	VCLK(DOUT_CLK_NOCL1B_NOCP,
		DIV_CLK_NOCL1B_NOCP,
		"DOUT_CLK_NOCL1B_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_nocl2a_vclks[] = {
	VCLK(DOUT_CLK_NOCL2A_NOCP,
		DIV_CLK_NOCL2A_NOCP,
		"DOUT_CLK_NOCL2A_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_eh_vclks[] = {
	VCLK(DOUT_CLK_EH_NOCP,
		DIV_CLK_EH_NOCP,
		"DOUT_CLK_EH_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_dpu_vclks[] = {
	VCLK(DOUT_CLK_DPU_NOCP,
		DIV_CLK_DPU_NOCP,
		"DOUT_CLK_DPU_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_disp_vclks[] = {
	VCLK(DOUT_CLK_DISP_NOCP,
		DIV_CLK_DISP_NOCP,
		"DOUT_CLK_DISP_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_g2d_vclks[] = {
	VCLK(DOUT_CLK_G2D_NOCP,
		DIV_CLK_G2D_NOCP,
		"DOUT_CLK_G2D_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_hsi0_vclks[] = {
	VCLK(DOUT_CLK_HSI0_USB31DRD,
		DIV_CLK_HSI0_USB31DRD,
		"DOUT_CLK_HSI0_USB31DRD",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_csis_vclks[] = {
	VCLK(DOUT_CLK_CSIS_NOCP,
		DIV_CLK_CSIS_NOCP,
		"DOUT_CLK_CSIS_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_pdp_vclks[] = {
	VCLK(DOUT_CLK_PDP_NOCP,
		DIV_CLK_PDP_NOCP,
		"DOUT_CLK_PDP_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_ipp_vclks[] = {
	VCLK(DOUT_CLK_IPP_NOCP,
		DIV_CLK_IPP_NOCP,
		"DOUT_CLK_IPP_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_g3aa_vclks[] = {
	VCLK(DOUT_CLK_G3AA_NOCP,
		DIV_CLK_G3AA_NOCP,
		"DOUT_CLK_G3AA_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_itp_vclks[] = {
	VCLK(DOUT_CLK_ITP_NOCP,
		DIV_CLK_ITP_NOCP,
		"DOUT_CLK_ITP_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_dns_vclks[] = {
	VCLK(DOUT_CLK_DNS_NOCP,
		DIV_CLK_DNS_NOCP,
		"DOUT_CLK_DNS_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_tnr_vclks[] = {
	VCLK(DOUT_CLK_TNR_NOCP,
		DIV_CLK_TNR_NOCP,
		"DOUT_CLK_TNR_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_mcsc_vclks[] = {
	VCLK(DOUT_CLK_MCSC_NOCP,
		DIV_CLK_MCSC_NOCP,
		"DOUT_CLK_MCSC_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_gdc_vclks[] = {
	VCLK(DOUT_CLK_GDC_NOCP,
		DIV_CLK_GDC_NOCP,
		"DOUT_CLK_GDC_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_mfc_vclks[] = {
	VCLK(DOUT_CLK_MFC_NOCP,
		DIV_CLK_MFC_NOCP,
		"DOUT_CLK_MFC_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_misc_vclks[] = {
	VCLK(DOUT_CLK_MISC_NOCP,
		DIV_CLK_MISC_NOCP,
		"DOUT_CLK_MISC_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_top_vclks[] = {
	VCLK(VDOUT_CLK_TOP_HSI0_NOC,
		CLKCMU_HSI0_NOC,
		"VDOUT_CLK_TOP_HSI0_NOC",
		0,
		0,
		NULL),
	VCLK(CIS_CLK0,
		CLKCMU_CIS_CLK0,
		"CIS_CLK0",
		0,
		0,
		NULL),
	VCLK(CIS_CLK1,
		CLKCMU_CIS_CLK1,
		"CIS_CLK1",
		0,
		0,
		NULL),
	VCLK(CIS_CLK2,
		CLKCMU_CIS_CLK2,
		"CIS_CLK2",
		0,
		0,
		NULL),
	VCLK(CIS_CLK3,
		CLKCMU_CIS_CLK3,
		"CIS_CLK3",
		0,
		0,
		NULL),
	VCLK(CIS_CLK4,
		CLKCMU_CIS_CLK4,
		"CIS_CLK4",
		0,
		0,
		NULL),
	VCLK(CIS_CLK5,
		CLKCMU_CIS_CLK5,
		"CIS_CLK5",
		0,
		0,
		NULL),
	VCLK(CIS_CLK6,
		CLKCMU_CIS_CLK6,
		"CIS_CLK6",
		0,
		0,
		NULL),
	VCLK(CIS_CLK7,
		CLKCMU_CIS_CLK7,
		"CIS_CLK7",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_hsi2_vclks[] = {
	VCLK(DOUT_CLKCMU_HSI2_MMC_CARD,
		CLKCMU_HSI2_MMC_CARD,
		"DOUT_CLKCMU_HSI2_MMC_CARD",
		0,
		0,
		NULL),
	VCLK(UFS_EMBD,
		CLKCMU_HSI2_UFS_EMBD,
		"UFS_EMBD",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_peric0_vclks[] = {
	VCLK(VDOUT_CLK_PERIC0_USI0_UART,
		VCLK_DIV_CLK_PERIC0_USI0_UART,
		"VDOUT_CLK_PERIC0_USI0_UART",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_USI1_USI,
		VCLK_DIV_CLK_PERIC0_USI1_USI,
		"VDOUT_CLK_PERIC0_USI1_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_USI2_USI,
		VCLK_DIV_CLK_PERIC0_USI2_USI,
		"VDOUT_CLK_PERIC0_USI2_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_USI3_USI,
		VCLK_DIV_CLK_PERIC0_USI3_USI,
		"VDOUT_CLK_PERIC0_USI3_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_USI4_USI,
		VCLK_DIV_CLK_PERIC0_USI4_USI,
		"VDOUT_CLK_PERIC0_USI4_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_USI5_USI,
		VCLK_DIV_CLK_PERIC0_USI5_USI,
		"VDOUT_CLK_PERIC0_USI5_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_USI6_USI,
		VCLK_DIV_CLK_PERIC0_USI6_USI,
		"VDOUT_CLK_PERIC0_USI6_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_USI7_USI,
		VCLK_DIV_CLK_PERIC0_USI7_USI,
		"VDOUT_CLK_PERIC0_USI7_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_USI8_USI,
		VCLK_DIV_CLK_PERIC0_USI8_USI,
		"VDOUT_CLK_PERIC0_USI8_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_USI14_USI,
		VCLK_DIV_CLK_PERIC0_USI14_USI,
		"VDOUT_CLK_PERIC0_USI14_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC0_I3C,
		DIV_CLK_PERIC0_I3C,
		"VDOUT_CLK_PERIC0_I3C",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_peric1_vclks[] = {
	VCLK(VDOUT_CLK_PERIC1_USI0_USI,
		VCLK_DIV_CLK_PERIC1_USI0_USI,
		"VDOUT_CLK_PERIC1_USI0_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC1_USI9_USI,
		VCLK_DIV_CLK_PERIC1_USI9_USI,
		"VDOUT_CLK_PERIC1_USI9_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC1_USI10_USI,
		VCLK_DIV_CLK_PERIC1_USI10_USI,
		"VDOUT_CLK_PERIC1_USI10_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC1_USI11_USI,
		VCLK_DIV_CLK_PERIC1_USI11_USI,
		"VDOUT_CLK_PERIC1_USI11_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC1_USI12_USI,
		VCLK_DIV_CLK_PERIC1_USI12_USI,
		"VDOUT_CLK_PERIC1_USI12_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC1_USI13_USI,
		VCLK_DIV_CLK_PERIC1_USI13_USI,
		"VDOUT_CLK_PERIC1_USI13_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC1_I3C,
		VCLK_DIV_CLK_PERIC1_I3C,
		"VDOUT_CLK_PERIC1_I3C",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC1_USI15_USI,
		VCLK_DIV_CLK_PERIC1_USI15_USI,
		"VDOUT_CLK_PERIC1_USI15_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_PERIC1_USI16_USI,
		VCLK_DIV_CLK_PERIC1_USI16_USI,
		"VDOUT_CLK_PERIC1_USI16_USI",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_tpu_vclks[] = {
	VCLK(DOUT_CLK_TPU_TPU,
		DIV_CLK_TPU_TPU,
		"DOUT_CLK_TPU_TPU",
		0,
		0,
		NULL),
	VCLK(DOUT_CLK_TPU_TPUCTL,
		DIV_CLK_TPU_TPUCTL,
		"DOUT_CLK_TPU_TPUCTL",
		0,
		0,
		NULL),
	VCLK(DOUT_CLK_TPU_NOCP,
		DIV_CLK_TPU_NOCP,
		"DOUT_CLK_TPU_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_bo_vclks[] = {
	VCLK(DOUT_CLK_BO_NOCP,
		DIV_CLK_BO_NOCP,
		"DOUT_CLK_BO_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk gs201_clkout_vclks[] = {
	VCLK(CLKOUT1,
		VCLK_CLKOUT1,
		"CLKOUT1",
		0,
		0,
		NULL),
	VCLK(CLKOUT0,
		VCLK_CLKOUT0,
		"CLKOUT0",
		0,
		0,
		NULL),
};

#ifdef CONFIG_DEBUG_FS
static int pad_clkout0_get(void *data, u64 *val)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PRIV_REG, PAD_CTRL_CLKOUT0,
		      PRIV_REG_OPTION_READ, 0, 0, 0, 0, 0, &res);
	*val =  (u64)(res.a0 & 0xFFFFFFFFUL);

	return 0;
}

static int pad_clkout0_set(void *data, u64 val)
{
	int ret = 0;
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PRIV_REG, PAD_CTRL_CLKOUT0,
		      PRIV_REG_OPTION_WRITE, val, 0, 0, 0, 0, &res);
	if (res.a0 != 0) {
		pr_err("error writing to pad_clkout0 err=%lu\n", res.a0);
		ret = res.a0;
	}

	return ret;
}

static int pad_clkout1_get(void *data, u64 *val)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PRIV_REG, PAD_CTRL_CLKOUT1,
		      PRIV_REG_OPTION_READ, 0, 0, 0, 0, 0, &res);
	*val =  (u64)(res.a0 & 0xFFFFFFFFUL);

	return 0;
}

static int pad_clkout1_set(void *data, u64 val)
{
	int ret = 0;
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PRIV_REG, PAD_CTRL_CLKOUT1,
		      PRIV_REG_OPTION_WRITE, val, 0, 0, 0, 0, &res);
	if (res.a0 != 0) {
		pr_err("error writing to pad_clkout1 err=%lu\n", res.a0);
		ret = res.a0;
	}

	return ret;
}

static int clkout_addr_get(void *data, u64 *val)
{
	struct samsung_clk_provider *scp = data;
	*val = scp->clkout_addr;
	return 0;
}

static int clkout_addr_set(void *data, u64 val)
{
	int i;
	struct samsung_clk_provider *scp = data;

	for (i = 0; i < ARRAY_SIZE(clkout_addresses); i++)
		if (clkout_addresses[i] == val)
			break;

	if (i >= ARRAY_SIZE(clkout_addresses)) {
		pr_err("error address not found\n");
		return -ENODEV;
	}

	scp->clkout_addr = val;

	return 0;
}

static int clkout_val_get(void *data, u64 *val)
{
	u32 __iomem *addr;
	struct samsung_clk_provider *scp = data;

	addr = ioremap(scp->clkout_addr, SZ_4);
	*val = (u64)ioread32(addr);

	return 0;
}

static int clkout_val_set(void *data, u64 val)
{
	u32 __iomem *addr;
	struct samsung_clk_provider *scp = data;

	addr = ioremap(scp->clkout_addr, SZ_4);
	iowrite32((u32)val, addr);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(pad_clkout0_fops, pad_clkout0_get,
			 pad_clkout0_set, "0x%08llx\n");

DEFINE_DEBUGFS_ATTRIBUTE(pad_clkout1_fops, pad_clkout1_get,
			 pad_clkout1_set, "0x%08llx\n");

DEFINE_DEBUGFS_ATTRIBUTE(clkout_addr_fops, clkout_addr_get,
			 clkout_addr_set, "0x%16llx\n");

DEFINE_DEBUGFS_ATTRIBUTE(clkout_val_fops, clkout_val_get,
			 clkout_val_set, "0x%08llx\n");

#endif

static const struct of_device_id ext_clk_match[] = {
	{.compatible = "samsung,gs201-oscclk", .data = (void *)0},
	{},
};

static int gs201_clock_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	void __iomem *reg_base;
#ifdef CONFIG_DEBUG_FS
	struct dentry *root;
#endif

	if (!np)
		panic("%s: unable to determine soc\n", __func__);

	reg_base = of_iomap(np, 0);
	if (!reg_base)
		panic("%s: failed to map registers\n", __func__);

	gs201_clk_provider = samsung_clk_init(np, reg_base, CLK_NR_CLKS);
	if (!gs201_clk_provider)
		panic("%s: unable to allocate context.\n", __func__);

	samsung_register_of_fixed_ext(gs201_clk_provider,
					gs201_fixed_rate_ext_clks,
					ARRAY_SIZE(gs201_fixed_rate_ext_clks),
					ext_clk_match);

	/* register HWACG vclk */
	samsung_register_vclk(gs201_clk_provider,
				gs201_apm_hwacg_vclks,
				ARRAY_SIZE(gs201_apm_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_aur_hwacg_vclks,
				ARRAY_SIZE(gs201_aur_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_top_hwacg_vclks,
				ARRAY_SIZE(gs201_top_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_nocl0_hwacg_vclks,
				ARRAY_SIZE(gs201_nocl0_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_nocl1a_hwacg_vclks,
				ARRAY_SIZE(gs201_nocl1a_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_nocl1b_hwacg_vclks,
				ARRAY_SIZE(gs201_nocl1b_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_nocl2a_hwacg_vclks,
				ARRAY_SIZE(gs201_nocl2a_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_eh_hwacg_vclks,
				ARRAY_SIZE(gs201_eh_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_g3d_hwacg_vclks,
				ARRAY_SIZE(gs201_g3d_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_dpu_hwacg_vclks,
				ARRAY_SIZE(gs201_dpu_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_disp_hwacg_vclks,
				ARRAY_SIZE(gs201_disp_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_g2d_hwacg_vclks,
				ARRAY_SIZE(gs201_g2d_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_hsi0_hwacg_vclks,
				ARRAY_SIZE(gs201_hsi0_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_hsi1_hwacg_vclks,
				ARRAY_SIZE(gs201_hsi1_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_hsi2_hwacg_vclks,
				ARRAY_SIZE(gs201_hsi2_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_csis_hwacg_vclks,
				ARRAY_SIZE(gs201_csis_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_pdp_hwacg_vclks,
				ARRAY_SIZE(gs201_pdp_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_ipp_hwacg_vclks,
				ARRAY_SIZE(gs201_ipp_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_g3aa_hwacg_vclks,
				ARRAY_SIZE(gs201_g3aa_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_itp_hwacg_vclks,
				ARRAY_SIZE(gs201_itp_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_dns_hwacg_vclks,
				ARRAY_SIZE(gs201_dns_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_tnr_hwacg_vclks,
				ARRAY_SIZE(gs201_tnr_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_mcsc_hwacg_vclks,
				ARRAY_SIZE(gs201_mcsc_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_gdc_hwacg_vclks,
				ARRAY_SIZE(gs201_gdc_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_mfc_hwacg_vclks,
				ARRAY_SIZE(gs201_mfc_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_mif_hwacg_vclks,
				ARRAY_SIZE(gs201_mif_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_misc_hwacg_vclks,
				ARRAY_SIZE(gs201_misc_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_peric0_hwacg_vclks,
				ARRAY_SIZE(gs201_peric0_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_peric1_hwacg_vclks,
				ARRAY_SIZE(gs201_peric1_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_tpu_hwacg_vclks,
				ARRAY_SIZE(gs201_tpu_hwacg_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_bo_hwacg_vclks,
				ARRAY_SIZE(gs201_bo_hwacg_vclks));

	/* register special vclk */
	samsung_register_vclk(gs201_clk_provider,
				gs201_apm_vclks,
				ARRAY_SIZE(gs201_apm_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_aur_vclks,
				ARRAY_SIZE(gs201_aur_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_nocl0_vclks,
				ARRAY_SIZE(gs201_nocl0_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_nocl1a_vclks,
				ARRAY_SIZE(gs201_nocl1a_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_nocl1b_vclks,
				ARRAY_SIZE(gs201_nocl1b_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_nocl2a_vclks,
				ARRAY_SIZE(gs201_nocl2a_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_eh_vclks,
				ARRAY_SIZE(gs201_eh_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_dpu_vclks,
				ARRAY_SIZE(gs201_dpu_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_disp_vclks,
				ARRAY_SIZE(gs201_disp_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_g2d_vclks,
				ARRAY_SIZE(gs201_g2d_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_hsi0_vclks,
				ARRAY_SIZE(gs201_hsi0_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_csis_vclks,
				ARRAY_SIZE(gs201_csis_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_pdp_vclks,
				ARRAY_SIZE(gs201_pdp_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_ipp_vclks,
				ARRAY_SIZE(gs201_ipp_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_g3aa_vclks,
				ARRAY_SIZE(gs201_g3aa_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_itp_vclks,
				ARRAY_SIZE(gs201_itp_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_dns_vclks,
				ARRAY_SIZE(gs201_dns_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_tnr_vclks,
				ARRAY_SIZE(gs201_tnr_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_mcsc_vclks,
				ARRAY_SIZE(gs201_mcsc_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_gdc_vclks,
				ARRAY_SIZE(gs201_gdc_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_mfc_vclks,
				ARRAY_SIZE(gs201_mfc_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_misc_vclks,
				ARRAY_SIZE(gs201_misc_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_top_vclks,
				ARRAY_SIZE(gs201_top_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_hsi2_vclks,
				ARRAY_SIZE(gs201_hsi2_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_peric0_vclks,
				ARRAY_SIZE(gs201_peric0_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_peric1_vclks,
				ARRAY_SIZE(gs201_peric1_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_tpu_vclks,
				ARRAY_SIZE(gs201_tpu_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_bo_vclks,
				ARRAY_SIZE(gs201_bo_vclks));
	samsung_register_vclk(gs201_clk_provider,
				gs201_clkout_vclks,
				ARRAY_SIZE(gs201_clkout_vclks));

	clk_register_fixed_factor(NULL,
				"pwm-clock",
				"fin_pll",
				CLK_SET_RATE_PARENT,
				1,
				1);

	samsung_clk_of_add_provider(np, gs201_clk_provider);

#ifdef CONFIG_DEBUG_FS
	root = debugfs_create_dir("xclkout", NULL);
	debugfs_create_file("pad_clkout0", 0644, root, gs201_clk_provider,
			    &pad_clkout0_fops);
	debugfs_create_file("pad_clkout1", 0644, root, gs201_clk_provider,
			    &pad_clkout1_fops);
	debugfs_create_file("clkout_addr", 0644, root, gs201_clk_provider,
			    &clkout_addr_fops);
	debugfs_create_file("clkout_val", 0644, root, gs201_clk_provider,
			    &clkout_val_fops);
#endif

	pr_info("GS201: Clock setup completed\n");

	return 0;
}

static const struct of_device_id of_exynos_clock_match[] = {
	{ .compatible = "samsung,gs201-clock", },
	{ },
};
MODULE_DEVICE_TABLE(of, of_exynos_clock_match);

static const struct platform_device_id exynos_clock_ids[] = {
	{ "gs201-clock", },
	{ }
};

static struct platform_driver gs201_clock_driver = {
	.driver = {
		.name = "gs201_clock",
		.of_match_table = of_exynos_clock_match,
	},
	.probe		= gs201_clock_probe,
	.id_table	= exynos_clock_ids,
};

module_platform_driver(gs201_clock_driver);

MODULE_LICENSE("GPL");
