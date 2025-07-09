// SPDX-License-Identifier: GPL-2.0-only
/*
 * Common Clock Framework support for ZUMA SoC.
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/google/cal-if.h>
#include <dt-bindings/clock/zuma.h>

#include "../../soc/google/cal-if/zuma/cmucal-vclk.h"
#include "../../soc/google/cal-if/zuma/cmucal-node.h"
#include "../../soc/google/cal-if/zuma/cmucal-qch.h"
#include "../../soc/google/cal-if/zuma/clkout_zuma.h"
#include "composite.h"


#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/arm-smccc.h>

#define PAD_CTRL_CLKOUT0 0x15463e80
#define PAD_CTRL_CLKOUT1 0x15463e84

static const phys_addr_t clkout_addresses[] = {
	(0x15400000 + 0x0810),	/* "CLKOUT_CON_BLK_APM_CMU_APM_CLKOUT0" */
	(0x26040000 + 0x0810),	/* "CLKOUT_CON_BLK_CMU_CMU_TOP_CLKOUT0" */
	(0x29c00000 + 0x0810),	/* "CLKOUT_CON_BLK_CPUCL0_CMU_CPUCL0_CLKOUT0" */
	(0x29c00000 + 0x0814),	/* "CLKOUT_CON_BLK_CPUCL0_EMBEDDED_CMU_CPUCL0_CLKOUT0" */
	(0x27d00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x27e00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x27f00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x10010000 + 0x0810),	/* "CLKOUT_CON_BLK_MISC_CMU_MISC_CLKOUT0" */
	(0x13000000 + 0x0810),	/* "CLKOUT_CON_BLK_HSI2_CMU_HSI2_CLKOUT0" */
	(0x10800000 + 0x0810),	/* "CLKOUT_CON_BLK_PERIC0_CMU_PERIC0_CLKOUT0" */
	(0x10c00000 + 0x0810),	/* "CLKOUT_CON_BLK_PERIC1_CMU_PERIC1_CLKOUT0" */
	(0x15400000 + 0x0810),	/* "CLKOUT_CON_BLK_APM_CMU_APM_CLKOUT0" */
	(0x26040000 + 0x0810),	/* "CLKOUT_CON_BLK_CMU_CMU_TOP_CLKOUT0" */
	(0x29c00000 + 0x0810),	/* "CLKOUT_CON_BLK_CPUCL0_CMU_CPUCL0_CLKOUT0" */
	(0x29c00000 + 0x0814),	/* "CLKOUT_CON_BLK_CPUCL0_EMBEDDED_CMU_CPUCL0_CLKOUT0" */
	(0x27c00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x27d00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x27e00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x27f00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x10010000 + 0x0810),	/* "CLKOUT_CON_BLK_MISC_CMU_MISC_CLKOUT0" */
	(0x13000000 + 0x0810),	/* "CLKOUT_CON_BLK_HSI2_CMU_HSI2_CLKOUT0" */
	(0x10800000 + 0x0810),	/* "CLKOUT_CON_BLK_PERIC0_CMU_PERIC0_CLKOUT0" */
	(0x10c00000 + 0x0810),	/* "CLKOUT_CON_BLK_PERIC1_CMU_PERIC1_CLKOUT0" */
	(0x15400000 + 0x0810),	/* "CLKOUT_CON_BLK_APM_CMU_APM_CLKOUT0" */
	(0x26040000 + 0x0810),	/* "CLKOUT_CON_BLK_CMU_CMU_TOP_CLKOUT0" */
	(0x29c00000 + 0x0810),	/* "CLKOUT_CON_BLK_CPUCL0_CMU_CPUCL0_CLKOUT0" */
	(0x29c00000 + 0x0814),	/* "CLKOUT_CON_BLK_CPUCL0_EMBEDDED_CMU_CPUCL0_CLKOUT0" */
	(0x27c00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x27d00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x27e00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x27f00000 + 0x0810),	/* "CLKOUT_CON_BLK_MIF_CMU_MIF_CLKOUT0" */
	(0x10010000 + 0x0810),	/* "CLKOUT_CON_BLK_MISC_CMU_MISC_CLKOUT0" */
	(0x13000000 + 0x0810),	/* "CLKOUT_CON_BLK_HSI2_CMU_HSI2_CLKOUT0" */
	(0x10800000 + 0x0810),	/* "CLKOUT_CON_BLK_PERIC0_CMU_PERIC0_CLKOUT0" */
	(0x10c00000 + 0x0810),	/* "CLKOUT_CON_BLK_PERIC1_CMU_PERIC1_CLKOUT0" */
};
#endif

static struct samsung_clk_provider *zuma_clk_provider;
/*
 * list of controller registers to be saved and restored during a
 * suspend/resume cycle.
 */
/* fixed rate clocks generated outside the soc */
struct samsung_fixed_rate zuma_fixed_rate_ext_clks[] = {
	FRATE(OSCCLK,
		"fin_pll",
		NULL,
		0,
		24576000),
};

/* HWACG VCLK */
struct init_vclk zuma_apm_hwacg_vclks[] = {
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

struct init_vclk zuma_top_hwacg_vclks[] = {
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

/* NOCL1B: AON bus(BUS0) */
struct init_vclk zuma_nocl1b_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_NOCL1B_NOC,
		MUX_CLKCMU_NOCL1B_NOC_USER,
		"UMUX_CLKCMU_NOCL1B_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(MUX_NOCL1B_NOC_OPTION1,
		MUX_CLK_NOCL1B_NOC_OPTION1,
		"MUX_NOCL1B_NOC_OPTION1",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

/* NOCL2AA+AB: M/M bus */
struct init_vclk zuma_nocl2a_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_NOCL2AA_NOC,
		MUX_CLKCMU_NOCL2AA_NOC_USER,
		"UMUX_CLKCMU_NOCL2AA_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_NOCL2AB_NOC,
		MUX_CLKCMU_NOCL2AB_NOC_USER,
		"UMUX_CLKCMU_NOCL2AB_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

/* NOCL1A: BUS2 */
struct init_vclk zuma_nocl1a_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_NOCL1A_NOC,
		MUX_CLKCMU_NOCL1A_NOC_USER,
		"UMUX_CLKCMU_NOCL1_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

/* CORE */
struct init_vclk zuma_nocl0_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_NOCL0_NOC,
		MUX_CLKCMU_NOCL0_NOC_USER,
		"UMUX_CLKCMU_NOCL0_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_eh_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_EH_NOC,
		MUX_CLKCMU_EH_NOC_USER,
		"UMUX_CLKCMU_EH_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_g3d_hwacg_vclks[] = {
	HWACG_VCLK(GATE_GPU,
		GPU_QCH,
		"GATE_GPU",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_dpuf_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_DPUF0_NOC,
		MUX_CLKCMU_DPUF0_NOC_USER,
		"UMUX_CLKCMU_DPUF0_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_DPUF1_NOC,
		MUX_CLKCMU_DPUF1_NOC_USER,
		"UMUX_CLKCMU_DPUF1_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_dpub_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_DPUB_NOC,
		MUX_CLKCMU_DPUB_NOC_USER,
		"UMUX_CLKCMU_DPUB_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_DPUB,
		DPUB_QCH,
		"GATE_DPUB",
		"UMUX_CLKCMU_DPUB_NOC",
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_g2d_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_G2D_G2D,
		MUX_CLKCMU_G2D_G2D_USER,
		"UMUX_CLKCMU_G2D_G2D",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_G2D_JPEG,
		MUX_CLKCMU_G2D_JPEG_USER,
		"UMUX_CLKCMU_G2D_JPEG",
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
		"UMUX_CLKCMU_G2D_JPEG",
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_hsi0_hwacg_vclks[] = {
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
	HWACG_VCLK(UMUX_CLKCMU_HSI0_USB32DRD,
		MUX_CLKCMU_HSI0_USB32DRD_USER,
		"UMUX_CLKCMU_HSI0_USB32DRD",
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
	HWACG_VCLK(MUX_HSI0_USB32DRD,
		MUX_CLK_HSI0_USB32DRD,
		"MUX_HSI0_USB32DRD",
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
	HWACG_VCLK(GATE_USB32DRD_LINK,
		USB32DRD_QCH_LINK,
		"GATE_USB32DRD_LINK",
		"MUX_HSI0_USB32DRD",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_HSI0_USI0_USI,
		USI0_HSI0_QCH,
		"GATE_HSI0_USI0_USI",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_HSI0_USI1_USI,
		USI1_HSI0_QCH,
		"GATE_HSI0_USI1_USI",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_HSI0_USI2_USI,
		USI2_HSI0_QCH,
		"GATE_HSI0_USI2_USI",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_HSI0_USI3_USI,
		USI3_HSI0_QCH,
		"GATE_HSI0_USI3_USI",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_HSI0_USI4_USI,
		USI4_HSI0_QCH,
		"GATE_HSI0_USI4_USI",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_hsi1_hwacg_vclks[] = {
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
	HWACG_VCLK(GATE_PCIE_GEN3_0_UDBG,
		PCIE_GEN3_0_QCH_UDBG,
		"GATE_PCIE_GEN3_0_UDBG",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3_0_AXI,
		PCIE_GEN3_0_QCH_AXI,
		"GATE_PCIE_GEN3_0_AXI",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3_0_APB,
		PCIE_GEN3_0_QCH_APB,
		"GATE_PCIE_GEN3_0_APB",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3_0_PCS_APB,
		PCIE_GEN3_0_QCH_PCS_APB,
		"GATE_PCIE_GEN3_0_PCS_APB",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3_0_PMA_APB,
		PCIE_GEN3_0_QCH_PMA_APB,
		"GATE_PCIE_GEN3_0_PMA_APB",
		"UMUX_CLKCMU_HSI1_NOC",
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_hsi2_hwacg_vclks[] = {
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
	HWACG_VCLK(GATE_PCIE_GEN3A_1_DBG,
		PCIE_GEN3A_1_QCH_DBG,
		"GATE_PCIE_GEN3A_1_DBG",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3A_1_AXI,
		PCIE_GEN3A_1_QCH_AXI,
		"GATE_PCIE_GEN3A_1_AXI",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3A_1_APB,
		PCIE_GEN3A_1_QCH_APB,
		"GATE_PCIE_GEN3A_1_APB",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3A_1_PCS_APB,
		PCIE_GEN3A_1_QCH_PCS_APB,
		"GATE_PCIE_GEN3A_1_PCS_APB",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3A_1_PMA_APB,
		PCIE_GEN3A_1_QCH_PMA_APB,
		"GATE_PCIE_GEN3A_1_PMA_APB",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3B_1_DBG,
		PCIE_GEN3B_1_QCH_UDBG,
		"GATE_PCIE_GEN3B_1_DBG",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3B_1_AXI,
		PCIE_GEN3B_1_QCH_AXI,
		"GATE_PCIE_GEN3B_1_AXI",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3B_1_APB,
		PCIE_GEN3B_1_QCH_APB,
		"GATE_PCIE_GEN3B_1_APB",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3B_1_PCS_APB,
		PCIE_GEN3B_1_QCH_PCS_APB,
		"GATE_PCIE_GEN3B_1_PCS_APB",
		"UMUX_CLKCMU_HSI2_NOC",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PCIE_GEN3B_1_PMA_APB,
		PCIE_GEN3B_1_QCH_PMA_APB,
		"GATE_PCIE_GEN3B_1_PMA_APB",
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

struct init_vclk zuma_ispfe_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_ISPFE,
		MUX_CLKCMU_ISPFE_NOC_USER,
		"UMUX_CLKCMU_ISPFE",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_rgbp_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_RGBP_RGBP,
		MUX_CLKCMU_RGBP_RGBP_USER,
		"UMUX_CLKCMU_RGBP_RGBP",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_RGBP_MCFP,
		MUX_CLKCMU_RGBP_MCFP_USER,
		"UMUX_CLKCMU_RGBP_MCFP",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_yuvp_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_YUVP,
		MUX_CLKCMU_YUVP_NOC_USER,
		"UMUX_CLKCMU_YUVP",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_tnr_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_TNR_MERGE,
		MUX_CLKCMU_TNR_MERGE_USER,
		"UMUX_CLKCMU_TNR_MERGE",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_TNR_ALIGN,
		MUX_CLKCMU_TNR_ALIGN_USER,
		"UMUX_CLKCMU_TNR_ALIGN",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_mcsc_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_MCSC,
		MUX_CLKCMU_MCSC_NOC_USER,
		"UMUX_CLKCMU_MCSC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_gdc_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_GDC_LME,
		MUX_CLKCMU_GDC_LME_USER,
		"UMUX_CLKCMU_GDC_LME",
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

struct init_vclk zuma_gse_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_GSE,
		MUX_CLKCMU_GSE_NOC_USER,
		"UMUX_CLKCMU_GSE",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_mfc_hwacg_vclks[] = {
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

struct init_vclk zuma_mif_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_MIF_DDRPHY2X,
		CLKMUX_MIF_DDRPHY2X,
		"UMUX_MIF_DDRPHY2X",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

struct init_vclk zuma_misc_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_MISC_NOC,
		MUX_CLKCMU_MISC_NOC_USER,
		"UMUX_CLKCMU_MISC_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(UMUX_CLKCMU_MISC_SC,
		MUX_CLKCMU_MISC_SC_USER,
		"UMUX_CLKCMU_MISC_SC",
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
		"GATE_PDMA0",
		"DOUT_CLK_MISC_NOCP",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK(GATE_PDMA1,
		PDMA1_QCH,
		"GATE_PDMA1",
		"DOUT_CLK_MISC_NOCP",
		0,
		VCLK_GATE,
		NULL),
	HWACG_VCLK_QACTIVE(ATCLK,
		"ATCLK",
		"DOUT_CLK_MISC_NOCP",
		0,
		VCLK_QACTIVE,
		NULL,
		0x2a00c02c,
		0x1,
		0x1),
};

struct init_vclk zuma_peric0_hwacg_vclks[] = {
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
	HWACG_VCLK(GATE_PERIC0_TOP0_USI14_USI,
		USI14_USI_QCH,
		"GATE_PERIC0_TOP0_USI14_USI",
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
};

struct init_vclk zuma_peric1_hwacg_vclks[] = {
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
	HWACG_VCLK(GATE_PERIC1_TOP0_USI15_USI,
		USI15_USI_QCH,
		"GATE_PERIC1_TOP0_USI15_USI",
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
};

struct init_vclk zuma_tpu_hwacg_vclks[] = {
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

struct init_vclk zuma_bw_hwacg_vclks[] = {
	HWACG_VCLK(UMUX_CLKCMU_BW_NOC,
		MUX_CLKCMU_BW_NOC_USER,
		"UMUX_CLKCMU_BW_NOC",
		NULL,
		0,
		VCLK_GATE,
		NULL),
};

/* Special VCLK */
struct init_vclk zuma_apm_vclks[] = {
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

struct init_vclk zuma_nocl1b_vclks[] = {
	VCLK(DOUT_CLK_NOCL1B_NOCP,
		DIV_CLK_NOCL1B_NOCP,
		"DOUT_CLK_NOCL1B_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_nocl2a_vclks[] = {
	VCLK(DOUT_CLK_NOCL2AA_NOCP,
		DIV_CLK_NOCL2AA_NOCP,
		"DOUT_CLK_NOCL2AA_NOCP",
		0,
		0,
		NULL),
	VCLK(DOUT_CLK_NOCL2AB_NOCP,
		DIV_CLK_NOCL2AB_NOCP,
		"DOUT_CLK_NOCL2AB_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_nocl1a_vclks[] = {
	VCLK(DOUT_CLK_NOCL1A_NOCP,
		DIV_CLK_NOCL1A_NOCP,
		"DOUT_CLK_NOCL1A_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_eh_vclks[] = {
	VCLK(DOUT_CLK_EH_NOCP,
		DIV_CLK_EH_NOCP,
		"DOUT_CLK_EH_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_dpuf_vclks[] = {
	VCLK(DOUT_CLK_DPUF0_NOCP,
		DIV_CLK_DPUF0_NOCP,
		"DOUT_CLK_DPUF0_NOCP",
		0,
		0,
		NULL),
	VCLK(DOUT_CLK_DPUF1_NOCP,
		DIV_CLK_DPUF1_NOCP,
		"DOUT_CLK_DPUF1_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_dpub_vclks[] = {
	VCLK(DOUT_CLK_DPUB_NOCP,
		DIV_CLK_DPUB_NOCP,
		"DOUT_CLK_DPUB_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_g2d_vclks[] = {
	VCLK(DOUT_CLK_G2D_NOCP,
		DIV_CLK_G2D_NOCP,
		"DOUT_CLK_G2D_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_hsi0_vclks[] = {
	VCLK(DOUT_CLK_HSI0_USB32DRD,
		DIV_CLK_HSI0_USB32DRD,
		"DOUT_CLK_HSI0_USB32DRD",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_HSI0_USI0_USI,
		VCLK_DIV_CLK_HSI0_USI0,
		"VDOUT_CLK_HSI0_USI0_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_HSI0_USI1_USI,
		VCLK_DIV_CLK_HSI0_USI1,
		"VDOUT_CLK_HSI0_USI1_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_HSI0_USI2_USI,
		VCLK_DIV_CLK_HSI0_USI2,
		"VDOUT_CLK_HSI0_USI2_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_HSI0_USI3_USI,
		VCLK_DIV_CLK_HSI0_USI3,
		"VDOUT_CLK_HSI0_USI3_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_HSI0_USI4_USI,
		VCLK_DIV_CLK_HSI0_USI4,
		"VDOUT_CLK_HSI0_USI4_USI",
		0,
		0,
		NULL),
	VCLK(VDOUT_CLK_TOP_HSI0_NOC,
		CLKCMU_HSI0_NOC,
		"VDOUT_CLK_TOP_HSI0_NOC",
		0,
		0,
		NULL),
	VCLK(UMUX_CLKCMU_HSI0_DPOSC_USER,
		MUX_CLKCMU_HSI0_DPOSC_USER,
		"UMUX_CLKCMU_HSI0_DPOSC_USER",
		0,
		0,
		NULL),
};

struct init_vclk zuma_hsi2_vclks[] = {
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

struct init_vclk zuma_ispfe_vclks[] = {
	VCLK(DOUT_CLK_ISPFE_NOCP,
		DIV_CLK_ISPFE_NOCP,
		"DOUT_CLK_ISPFE_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_yuvp_vclks[] = {
	VCLK(DOUT_CLK_YUVP_NOCP,
		DIV_CLK_YUVP_NOCP,
		"DOUT_CLK_YUVP_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_gse_vclks[] = {
	VCLK(DOUT_CLK_GSE_NOCP,
		DIV_CLK_GSE_NOCP,
		"DOUT_CLK_GSE_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_rgbp_vclks[] = {
	VCLK(DOUT_CLK_RGBP_NOCP,
		DIV_CLK_RGBP_NOCP,
		"DOUT_CLK_RGBP_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_tnr_vclks[] = {
	VCLK(DOUT_CLK_TNR_NOCP,
		DIV_CLK_TNR_NOCP,
		"DOUT_CLK_TNR_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_mcsc_vclks[] = {
	VCLK(DOUT_CLK_MCSC_NOCP,
		DIV_CLK_MCSC_NOCP,
		"DOUT_CLK_MCSC_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_gdc_vclks[] = {
	VCLK(DOUT_CLK_GDC_NOCP,
		DIV_CLK_GDC_NOCP,
		"DOUT_CLK_GDC_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_mfc_vclks[] = {
	VCLK(DOUT_CLK_MFC_NOCP,
		DIV_CLK_MFC_NOCP,
		"DOUT_CLK_MFC_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_misc_vclks[] = {
	VCLK(DOUT_CLK_MISC_NOCP,
		DIV_CLK_MISC_NOCP,
		"DOUT_CLK_MISC_NOCP",
		0,
		0,
		NULL),
};

struct init_vclk zuma_top_vclks[] = {
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

struct init_vclk zuma_peric0_vclks[] = {
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
	VCLK(VDOUT_CLK_PERIC0_USI14_USI,
		VCLK_DIV_CLK_PERIC0_USI14_USI,
		"VDOUT_CLK_PERIC0_USI14_USI",
		0,
		0,
		NULL),
};

struct init_vclk zuma_peric1_vclks[] = {
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
	VCLK(VDOUT_CLK_PERIC1_USI15_USI,
		VCLK_DIV_CLK_PERIC1_USI15_USI,
		"VDOUT_CLK_PERIC1_USI15_USI",
		0,
		0,
		NULL),
};

struct init_vclk zuma_tpu_vclks[] = {
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

struct init_vclk zuma_bw_vclks[] = {
	VCLK(DOUT_CLK_BW_NOCP,
		DIV_CLK_BW_NOCP,
		"DOUT_CLK_BW_NOCP",
		0,
		0,
		NULL),
};

static struct init_vclk zuma_clkout_vclks[] = {
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

static const struct of_device_id ext_clk_match[] = {
	{.compatible = "samsung,zuma-oscclk", .data = (void *)0},
	{},
};

#if IS_ENABLED(CONFIG_DEBUG_FS)
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
	iounmap(addr);

	return 0;
}

static int clkout_val_set(void *data, u64 val)
{
	u32 __iomem *addr;
	struct samsung_clk_provider *scp = data;

	addr = ioremap(scp->clkout_addr, SZ_4);
	iowrite32((u32)val, addr);
	iounmap(addr);

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

static int zuma_clock_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	void __iomem *reg_base;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct dentry *root;
#endif

	if (np) {
		reg_base = of_iomap(np, 0);
		if (!reg_base)
			panic("%s: failed to map registers\n", __func__);
	} else {
		panic("%s: unable to determine soc\n", __func__);
	}

	zuma_clk_provider = samsung_clk_init(np, reg_base, CLK_NR_CLKS);
	if (!zuma_clk_provider)
		panic("%s: unable to allocate context.\n", __func__);

	samsung_register_of_fixed_ext(zuma_clk_provider,
					zuma_fixed_rate_ext_clks,
					ARRAY_SIZE(zuma_fixed_rate_ext_clks),
					ext_clk_match);

	/* register HWACG vclk */
	samsung_register_vclk(zuma_clk_provider,
				zuma_apm_hwacg_vclks,
				ARRAY_SIZE(zuma_apm_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_top_hwacg_vclks,
				ARRAY_SIZE(zuma_top_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_nocl1a_hwacg_vclks,
				ARRAY_SIZE(zuma_nocl1a_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_nocl1b_hwacg_vclks,
				ARRAY_SIZE(zuma_nocl1b_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_nocl2a_hwacg_vclks,
				ARRAY_SIZE(zuma_nocl2a_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_nocl0_hwacg_vclks,
				ARRAY_SIZE(zuma_nocl0_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_eh_hwacg_vclks,
				ARRAY_SIZE(zuma_eh_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_g3d_hwacg_vclks,
				ARRAY_SIZE(zuma_g3d_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_dpuf_hwacg_vclks,
				ARRAY_SIZE(zuma_dpuf_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_dpub_hwacg_vclks,
				ARRAY_SIZE(zuma_dpub_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_g2d_hwacg_vclks,
				ARRAY_SIZE(zuma_g2d_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_hsi0_hwacg_vclks,
				ARRAY_SIZE(zuma_hsi0_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_hsi1_hwacg_vclks,
				ARRAY_SIZE(zuma_hsi1_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_hsi2_hwacg_vclks,
				ARRAY_SIZE(zuma_hsi2_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_ispfe_hwacg_vclks,
				ARRAY_SIZE(zuma_ispfe_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_rgbp_hwacg_vclks,
				ARRAY_SIZE(zuma_rgbp_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_yuvp_hwacg_vclks,
				ARRAY_SIZE(zuma_yuvp_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_tnr_hwacg_vclks,
				ARRAY_SIZE(zuma_tnr_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_mcsc_hwacg_vclks,
				ARRAY_SIZE(zuma_mcsc_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_gdc_hwacg_vclks,
				ARRAY_SIZE(zuma_gdc_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_gse_hwacg_vclks,
				ARRAY_SIZE(zuma_gse_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_mfc_hwacg_vclks,
				ARRAY_SIZE(zuma_mfc_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_mif_hwacg_vclks,
				ARRAY_SIZE(zuma_mif_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_misc_hwacg_vclks,
				ARRAY_SIZE(zuma_misc_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_peric0_hwacg_vclks,
				ARRAY_SIZE(zuma_peric0_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_peric1_hwacg_vclks,
				ARRAY_SIZE(zuma_peric1_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_tpu_hwacg_vclks,
				ARRAY_SIZE(zuma_tpu_hwacg_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_bw_hwacg_vclks,
				ARRAY_SIZE(zuma_bw_hwacg_vclks));

	/* register special vclk */
	samsung_register_vclk(zuma_clk_provider,
				zuma_apm_vclks,
				ARRAY_SIZE(zuma_apm_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_nocl1b_vclks,
				ARRAY_SIZE(zuma_nocl1b_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_nocl2a_vclks,
				ARRAY_SIZE(zuma_nocl2a_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_nocl1a_vclks,
				ARRAY_SIZE(zuma_nocl1a_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_eh_vclks,
				ARRAY_SIZE(zuma_eh_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_dpuf_vclks,
				ARRAY_SIZE(zuma_dpuf_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_dpub_vclks,
				ARRAY_SIZE(zuma_dpub_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_g2d_vclks,
				ARRAY_SIZE(zuma_g2d_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_hsi0_vclks,
				ARRAY_SIZE(zuma_hsi0_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_ispfe_vclks,
				ARRAY_SIZE(zuma_ispfe_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_yuvp_vclks,
				ARRAY_SIZE(zuma_yuvp_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_gse_vclks,
				ARRAY_SIZE(zuma_gse_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_rgbp_vclks,
				ARRAY_SIZE(zuma_rgbp_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_tnr_vclks,
				ARRAY_SIZE(zuma_tnr_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_mcsc_vclks,
				ARRAY_SIZE(zuma_mcsc_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_gdc_vclks,
				ARRAY_SIZE(zuma_gdc_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_mfc_vclks,
				ARRAY_SIZE(zuma_mfc_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_misc_vclks,
				ARRAY_SIZE(zuma_misc_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_top_vclks,
				ARRAY_SIZE(zuma_top_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_hsi2_vclks,
				ARRAY_SIZE(zuma_hsi2_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_peric0_vclks,
				ARRAY_SIZE(zuma_peric0_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_peric1_vclks,
				ARRAY_SIZE(zuma_peric1_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_tpu_vclks,
				ARRAY_SIZE(zuma_tpu_vclks));
	samsung_register_vclk(zuma_clk_provider,
				zuma_bw_vclks,
				ARRAY_SIZE(zuma_bw_vclks));

	/* CLKOUT */
	samsung_register_vclk(zuma_clk_provider,
				zuma_clkout_vclks,
				ARRAY_SIZE(zuma_clkout_vclks));

	clk_register_fixed_factor(NULL,
				"pwm-clock",
				"fin_pll",
				CLK_SET_RATE_PARENT,
				1,
				1);

	samsung_clk_of_add_provider(np, zuma_clk_provider);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	zuma_clk_provider->clkout_addr = clkout_addresses[0];
	root = debugfs_create_dir("xclkout", NULL);
	debugfs_create_file("pad_clkout0", 0644, root, zuma_clk_provider,
			    &pad_clkout0_fops);
	debugfs_create_file("pad_clkout1", 0644, root, zuma_clk_provider,
			    &pad_clkout1_fops);
	debugfs_create_file("clkout_addr", 0644, root, zuma_clk_provider,
			    &clkout_addr_fops);
	debugfs_create_file("clkout_val", 0644, root, zuma_clk_provider,
			    &clkout_val_fops);
#endif

	pr_info("ZUMA: Clock setup completed\n");

	return 0;
}

static const struct of_device_id of_exynos_clock_match[] = {
	{ .compatible = "samsung,zuma-clock", },
	{ },
};
MODULE_DEVICE_TABLE(of, of_exynos_clock_match);

static const struct platform_device_id exynos_clock_ids[] = {
	{ "zuma-clock", },
	{ }
};

static struct platform_driver zuma_clock_driver = {
	.driver = {
		.name = "zuma_clock",
		.of_match_table = of_exynos_clock_match,
	},
	.probe		= zuma_clock_probe,
	.id_table	= exynos_clock_ids,
};

module_platform_driver(zuma_clock_driver);

MODULE_LICENSE("GPL");
