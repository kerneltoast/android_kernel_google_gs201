// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 */

#include "../cmucal.h"
#include "cmucal-node.h"
#include "cmucal-vclk.h"
#include "cmucal-vclklut.h"

/* DVFS VCLK -> Clock Node List */
enum clk_id cmucal_vclk_vdd_int[] = {
	CLKCMU_MFC_MFC,
	MUX_CLKCMU_MFC_MFC,
	CLKCMU_G2D_G2D,
	MUX_CLKCMU_G2D_G2D,
	CLKCMU_ISPFE_NOC,
	MUX_CLKCMU_ISPFE_NOC,
	CLKCMU_CPUCL0_DSU_SWITCH,
	MUX_CLKCMU_CPUCL0_DSU_SWITCH,
	CLKCMU_RGBP_RGBP,
	MUX_CLKCMU_RGBP_RGBP,
	CLKCMU_MCSC_NOC,
	MUX_CLKCMU_MCSC_NOC,
	CLKCMU_G2D_JPEG,
	MUX_CLKCMU_G2D_JPEG,
	CLKCMU_BW_NOC,
	MUX_CLKCMU_BW_NOC,
	CLKCMU_MISC_NOC,
	MUX_CLKCMU_MISC_NOC,
	DIV_CLK_CMU_CMUREF,
	CLKCMU_MIF_NOCP,
	MUX_CLKCMU_MIF_NOCP,
	CLKCMU_CPUCL1_SWITCH,
	MUX_CLKCMU_CPUCL1_SWITCH,
	CLKCMU_GSE_NOC,
	MUX_CLKCMU_GSE_NOC,
	CLKCMU_TNR_MERGE,
	MUX_CLKCMU_TNR_MERGE,
	CLKCMU_NOCL2AA_NOC,
	MUX_CLKCMU_NOCL2AA_NOC,
	CLKCMU_NOCL1B_NOC,
	MUX_CLKCMU_NOCL1B_NOC,
	CLKCMU_YUVP_NOC,
	MUX_CLKCMU_YUVP_NOC,
	CLKCMU_GDC_GDC0,
	MUX_CLKCMU_GDC_GDC0,
	CLKCMU_GDC_GDC1,
	MUX_CLKCMU_GDC_GDC1,
	CLKCMU_CPUCL2_SWITCH,
	MUX_CLKCMU_CPUCL2_SWITCH,
	CLKCMU_GDC_LME,
	MUX_CLKCMU_GDC_LME,
	CLKCMU_MISC_SC,
	MUX_CLKCMU_MISC_SC,
	DIV_CLK_EH_NOCP,
	CLKCMU_EH_NOC,
	MUX_CLKCMU_EH_NOC,
	CLKCMU_AUR_NOC,
	MUX_CLKCMU_AUR_NOC,
	CLKCMU_AUR_AURCTL,
	MUX_CLKCMU_AUR_AURCTL,
	CLKCMU_RGBP_MCFP,
	MUX_CLKCMU_RGBP_MCFP,
	CLKCMU_NOCL2AB_NOC,
	MUX_CLKCMU_NOCL2AB_NOC,
	MUX_CLKCMU_MIF_SWITCH,
	PLL_LF_MIF,
	CLKCMU_CPUCL0_CPU_SWITCH,
	MUX_CLKCMU_CPUCL0_CPU_SWITCH,
	CLKCMU_CPUCL0_BCI_SWITCH,
	MUX_CLKCMU_CPUCL0_BCI_SWITCH,
	MUX_CLKCMU_HSI0_PERI,
	CLKCMU_TNR_ALIGN,
	MUX_CLKCMU_TNR_ALIGN,
	CLKCMU_CPUCL0_DBG,
	MUX_CLKCMU_CPUCL0_DBG,
	DIV_CLK_HSI2_NOCP,
	CLKCMU_HSI2_NOC,
	MUX_CLKCMU_HSI2_NOC,
	CLKCMU_DPUF0_NOC,
	MUX_CLKCMU_DPUF0_NOC,
	MUX_CLKCMU_HSI0_NOC,
	CLKCMU_NOCL1A_NOC,
	MUX_CLKCMU_NOCL1A_NOC,
	CLKCMU_DPUB_NOC,
	MUX_CLKCMU_DPUB_NOC,
	CLKCMU_DPUF1_NOC,
	MUX_CLKCMU_DPUF1_NOC,
	CLKCMU_HSI1_NOC,
	MUX_CLKCMU_HSI1_NOC,
	CLKCMU_CIS_CLK0,
	CLKCMU_CIS_CLK1,
	CLKCMU_CIS_CLK2,
	CLKCMU_CIS_CLK3,
	CLKCMU_HSI2_UFS_EMBD,
	MUX_CLKCMU_HSI2_UFS_EMBD,
	CLKCMU_HSI2_PCIE,
	MUX_CLKCMU_HSI2_PCIE,
	CLKCMU_PERIC0_IP,
	CLKCMU_PERIC1_IP,
	CLKCMU_HSI1_PCIE,
	MUX_CLKCMU_HSI1_PCIE,
	CLKCMU_CIS_CLK4,
	MUX_NOCL1A_CMUREF,
	DIV_CLKCMU_CMU_BOOST,
	CLKCMU_CIS_CLK5,
	CLKCMU_CIS_CLK6,
	CLKCMU_CIS_CLK7,
	CLKCMU_HSI2_MMC_CARD,
	MUX_CLKCMU_HSI2_MMC_CARD,
};
enum clk_id cmucal_vclk_vdd_mif[] = {
	PLL_MIF_MAIN,
	PLL_MIF_SUB,
	DIV_CLK_NOCL0_NOCP,
	PLL_NOCL0,
};
enum clk_id cmucal_vclk_vdd_g3d[] = {
	MUX_CLK_G3D_TOP,
	PLL_G3D,
	PLL_G3D_L2,
};
enum clk_id cmucal_vclk_vdd_cam[] = {
	PLL_AUR,
	DIV_CLK_ISPFE_DCPHY,
	DIV_CLK_MFC_NOCP,
};
enum clk_id cmucal_vclk_vdd_cpucl0[] = {
	DIV_CLK_CLUSTER0_ACLK_MAIN,
	PLL_DSU,
	PLL_CPUCL0,
	PLL_BCI,
};
enum clk_id cmucal_vclk_vdd_cpucl1[] = {
	PLL_CPUCL1,
};
enum clk_id cmucal_vclk_vdd_tpu[] = {
	DIV_CLK_TPU_TPUCTL_DBG,
	PLL_TPU,
};
enum clk_id cmucal_vclk_vdd_cpucl2[] = {
	PLL_CPUCL2,
};
/* SPECIAL VCLK -> Clock Node List */
enum clk_id cmucal_vclk_mux_cmu_cmuref[] = {
	MUX_CMU_CMUREF,
	MUX_CLKCMU_TOP_BOOST_OPTION1,
	MUX_CLKCMU_TOP_CMUREF,
};
enum clk_id cmucal_vclk_mux_cpucl0_cmuref[] = {
	MUX_CPUCL0_CMUREF,
};
enum clk_id cmucal_vclk_mux_cpucl1_cmuref[] = {
	MUX_CPUCL1_CMUREF,
};
enum clk_id cmucal_vclk_mux_cpucl2_cmuref[] = {
	MUX_CPUCL2_CMUREF,
};
enum clk_id cmucal_vclk_mux_clk_hsi0_usb20_ref[] = {
	MUX_CLK_HSI0_USB20_REF,
	MUX_CLK_HSI0_USB32DRD,
	DIV_CLK_HSI0_USB,
	PLL_USB,
	DIV_CLK_HSI0_USB32DRD,
	MUX_CLK_HSI0_RTCCLK,
};
enum clk_id cmucal_vclk_clkcmu_hsi0_usb32drd[] = {
	CLKCMU_HSI0_USB32DRD,
	MUX_CLKCMU_HSI0_USB32DRD,
};
enum clk_id cmucal_vclk_mux_mif_cmuref[] = {
	MUX_MIF_CMUREF,
};
enum clk_id cmucal_vclk_mux_nocl0_cmuref[] = {
	MUX_NOCL0_CMUREF,
};
enum clk_id cmucal_vclk_mux_nocl1b_cmuref[] = {
	MUX_NOCL1B_CMUREF,
};
enum clk_id cmucal_vclk_mux_nocl2aa_cmuref[] = {
	MUX_NOCL2AA_CMUREF,
};
enum clk_id cmucal_vclk_mux_nocl2ab_cmuref[] = {
	MUX_NOCL2AB_CMUREF,
};
enum clk_id cmucal_vclk_clkcmu_dpub_dsim[] = {
	CLKCMU_DPUB_DSIM,
	MUX_CLKCMU_DPUB_DSIM,
};
enum clk_id cmucal_vclk_clkcmu_hsi0_dpgtc[] = {
	CLKCMU_HSI0_DPGTC,
	MUX_CLKCMU_HSI0_DPGTC,
	CLKCMU_HSI0_DPOSC,
	MUX_CLKCMU_HSI0_DPOSC,
};
enum clk_id cmucal_vclk_div_clk_apm_usi0_usi[] = {
	DIV_CLK_APM_USI0_USI,
};
enum clk_id cmucal_vclk_div_clk_apm_usi0_uart[] = {
	DIV_CLK_APM_USI0_UART,
};
enum clk_id cmucal_vclk_div_clk_apm_usi1_uart[] = {
	DIV_CLK_APM_USI1_UART,
};
enum clk_id cmucal_vclk_div_clk_apm_i3c_pmic[] = {
	DIV_CLK_APM_I3C_PMIC,
};
enum clk_id cmucal_vclk_clk_aur_add_ch_clk[] = {
	CLK_AUR_ADD_CH_CLK,
};
enum clk_id cmucal_vclk_mux_clkcmu_cis_clk0[] = {
	MUX_CLKCMU_CIS_CLK0,
};
enum clk_id cmucal_vclk_mux_clkcmu_cis_clk1[] = {
	MUX_CLKCMU_CIS_CLK1,
};
enum clk_id cmucal_vclk_mux_clkcmu_cis_clk2[] = {
	MUX_CLKCMU_CIS_CLK2,
};
enum clk_id cmucal_vclk_mux_clkcmu_cis_clk3[] = {
	MUX_CLKCMU_CIS_CLK3,
};
enum clk_id cmucal_vclk_mux_clkcmu_cis_clk4[] = {
	MUX_CLKCMU_CIS_CLK4,
};
enum clk_id cmucal_vclk_mux_clkcmu_cis_clk5[] = {
	MUX_CLKCMU_CIS_CLK5,
};
enum clk_id cmucal_vclk_mux_clkcmu_cis_clk6[] = {
	MUX_CLKCMU_CIS_CLK6,
};
enum clk_id cmucal_vclk_mux_clkcmu_cis_clk7[] = {
	MUX_CLKCMU_CIS_CLK7,
};
enum clk_id cmucal_vclk_div_clk_cpucl0_cmuref[] = {
	DIV_CLK_CPUCL0_CMUREF,
};
enum clk_id cmucal_vclk_div_clk_cpucl0_add_ch_clk[] = {
	DIV_CLK_CPUCL0_ADD_CH_CLK,
};
enum clk_id cmucal_vclk_div_clk_cpucl1_cmuref[] = {
	DIV_CLK_CPUCL1_CMUREF,
};
enum clk_id cmucal_vclk_div_clk_cpucl2_cmuref[] = {
	DIV_CLK_CPUCL2_CMUREF,
};
enum clk_id cmucal_vclk_clk_g3d_add_ch_clk[] = {
	CLK_G3D_ADD_CH_CLK,
};
enum clk_id cmucal_vclk_div_clk_gsacore_spi_fps[] = {
	DIV_CLK_GSACORE_SPI_FPS,
};
enum clk_id cmucal_vclk_div_clk_gsacore_spi_gsc[] = {
	DIV_CLK_GSACORE_SPI_GSC,
};
enum clk_id cmucal_vclk_div_clk_gsacore_uart[] = {
	DIV_CLK_GSACORE_UART,
};
enum clk_id cmucal_vclk_div_clk_hsi0_usi2[] = {
	DIV_CLK_HSI0_USI2,
	MUX_CLK_HSI0_USI2,
};
enum clk_id cmucal_vclk_clkcmu_hsi0_peri[] = {
	CLKCMU_HSI0_PERI,
};
enum clk_id cmucal_vclk_div_clk_hsi0_usi0[] = {
	DIV_CLK_HSI0_USI0,
	MUX_CLK_HSI0_USI0,
};
enum clk_id cmucal_vclk_div_clk_hsi0_usi1[] = {
	DIV_CLK_HSI0_USI1,
	MUX_CLK_HSI0_USI1,
};
enum clk_id cmucal_vclk_div_clk_hsi0_usi3[] = {
	DIV_CLK_HSI0_USI3,
	MUX_CLK_HSI0_USI3,
};
enum clk_id cmucal_vclk_div_clk_hsi0_usi4[] = {
	DIV_CLK_HSI0_USI4,
	MUX_CLK_HSI0_USI4,
};
enum clk_id cmucal_vclk_div_clk_slc_dclk[] = {
	DIV_CLK_SLC_DCLK,
};
enum clk_id cmucal_vclk_div_clk_slc1_dclk[] = {
	DIV_CLK_SLC1_DCLK,
};
enum clk_id cmucal_vclk_div_clk_slc2_dclk[] = {
	DIV_CLK_SLC2_DCLK,
};
enum clk_id cmucal_vclk_div_clk_slc3_dclk[] = {
	DIV_CLK_SLC3_DCLK,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi6_usi[] = {
	DIV_CLK_PERIC0_USI6_USI,
	MUX_CLKCMU_PERIC0_USI6_USI_USER,
};
enum clk_id cmucal_vclk_mux_clkcmu_peric0_ip[] = {
	MUX_CLKCMU_PERIC0_IP,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi3_usi[] = {
	DIV_CLK_PERIC0_USI3_USI,
	MUX_CLKCMU_PERIC0_USI3_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi4_usi[] = {
	DIV_CLK_PERIC0_USI4_USI,
	MUX_CLKCMU_PERIC0_USI4_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi5_usi[] = {
	DIV_CLK_PERIC0_USI5_USI,
	MUX_CLKCMU_PERIC0_USI5_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi14_usi[] = {
	DIV_CLK_PERIC0_USI14_USI,
	MUX_CLKCMU_PERIC0_USI14_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi1_usi[] = {
	DIV_CLK_PERIC0_USI1_USI,
	MUX_CLKCMU_PERIC0_USI1_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi0_uart[] = {
	DIV_CLK_PERIC0_USI0_UART,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi2_usi[] = {
	DIV_CLK_PERIC0_USI2_USI,
	MUX_CLKCMU_PERIC0_USI2_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric1_usi11_usi[] = {
	DIV_CLK_PERIC1_USI11_USI,
	MUX_CLKCMU_PERIC1_USI11_USI_USER,
};
enum clk_id cmucal_vclk_mux_clkcmu_peric1_ip[] = {
	MUX_CLKCMU_PERIC1_IP,
};
enum clk_id cmucal_vclk_div_clk_peric1_i3c[] = {
	DIV_CLK_PERIC1_I3C,
};
enum clk_id cmucal_vclk_div_clk_peric1_usi12_usi[] = {
	DIV_CLK_PERIC1_USI12_USI,
	MUX_CLKCMU_PERIC1_USI12_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric1_usi0_usi[] = {
	DIV_CLK_PERIC1_USI0_USI,
	MUX_CLKCMU_PERIC1_USI0_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric1_usi9_usi[] = {
	DIV_CLK_PERIC1_USI9_USI,
	MUX_CLKCMU_PERIC1_USI9_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric1_usi10_usi[] = {
	DIV_CLK_PERIC1_USI10_USI,
	MUX_CLKCMU_PERIC1_USI10_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric1_usi13_usi[] = {
	DIV_CLK_PERIC1_USI13_USI,
	MUX_CLKCMU_PERIC1_USI13_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric1_usi15_usi[] = {
	DIV_CLK_PERIC1_USI15_USI,
	MUX_CLKCMU_PERIC1_USI15_USI_USER,
};
enum clk_id cmucal_vclk_clk_tpu_add_ch_clk[] = {
	CLK_TPU_ADD_CH_CLK,
};
/* COMMON VCLK -> Clock Node List */
enum clk_id cmucal_vclk_blk_cmu[] = {
	PLL_SHARED0_DIV5,
	CLKCMU_TPU_NOC,
	MUX_CLKCMU_TPU_NOC,
	CLKCMU_G3D_NOCD,
	MUX_CLKCMU_G3D_NOCD,
	CLKCMU_PERIC0_NOC,
	MUX_CLKCMU_PERIC0_NOC,
	CLKCMU_HSI0_NOC,
	MUX_CLKCMU_CMU_BOOST_OPTION1,
	MUX_CLKCMU_CMU_BOOST,
	CLKCMU_PERIC1_NOC,
	MUX_CLKCMU_PERIC1_NOC,
	CLKCMU_G3D_TRACE,
	MUX_CLKCMU_G3D_TRACE,
	PLL_SHARED0_D1,
	PLL_SHARED1_D1,
	MUX_CLKCMU_HSI0_USBDPDBG,
	PLL_SHARED2_D1,
	PLL_SHARED3_D1,
	PLL_SPARE_D1,
};
enum clk_id cmucal_vclk_blk_gsactrl[] = {
	DIV_CLK_GSACTRL_NOCP_LH,
	DIV_CLK_GSACTRL_NOCP,
	DIV_CLK_GSACTRL_NOCD,
	MUX_CLK_GSACTRL_NOC,
	MUX_CLK_GSACTRL_SC,
	PLL_GSA,
	MUX_CLKCMU_GSA_FUNC,
	MUX_CLKCMU_GSA_FUNCSRC,
};
enum clk_id cmucal_vclk_blk_s2d[] = {
	DIV_CLK_S2D_CORE_LH,
	MUX_CLK_S2D_CORE,
	PLL_MIF_S2D,
};
enum clk_id cmucal_vclk_blk_apm[] = {
	DIV_CLK_APM_BOOST,
	DIV_CLK_APM_NOC_LH,
	DIV_CLK_APM_NOC,
	MUX_CLKCMU_APM_FUNC,
	MUX_CLKCMU_APM_FUNCSRC,
};
enum clk_id cmucal_vclk_blk_cpucl0[] = {
	DIV_CLK_CLUSTER0_GICCLK_LH,
	DIV_CLK_CLUSTER0_GICCLK,
	DIV_CLK_CLUSTER0_PCLKDBG,
	DIV_CLK_CLUSTER0_PERIPHCLK,
	DIV_CLK_CLUSTER0_PPUCLK,
	DIV_CLK_CLUSTER0_ACLK_NON_MAIN_LH,
	DIV_CLK_CLUSTER0_ACLK_NON_MAIN,
	DIV_CLK_CLUSTER0_ACLK_DBG,
	CLK_CPUCL0_BCI_SHORTSTOP_SYNC,
	DIV_CLK_CPUCL0_PCLK_LH,
	DIV_CLK_CPUCL0_PCLK,
	MUX_CLK_CPUCL0_DSU,
	MUX_CLK_CPUCL0_CPU,
	MUX_CLK_CPUCL0_BCI,
	DIV_CLK_CPUCL0_DBG_PCLKDBG,
	DIV_CLK_CPUCL0_DBG_NOC_LH,
	DIV_CLK_CPUCL0_DBG_NOC,
	DIV_CLK_CPUCL0_DBG_ATCLK_LH,
};
enum clk_id cmucal_vclk_blk_cpucl1[] = {
	MUX_CLK_CPUCL1_CPU,
};
enum clk_id cmucal_vclk_blk_cpucl2[] = {
	MUX_CLK_CPUCL2_CPU,
};
enum clk_id cmucal_vclk_blk_gsacore[] = {
	DIV_CLK_GSACORE_NOCD,
	DIV_CLK_GSACORE_NOCP,
	DIV_CLK_GSACORE_CPU_LH,
	MUX_CLK_GSACORE_CPU_HCH,
	DIV_CLK_GSACORE_NOC,
	DIV_CLK_GSACORE_SC,
};
enum clk_id cmucal_vclk_blk_hsi0[] = {
	DIV_CLK_HSI0_NOC_LH,
	DIV_CLK_HSI0_EUSB,
	MUX_CLK_HSI0_NOC,
	DIV_CLK_HSI0_I3C,
	MUX_CLK_HSI0_I3C,
};
enum clk_id cmucal_vclk_blk_hsi1[] = {
	DIV_CLK_HSI1_NOC_LH,
	DIV_CLK_HSI1_NOCP,
	MUX_CLK_HSI1_NOC,
	CLK_HSI1_ALT,
};
enum clk_id cmucal_vclk_blk_nocl0[] = {
	DIV_CLK_NOCL0_NOCD_LH,
	DIV_CLK_NOCL0_NOCP_LH,
	MUX_CLK_NOCL0_NOC_OPTION1,
};
enum clk_id cmucal_vclk_blk_nocl1b[] = {
	DIV_CLK_NOCL1B_NOCP_LH,
	DIV_CLK_NOCL1B_NOCP,
	DIV_CLK_NOCL1B_NOCD_LH,
	MUX_CLK_NOCL1B_NOC_OPTION1,
};
enum clk_id cmucal_vclk_blk_aoc[] = {
	DIV_CLK_AOC_NOC_LH,
	DIV_CLK_AOC_TRACE_LH,
};
enum clk_id cmucal_vclk_blk_aur[] = {
	DIV_CLK_AUR_NOCP_LH,
	DIV_CLK_AUR_NOCP,
	DIV_CLK_AUR_AURCTL_LH,
};
enum clk_id cmucal_vclk_blk_bw[] = {
	DIV_CLK_BW_NOCP,
};
enum clk_id cmucal_vclk_blk_dpub[] = {
	DIV_CLK_DPUB_NOCP,
};
enum clk_id cmucal_vclk_blk_dpuf0[] = {
	DIV_CLK_DPUF0_NOCP,
};
enum clk_id cmucal_vclk_blk_dpuf1[] = {
	DIV_CLK_DPUF1_NOCP,
};
enum clk_id cmucal_vclk_blk_eh[] = {
	DIV_CLK_EH_NOCP_LH,
};
enum clk_id cmucal_vclk_blk_g2d[] = {
	DIV_CLK_G2D_NOCP,
};
enum clk_id cmucal_vclk_blk_g3d[] = {
	DIV_CLK_G3D_NOCP_LH,
	DIV_CLK_G3D_NOCP,
	DIV_CLK_G3D_TOP,
};
enum clk_id cmucal_vclk_blk_gdc[] = {
	DIV_CLK_GDC_NOCP,
};
enum clk_id cmucal_vclk_blk_gse[] = {
	DIV_CLK_GSE_NOCP,
};
enum clk_id cmucal_vclk_blk_hsi2[] = {
	DIV_CLK_HSI2_NOC_LH,
};
enum clk_id cmucal_vclk_blk_ispfe[] = {
	DIV_CLK_ISPFE_NOCP,
};
enum clk_id cmucal_vclk_blk_mcsc[] = {
	DIV_CLK_MCSC_NOCP,
};
enum clk_id cmucal_vclk_blk_mif[] = {
	DIV_CLK_MIF_NOCP_LH,
};
enum clk_id cmucal_vclk_blk_misc[] = {
	DIV_CLK_MISC_NOCP_LH,
	DIV_CLK_MISC_NOCP,
	DIV_CLK_MISC_GIC_LH,
	DIV_CLK_MISC_GIC,
};
enum clk_id cmucal_vclk_blk_nocl1a[] = {
	DIV_CLK_NOCL1A_NOCP_LH,
	DIV_CLK_NOCL1A_NOCP,
	DIV_CLK_NOCL1A_NOCD_LH,
};
enum clk_id cmucal_vclk_blk_nocl2aa[] = {
	DIV_CLK_NOCL2AA_NOCP_LH,
	DIV_CLK_NOCL2AA_NOCP,
	DIV_CLK_NOCL2AA_NOCD_LH,
};
enum clk_id cmucal_vclk_blk_nocl2ab[] = {
	DIV_CLK_NOCL2AB_NOCD_LH,
	DIV_CLK_NOCL2AB_NOCP_LH,
	DIV_CLK_NOCL2AB_NOCP,
};
enum clk_id cmucal_vclk_blk_peric0[] = {
	DIV_CLK_PERIC0_I3C,
	DIV_CLK_PERIC0_NOCP_LH,
};
enum clk_id cmucal_vclk_blk_peric1[] = {
	DIV_CLK_PERIC1_NOCP_LH,
};
enum clk_id cmucal_vclk_blk_rgbp[] = {
	DIV_CLK_RGBP_NOCP,
};
enum clk_id cmucal_vclk_blk_tnr[] = {
	DIV_CLK_TNR_NOCP,
};
enum clk_id cmucal_vclk_blk_tpu[] = {
	DIV_CLK_TPU_NOCP_LH,
	DIV_CLK_TPU_NOCP,
};
enum clk_id cmucal_vclk_blk_yuvp[] = {
	DIV_CLK_YUVP_NOCP,
};
/* GATE VCLK -> Clock Node List */
enum clk_id cmucal_vclk_ip_aoc_cmu_aoc[] = {
	CLK_BLK_AOC_UID_AOC_CMU_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_baaw_aoc[] = {
	GOUT_BLK_AOC_UID_BAAW_AOC_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_aoc[] = {
	GOUT_BLK_AOC_UID_D_TZPC_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_aoc[] = {
	GOUT_BLK_AOC_UID_GPC_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ld_hsi0_aoc[] = {
	GOUT_BLK_AOC_UID_LH_AXI_MI_LD_HSI0_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d_aoc[] = {
	GOUT_BLK_AOC_UID_LH_AXI_SI_D_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_aoc[] = {
	GOUT_BLK_AOC_UID_PPMU_AOC_IPCLKPORT_ACLK,
	GOUT_BLK_AOC_UID_PPMU_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_usb[] = {
	GOUT_BLK_AOC_UID_PPMU_USB_IPCLKPORT_PCLK,
	GOUT_BLK_AOC_UID_PPMU_USB_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_aoc[] = {
	GOUT_BLK_AOC_UID_SSMT_AOC_IPCLKPORT_PCLK,
	GOUT_BLK_AOC_UID_SSMT_AOC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu_aoc[] = {
	GOUT_BLK_AOC_UID_SYSMMU_S0_PMMU_AOC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_aoc[] = {
	GOUT_BLK_AOC_UID_SYSREG_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_aoc[] = {
	GOUT_BLK_AOC_UID_UASC_AOC_IPCLKPORT_ACLK,
	GOUT_BLK_AOC_UID_UASC_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_dp_aoc[] = {
	GOUT_BLK_AOC_UID_XIU_DP_AOC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_p_aoc[] = {
	GOUT_BLK_AOC_UID_XIU_P_AOC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_aoc_sysctrl_apb[] = {
	GOUT_BLK_AOC_UID_AOC_SYSCTRL_APB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_aoc_alive_cd[] = {
	CLK_BLK_AOC_UID_LH_AXI_SI_LP_AOC_ALIVE_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_aoc_alive_cd[] = {
	CLK_BLK_AOC_UID_LH_AXI_MI_LP_AOC_ALIVE_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_aoc_hsi0_cd[] = {
	CLK_BLK_AOC_UID_LH_AXI_SI_LP_AOC_HSI0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_aoc_hsi0_cd[] = {
	CLK_BLK_AOC_UID_LH_AXI_MI_LP_AOC_HSI0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_lp_aoc_alive[] = {
	CLK_BLK_AOC_UID_SLH_AXI_SI_LP_AOC_ALIVE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_lp_aoc_hsi0[] = {
	CLK_BLK_AOC_UID_SLH_AXI_SI_LP_AOC_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_aoc[] = {
	CLK_BLK_AOC_UID_LH_ATB_SI_LT_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_aoc_cd[] = {
	CLK_BLK_AOC_UID_LH_ATB_MI_LT_AOC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_aoc[] = {
	CLK_BLK_AOC_UID_SLH_AXI_MI_P_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_aoc_cu[] = {
	CLK_BLK_AOC_UID_LH_AXI_SI_P_AOC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_aoc_cu[] = {
	CLK_BLK_AOC_UID_LH_AXI_MI_P_AOC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_lg_alive_aoc[] = {
	CLK_BLK_AOC_UID_SLH_AXI_MI_LG_ALIVE_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_aoc_cd[] = {
	CLK_BLK_AOC_UID_LH_ATB_SI_LT_AOC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ld_hsi1_aoc[] = {
	CLK_BLK_AOC_UID_LH_AXI_MI_LD_HSI1_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_pcie[] = {
	CLK_BLK_AOC_UID_PPMU_PCIE_IPCLKPORT_ACLK,
	CLK_BLK_AOC_UID_PPMU_PCIE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_aoc[] = {
	CLK_BLK_AOC_UID_SYSMMU_S0_AOC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_aoc_hsi1_cd[] = {
	CLK_BLK_AOC_UID_LH_AXI_SI_LP_AOC_HSI1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_aoc_hsi1_cd[] = {
	CLK_BLK_AOC_UID_LH_AXI_MI_LP_AOC_HSI1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_lp_aoc_hsi1[] = {
	CLK_BLK_AOC_UID_SLH_AXI_SI_LP_AOC_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d_alive[] = {
	GOUT_BLK_APM_UID_LH_AXI_SI_D_ALIVE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_wdt_apm[] = {
	GOUT_BLK_APM_UID_WDT_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_apm[] = {
	GOUT_BLK_APM_UID_SYSREG_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_apm_ap[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM_AP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_apbif_pmu_alive[] = {
	GOUT_BLK_APM_UID_APBIF_PMU_ALIVE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_intmem[] = {
	GOUT_BLK_APM_UID_INTMEM_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_INTMEM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_pmu_intr_gen[] = {
	GOUT_BLK_APM_UID_PMU_INTR_GEN_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_dp_alive[] = {
	GOUT_BLK_APM_UID_XIU_DP_ALIVE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_apm_cmu_apm[] = {
	CLK_BLK_APM_UID_APM_CMU_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_grebeintegration[] = {
	GOUT_BLK_APM_UID_GREBEINTEGRATION_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_ip_apbif_gpio_alive[] = {
	GOUT_BLK_APM_UID_APBIF_GPIO_ALIVE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trtc[] = {
	GOUT_BLK_APM_UID_TRTC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_apm[] = {
	GOUT_BLK_APM_UID_D_TZPC_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_apm_aoc[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_dbgcore[] = {
	GOUT_BLK_APM_UID_MAILBOX_AP_DBGCORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_rtc[] = {
	GOUT_BLK_APM_UID_RTC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_apm_gsa[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM_GSA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d_alive[] = {
	GOUT_BLK_APM_UID_SSMT_D_ALIVE_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_SSMT_D_ALIVE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_lp_alive_cpucl0[] = {
	GOUT_BLK_APM_UID_SSMT_LP_ALIVE_CPUCL0_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_SSMT_LP_ALIVE_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_alive[] = {
	GOUT_BLK_APM_UID_SYSMMU_S0_PMMU0_ALIVE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_apm[] = {
	GOUT_BLK_APM_UID_GPC_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_apbif_gpio_far_alive[] = {
	GOUT_BLK_APM_UID_APBIF_GPIO_FAR_ALIVE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_rom_crc32_host[] = {
	GOUT_BLK_APM_UID_ROM_CRC32_HOST_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_ROM_CRC32_HOST_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ss_dbgcore[] = {
	GOUT_BLK_APM_UID_SS_DBGCORE_IPCLKPORT_SS_DBGCORE_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_apm_swd[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM_SWD_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_apm_tpu[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ig_swd[] = {
	GOUT_BLK_APM_UID_LH_AXI_MI_IG_SWD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_apm_usi0_uart[] = {
	GOUT_BLK_APM_UID_APM_USI0_UART_IPCLKPORT_PCLK,
	GOUT_BLK_APM_UID_APM_USI0_UART_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_apm_usi1_uart_int[] = {
	GOUT_BLK_APM_UID_APM_USI1_UART_INT_IPCLKPORT_PCLK,
	GOUT_BLK_APM_UID_APM_USI1_UART_INT_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_apm_usi0_usi[] = {
	GOUT_BLK_APM_UID_APM_USI0_USI_IPCLKPORT_PCLK,
	GOUT_BLK_APM_UID_APM_USI0_USI_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aoca32[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AOCA32_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aocf1[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AOCF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aocp6[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AOCP6_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aurmcutz[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AURMCUTZ_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aurcore0[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AURCORE0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aurcore1[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AURCORE1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aurcore2[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AURCORE2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_apm_i3c_pmic[] = {
	CLK_BLK_APM_UID_APM_I3C_PMIC_IPCLKPORT_I_PCLK,
	CLK_BLK_APM_UID_APM_I3C_PMIC_IPCLKPORT_I_SCLK,
};
enum clk_id cmucal_vclk_ip_apbif_intcomb_vgpio2pmu[] = {
	CLK_BLK_APM_UID_APBIF_INTCOMB_VGPIO2PMU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_apbif_intcomb_vgpio2ap[] = {
	CLK_BLK_APM_UID_APBIF_INTCOMB_VGPIO2AP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_apbif_intcomb_vgpio2apm[] = {
	CLK_BLK_APM_UID_APBIF_INTCOMB_VGPIO2APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_apm_aur[] = {
	CLK_BLK_APM_UID_MAILBOX_APM_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_lp_alive_cpucl0[] = {
	CLK_BLK_APM_UID_SLH_AXI_SI_LP_ALIVE_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_lg_scan2dram[] = {
	CLK_BLK_APM_UID_SLH_AXI_SI_LG_SCAN2DRAM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_alive[] = {
	CLK_BLK_APM_UID_SLH_AXI_MI_P_ALIVE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_lp_aoc_alive[] = {
	CLK_BLK_APM_UID_SLH_AXI_MI_LP_AOC_ALIVE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_alive_cpucl0_cd[] = {
	CLK_BLK_APM_UID_LH_AXI_SI_LP_ALIVE_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_alive_cpucl0_cd[] = {
	CLK_BLK_APM_UID_LH_AXI_MI_LP_ALIVE_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lg_scan2dram_cd[] = {
	CLK_BLK_APM_UID_LH_AXI_SI_LG_SCAN2DRAM_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lg_scan2dram_cd[] = {
	CLK_BLK_APM_UID_LH_AXI_MI_LG_SCAN2DRAM_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_aoc_alive_cu[] = {
	CLK_BLK_APM_UID_LH_AXI_SI_LP_AOC_ALIVE_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_aoc_alive_cu[] = {
	CLK_BLK_APM_UID_LH_AXI_MI_LP_AOC_ALIVE_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_alive_cu[] = {
	CLK_BLK_APM_UID_LH_AXI_SI_P_ALIVE_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_alive_cu[] = {
	CLK_BLK_APM_UID_LH_AXI_MI_P_ALIVE_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_apm_custom[] = {
	CLK_BLK_APM_UID_D_TZPC_APM_CUSTOM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_apm_custom[] = {
	CLK_BLK_APM_UID_GPC_APM_CUSTOM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_apm_dma[] = {
	CLK_BLK_APM_UID_APM_DMA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aurmcuns0[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AURMCUNS0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_aoc_aurcore0[] = {
	CLK_BLK_APM_UID_MAILBOX_AOC_AURCORE0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_aoc_aurcore1[] = {
	CLK_BLK_APM_UID_MAILBOX_AOC_AURCORE1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_aoc_aurcore2[] = {
	CLK_BLK_APM_UID_MAILBOX_AOC_AURCORE2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_alive[] = {
	CLK_BLK_APM_UID_SYSMMU_S0_ALIVE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_apm_custom[] = {
	CLK_BLK_APM_UID_SYSREG_APM_CUSTOM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_alive[] = {
	CLK_BLK_APM_UID_XIU_D_ALIVE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_pmu[] = {
	CLK_BLK_APM_UID_PMU_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_apbif_gpio_custom_alive[] = {
	CLK_BLK_APM_UID_APBIF_GPIO_CUSTOM_ALIVE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aurmcuns1[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AURMCUNS1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aurmcuns2[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AURMCUNS2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aurmcuns3[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AURMCUNS3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aurmcuns4[] = {
	CLK_BLK_APM_UID_MAILBOX_AP_AURMCUNS4_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_aoc_aurmcu[] = {
	CLK_BLK_APM_UID_MAILBOX_AOC_AURMCU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_apm_aurmcu[] = {
	CLK_BLK_APM_UID_MAILBOX_APM_AURMCU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_tpu_aurmcu[] = {
	CLK_BLK_APM_UID_MAILBOX_TPU_AURMCU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_aur_cmu_aur[] = {
	CLK_BLK_AUR_UID_AUR_CMU_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_aur[] = {
	CLK_BLK_AUR_UID_AUR_IPCLKPORT_AURORA_CORE_CLK,
	CLK_BLK_AUR_UID_AUR_IPCLKPORT_AURORA_PERI_CLK,
	CLK_BLK_AUR_UID_AUR_IPCLKPORT_AURORA_TRACE_CLK,
	CLK_BLK_AUR_UID_AUR_IPCLKPORT_AURORA_FABRIC_CLK,
};
enum clk_id cmucal_vclk_ip_as_apb_sysmmu_s1_ns_aur[] = {
	CLK_BLK_AUR_UID_AS_APB_SYSMMU_S1_NS_AUR_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_aur[] = {
	CLK_BLK_AUR_UID_D_TZPC_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_aur[] = {
	CLK_BLK_AUR_UID_GPC_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d0_aur[] = {
	CLK_BLK_AUR_UID_LH_ACEL_SI_D0_AUR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_aur[] = {
	CLK_BLK_AUR_UID_SSMT_D0_AUR_IPCLKPORT_ACLK,
	CLK_BLK_AUR_UID_SSMT_D0_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_aur[] = {
	CLK_BLK_AUR_UID_SSMT_D1_AUR_IPCLKPORT_ACLK,
	CLK_BLK_AUR_UID_SSMT_D1_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_aur[] = {
	CLK_BLK_AUR_UID_PPMU_D0_AUR_IPCLKPORT_ACLK,
	CLK_BLK_AUR_UID_PPMU_D0_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_aur[] = {
	CLK_BLK_AUR_UID_PPMU_D1_AUR_IPCLKPORT_ACLK,
	CLK_BLK_AUR_UID_PPMU_D1_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_aur[] = {
	CLK_BLK_AUR_UID_SYSMMU_S0_PMMU0_AUR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_aur[] = {
	CLK_BLK_AUR_UID_SYSMMU_S0_PMMU1_AUR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_aur[] = {
	CLK_BLK_AUR_UID_SYSREG_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_p0_aur[] = {
	CLK_BLK_AUR_UID_UASC_P0_AUR_IPCLKPORT_ACLK,
	CLK_BLK_AUR_UID_UASC_P0_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d1_aur[] = {
	CLK_BLK_AUR_UID_LH_ACEL_SI_D1_AUR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_adm_dap_g_aur[] = {
	CLK_BLK_AUR_UID_ADM_DAP_G_AUR_IPCLKPORT_DAPCLKM,
};
enum clk_id cmucal_vclk_ip_add_apbif_aur[] = {
	CLK_BLK_AUR_UID_ADD_APBIF_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_baaw_aur[] = {
	CLK_BLK_AUR_UID_BAAW_AUR_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_aur_cpucl0[] = {
	CLK_BLK_AUR_UID_LH_ATB_SI_LT_AUR_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_aur_cpucl0_cd[] = {
	CLK_BLK_AUR_UID_LH_ATB_SI_LT_AUR_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_aur_cpucl0_cd[] = {
	CLK_BLK_AUR_UID_LH_ATB_MI_LT_AUR_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_aur_cu[] = {
	CLK_BLK_AUR_UID_LH_AXI_SI_P_AUR_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_aur[] = {
	CLK_BLK_AUR_UID_SLH_AXI_MI_P_AUR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_aur_cu[] = {
	CLK_BLK_AUR_UID_LH_AXI_MI_P_AUR_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_p_aur[] = {
	CLK_BLK_AUR_UID_SSMT_P_AUR_IPCLKPORT_PCLK,
	CLK_BLK_AUR_UID_SSMT_P_AUR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_uasc_p1_aur[] = {
	CLK_BLK_AUR_UID_UASC_P1_AUR_IPCLKPORT_ACLK,
	CLK_BLK_AUR_UID_UASC_P1_AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_aur[] = {
	CLK_BLK_AUR_UID_SYSMMU_S0_AUR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_aur[] = {
	CLK_BLK_AUR_UID_XIU_D_AUR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_dapapbap_aur[] = {
	CLK_BLK_AUR_UID_DAPAPBAP_AUR_IPCLKPORT_DAPCLK,
};
enum clk_id cmucal_vclk_ip_blk_aur_frc_otp_deserial[] = {
	CLK_BLK_AUR_UID_BLK_AUR_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_add_aur[] = {
	CLK_BLK_AUR_UID_ADD_AUR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_bw_cmu_bw[] = {
	CLK_BLK_BW_UID_BW_CMU_BW_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d_bw[] = {
	GOUT_BLK_BW_UID_LH_AXI_SI_D_BW_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_bw[] = {
	GOUT_BLK_BW_UID_SLH_AXI_MI_P_BW_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_bw[] = {
	GOUT_BLK_BW_UID_PPMU_BW_IPCLKPORT_ACLK,
	GOUT_BLK_BW_UID_PPMU_BW_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_bw[] = {
	GOUT_BLK_BW_UID_SYSMMU_S0_PMMU0_BW_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_as_apb_sysmmu_s0_bw_s2[] = {
	GOUT_BLK_BW_UID_AS_APB_SYSMMU_S0_BW_S2_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_sysreg_bw[] = {
	GOUT_BLK_BW_UID_SYSREG_BW_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_bw[] = {
	GOUT_BLK_BW_UID_SSMT_BW_IPCLKPORT_PCLK,
	GOUT_BLK_BW_UID_SSMT_BW_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_bw[] = {
	GOUT_BLK_BW_UID_D_TZPC_BW_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_bw[] = {
	GOUT_BLK_BW_UID_GPC_BW_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_bw[] = {
	GOUT_BLK_BW_UID_UASC_BW_IPCLKPORT_ACLK,
	GOUT_BLK_BW_UID_UASC_BW_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_bw[] = {
	CLK_BLK_BW_UID_BW_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_bw[] = {
	CLK_BLK_BW_UID_LH_AXI_SI_IP_BW_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_bw[] = {
	CLK_BLK_BW_UID_LH_AXI_MI_IP_BW_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_bw[] = {
	CLK_BLK_BW_UID_XIU_D_BW_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_bw[] = {
	CLK_BLK_BW_UID_SYSMMU_S0_BW_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_trex_d_bw[] = {
	CLK_BLK_BW_UID_TREX_D_BW_IPCLKPORT_ACLK,
	CLK_BLK_BW_UID_TREX_D_BW_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_blk_bw_frc_otp_deserial[] = {
	CLK_BLK_BW_UID_BLK_BW_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_SYSREG_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_cssys[] = {
	GOUT_BLK_CPUCL0_UID_CSSYS_IPCLKPORT_PCLKDBG,
	GOUT_BLK_CPUCL0_UID_CSSYS_IPCLKPORT_ATCLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_it1_booker[] = {
	GOUT_BLK_CPUCL0_UID_LH_ATB_MI_IT1_BOOKER_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_cpucl0_cmu_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_CPUCL0_CMU_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_D_TZPC_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ig_cssys[] = {
	GOUT_BLK_CPUCL0_UID_LH_AXI_SI_IG_CSSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_apb_async_p_cssys_0[] = {
	GOUT_BLK_CPUCL0_UID_APB_ASYNC_P_CSSYS_0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_gpc_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_GPC_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_dp_cssys[] = {
	GOUT_BLK_CPUCL0_UID_XIU_DP_CSSYS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_SSMT_CPUCL0_IPCLKPORT_PCLK,
	GOUT_BLK_CPUCL0_UID_SSMT_CPUCL0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_s2mpu_s0_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_S2MPU_S0_CPUCL0_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ig_hsi0[] = {
	GOUT_BLK_CPUCL0_UID_LH_AXI_SI_IG_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ig_stm[] = {
	GOUT_BLK_CPUCL0_UID_LH_AXI_SI_IG_STM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ig_stm[] = {
	GOUT_BLK_CPUCL0_UID_LH_AXI_MI_IG_STM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_g_cssys_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_G_CSSYS_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_g_cssys_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_G_CSSYS_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_icc_cluster0_gic[] = {
	CLK_BLK_CPUCL0_UID_LH_AST_SI_L_ICC_CLUSTER0_GIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_icc_cluster0_gic_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AST_SI_L_ICC_CLUSTER0_GIC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_icc_cluster0_gic_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AST_MI_L_ICC_CLUSTER0_GIC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_lg_etr_hsi0[] = {
	CLK_BLK_CPUCL0_UID_SLH_AXI_SI_LG_ETR_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lg_etr_hsi0_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_LG_ETR_HSI0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lg_etr_hsi0_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_LG_ETR_HSI0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_iri_gic_cluster0[] = {
	CLK_BLK_CPUCL0_UID_LH_AST_MI_L_IRI_GIC_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_iri_gic_cluster0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_AST_SI_L_IRI_GIC_CLUSTER0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_iri_gic_cluster0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_AST_MI_L_IRI_GIC_CLUSTER0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_aoc[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_aoc_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_SI_LT_AOC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_aoc_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT_AOC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_aur_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT_AUR_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_aur_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_SI_LT_AUR_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_aur_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT_AUR_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_gsa_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT_GSA_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_gsa_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_SI_LT_GSA_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_gsa_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT_GSA_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_SLH_AXI_MI_P_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_P_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_P_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt0_tpu_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT0_TPU_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt1_tpu_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT1_TPU_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt0_tpu_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_SI_LT0_TPU_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt0_tpu_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT0_TPU_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt1_tpu_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_SI_LT1_TPU_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt1_tpu_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT1_TPU_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_t_bdu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_T_BDU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_t_slc[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_T_SLC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_t_bdu_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_SI_T_BDU_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_t_bdu_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_T_BDU_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_t_slc_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_SI_T_SLC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_t_slc_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_T_SLC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_cpucl0_con[] = {
	CLK_BLK_CPUCL0_UID_CPUCL0_CON_IPCLKPORT_I_PERIPHCLK,
};
enum clk_id cmucal_vclk_ip_apb_async_p_cssys_1[] = {
	CLK_BLK_CPUCL0_UID_APB_ASYNC_P_CSSYS_1_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_id_ppu[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_ID_PPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_booker[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_IP_BOOKER_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_id_ppu[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_ID_PPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_cluster0[] = {
	CLK_BLK_CPUCL0_UID_CLUSTER0_IPCLKPORT_GCLK0,
	CLK_BLK_CPUCL0_UID_CLUSTER0_IPCLKPORT_PPUCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_lp_alive_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_SLH_AXI_MI_LP_ALIVE_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_alive_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_LP_ALIVE_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_alive_cpucl0_cu[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_LP_ALIVE_CPUCL0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_cpucl0_d0[] = {
	CLK_BLK_CPUCL0_UID_PPMU_CPUCL0_D0_IPCLKPORT_PCLK,
	CLK_BLK_CPUCL0_UID_PPMU_CPUCL0_D0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_cpucl0_d1[] = {
	CLK_BLK_CPUCL0_UID_PPMU_CPUCL0_D1_IPCLKPORT_PCLK,
	CLK_BLK_CPUCL0_UID_PPMU_CPUCL0_D1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_cpucl0_d2[] = {
	CLK_BLK_CPUCL0_UID_PPMU_CPUCL0_D2_IPCLKPORT_PCLK,
	CLK_BLK_CPUCL0_UID_PPMU_CPUCL0_D2_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_cpucl0_d3[] = {
	CLK_BLK_CPUCL0_UID_PPMU_CPUCL0_D3_IPCLKPORT_PCLK,
	CLK_BLK_CPUCL0_UID_PPMU_CPUCL0_D3_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_g_cssys[] = {
	CLK_BLK_CPUCL0_UID_SLH_AXI_SI_G_CSSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi1[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_LP_CPUCL0_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi2[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_LP_CPUCL0_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi2_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_LP_CPUCL0_HSI2_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi1_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_LP_CPUCL0_HSI1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_add0_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_ADD0_CPUCL0_IPCLKPORT_CH_CLK,
};
enum clk_id cmucal_vclk_ip_add0_apbif_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_ADD0_APBIF_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_cpucl1[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_IP_CPUCL1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_cpucl2[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_IP_CPUCL2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ig_cssys[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_IG_CSSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_instrrun_cluster0_0[] = {
	CLK_BLK_CPUCL0_UID_PPC_INSTRRUN_CLUSTER0_0_IPCLKPORT_PCLK,
	CLK_BLK_CPUCL0_UID_PPC_INSTRRUN_CLUSTER0_0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppc_instrrun_cluster0_1[] = {
	CLK_BLK_CPUCL0_UID_PPC_INSTRRUN_CLUSTER0_1_IPCLKPORT_PCLK,
	CLK_BLK_CPUCL0_UID_PPC_INSTRRUN_CLUSTER0_1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppc_instrret_cluster0_0[] = {
	CLK_BLK_CPUCL0_UID_PPC_INSTRRET_CLUSTER0_0_IPCLKPORT_PCLK,
	CLK_BLK_CPUCL0_UID_PPC_INSTRRET_CLUSTER0_0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppc_instrret_cluster0_1[] = {
	CLK_BLK_CPUCL0_UID_PPC_INSTRRET_CLUSTER0_1_IPCLKPORT_PCLK,
	CLK_BLK_CPUCL0_UID_PPC_INSTRRET_CLUSTER0_1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_g3d_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_MI_LT_G3D_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_s2mpu_s0_pmmu0_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_S2MPU_S0_PMMU0_CPUCL0_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ig_hsi0[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_IG_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_cpucl0_qos[] = {
	CLK_BLK_CPUCL0_UID_CPUCL0_QOS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_axi_ds_128to32_p_pcie_cluster0[] = {
	CLK_BLK_CPUCL0_UID_AXI_DS_128TO32_P_PCIE_CLUSTER0_IPCLKPORT_MAINCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_cpucl0_nocl0[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_P_CPUCL0_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d0_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ACEL_SI_D0_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d1_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ACEL_SI_D1_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d2_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ACEL_SI_D2_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d3_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ACEL_SI_D3_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_booker[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_IP_BOOKER_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi2_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_LP_CPUCL0_HSI2_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi1_cd[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_LP_CPUCL0_HSI1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ig_booker[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_SI_IG_BOOKER_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ig_booker[] = {
	CLK_BLK_CPUCL0_UID_LH_AXI_MI_IG_BOOKER_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_apb_async_p_sysmmu[] = {
	CLK_BLK_CPUCL0_UID_APB_ASYNC_P_SYSMMU_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_gray2bin_cntvalueb[] = {
	CLK_BLK_CPUCL0_UID_GRAY2BIN_CNTVALUEB_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_gray2bin_tsvalueb[] = {
	CLK_BLK_CPUCL0_UID_GRAY2BIN_TSVALUEB_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_apb_async_p_booker_0[] = {
	CLK_BLK_CPUCL0_UID_APB_ASYNC_P_BOOKER_0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_apb_async_p_pcsm[] = {
	CLK_BLK_CPUCL0_UID_APB_ASYNC_P_PCSM_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_adm_apb_g_cluster0[] = {
	CLK_BLK_CPUCL0_UID_ADM_APB_G_CLUSTER0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_bps_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_BPS_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_p0_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_XIU_P0_CPUCL0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_p2_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_XIU_P2_CPUCL0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_it1_booker[] = {
	CLK_BLK_CPUCL0_UID_LH_ATB_SI_IT1_BOOKER_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d0_nocl0_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ACEL_MI_D0_NOCL0_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d1_nocl0_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ACEL_MI_D1_NOCL0_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_ld_eh_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_LH_ACEL_MI_LD_EH_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_cpucl1_cmu_cpucl1[] = {
	CLK_BLK_CPUCL1_UID_CPUCL1_CMU_CPUCL1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_cpucl1[] = {
	CLK_BLK_CPUCL1_UID_LH_AXI_MI_IP_CPUCL1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_add1_apbif_cpucl1[] = {
	CLK_BLK_CPUCL1_UID_ADD1_APBIF_CPUCL1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_cpucl2_cmu_cpucl2[] = {
	CLK_BLK_CPUCL2_UID_CPUCL2_CMU_CPUCL2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_add2_apbif_cpucl2[] = {
	CLK_BLK_CPUCL2_UID_ADD2_APBIF_CPUCL2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_cpucl2[] = {
	CLK_BLK_CPUCL2_UID_LH_AXI_MI_IP_CPUCL2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_dpub_cmu_dpub[] = {
	CLK_BLK_DPUB_UID_DPUB_CMU_DPUB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_decon_main[] = {
	GOUT_BLK_DPUB_UID_AD_APB_DECON_MAIN_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_dpub[] = {
	GOUT_BLK_DPUB_UID_DPUB_IPCLKPORT_ACLK_DECON,
	CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_ALVCLK_DSIM0,
	CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_ALVCLK_DSIM1,
	CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_OSCCLK_DSIM0,
	CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_OSCCLK_DSIM1,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_dpub[] = {
	CLK_BLK_DPUB_UID_SLH_AXI_MI_P_DPUB_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_dpub[] = {
	GOUT_BLK_DPUB_UID_D_TZPC_DPUB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_dpub[] = {
	GOUT_BLK_DPUB_UID_GPC_DPUB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_dpub[] = {
	GOUT_BLK_DPUB_UID_SYSREG_DPUB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_blk_dpub_frc_otp_deserial[] = {
	CLK_BLK_DPUB_UID_BLK_DPUB_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_dpuf0_cmu_dpuf0[] = {
	CLK_BLK_DPUF0_UID_DPUF0_CMU_DPUF0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_SYSREG_DPUF0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_SYSMMU_S0_DPUF0_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_SLH_AXI_MI_P_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d1_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_LH_AXI_SI_D1_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_SYSMMU_S0_PMMU0_DPUF0_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_PPMU_D0_DPUF0_IPCLKPORT_ACLK,
	GOUT_BLK_DPUF0_UID_PPMU_D0_DPUF0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_PPMU_D1_DPUF0_IPCLKPORT_ACLK,
	GOUT_BLK_DPUF0_UID_PPMU_D1_DPUF0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d0_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_LH_AXI_SI_D0_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_DPUF0_IPCLKPORT_ACLK_DPUF,
	CLK_BLK_DPUF0_UID_DPUF0_IPCLKPORT_ACLK_SRAMC,
};
enum clk_id cmucal_vclk_ip_d_tzpc_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_D_TZPC_DPUF0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_dpuf0_dma[] = {
	GOUT_BLK_DPUF0_UID_AD_APB_DPUF0_DMA_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_gpc_dpuf0[] = {
	GOUT_BLK_DPUF0_UID_GPC_DPUF0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_dpuf0[] = {
	CLK_BLK_DPUF0_UID_XIU_D0_DPUF0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_dpuf0[] = {
	CLK_BLK_DPUF0_UID_XIU_D1_DPUF0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ld0_dpuf1_dpuf0[] = {
	CLK_BLK_DPUF0_UID_LH_AXI_MI_LD0_DPUF1_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ld1_dpuf1_dpuf0[] = {
	CLK_BLK_DPUF0_UID_LH_AXI_MI_LD1_DPUF1_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_dpuf0[] = {
	CLK_BLK_DPUF0_UID_SYSMMU_S0_PMMU1_DPUF0_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_dpuf0[] = {
	CLK_BLK_DPUF0_UID_SSMT_D0_DPUF0_IPCLKPORT_ACLK,
	CLK_BLK_DPUF0_UID_SSMT_D0_DPUF0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_dpuf0[] = {
	CLK_BLK_DPUF0_UID_SSMT_D1_DPUF0_IPCLKPORT_ACLK,
	CLK_BLK_DPUF0_UID_SSMT_D1_DPUF0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_blk_dpuf0_frc_otp_deserial[] = {
	CLK_BLK_DPUF0_UID_BLK_DPUF0_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_dpuf1_dma[] = {
	GOUT_BLK_DPUF1_UID_AD_APB_DPUF1_DMA_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_sysreg_dpuf1[] = {
	GOUT_BLK_DPUF1_UID_SYSREG_DPUF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_dpuf1_cmu_dpuf1[] = {
	CLK_BLK_DPUF1_UID_DPUF1_CMU_DPUF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_dpuf1[] = {
	GOUT_BLK_DPUF1_UID_PPMU_D0_DPUF1_IPCLKPORT_ACLK,
	GOUT_BLK_DPUF1_UID_PPMU_D0_DPUF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_dpuf1[] = {
	GOUT_BLK_DPUF1_UID_D_TZPC_DPUF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_dpuf1[] = {
	GOUT_BLK_DPUF1_UID_GPC_DPUF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_dpuf1[] = {
	GOUT_BLK_DPUF1_UID_DPUF1_IPCLKPORT_ACLK_DPUF,
	CLK_BLK_DPUF1_UID_DPUF1_IPCLKPORT_ACLK_SRAMC,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_dpuf1[] = {
	GOUT_BLK_DPUF1_UID_SYSMMU_S0_PMMU0_DPUF1_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ld0_dpuf1_dpuf0[] = {
	GOUT_BLK_DPUF1_UID_LH_AXI_SI_LD0_DPUF1_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ld1_dpuf1_dpuf0[] = {
	GOUT_BLK_DPUF1_UID_LH_AXI_SI_LD1_DPUF1_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_dpuf1[] = {
	CLK_BLK_DPUF1_UID_SLH_AXI_MI_P_DPUF1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_dpuf1[] = {
	CLK_BLK_DPUF1_UID_PPMU_D1_DPUF1_IPCLKPORT_ACLK,
	CLK_BLK_DPUF1_UID_PPMU_D1_DPUF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_dpuf1[] = {
	CLK_BLK_DPUF1_UID_SYSMMU_S0_PMMU1_DPUF1_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_dpuf1[] = {
	CLK_BLK_DPUF1_UID_SYSMMU_S0_DPUF1_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_dpuf1[] = {
	CLK_BLK_DPUF1_UID_XIU_D_DPUF1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_dpuf1[] = {
	CLK_BLK_DPUF1_UID_SSMT_D0_DPUF1_IPCLKPORT_ACLK,
	CLK_BLK_DPUF1_UID_SSMT_D0_DPUF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_dpuf1[] = {
	CLK_BLK_DPUF1_UID_SSMT_D1_DPUF1_IPCLKPORT_ACLK,
	CLK_BLK_DPUF1_UID_SSMT_D1_DPUF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_blk_dpuf1_frc_otp_deserial[] = {
	CLK_BLK_DPUF1_UID_BLK_DPUF1_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_eh_cmu_eh[] = {
	CLK_BLK_EH_UID_EH_CMU_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_as_p_sysmmu_s1_ns_eh[] = {
	GOUT_BLK_EH_UID_AS_P_SYSMMU_S1_NS_EH_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_eh[] = {
	GOUT_BLK_EH_UID_D_TZPC_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_eh[] = {
	GOUT_BLK_EH_UID_GPC_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_eh_cu[] = {
	GOUT_BLK_EH_UID_LH_AXI_MI_P_EH_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_ld_eh_cpucl0[] = {
	GOUT_BLK_EH_UID_LH_ACEL_SI_LD_EH_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_eh[] = {
	GOUT_BLK_EH_UID_EH_IPCLKPORT_AXI_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_eh[] = {
	GOUT_BLK_EH_UID_SSMT_EH_IPCLKPORT_ACLK,
	GOUT_BLK_EH_UID_SSMT_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_eh[] = {
	GOUT_BLK_EH_UID_PPMU_EH_IPCLKPORT_ACLK,
	GOUT_BLK_EH_UID_PPMU_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_eh[] = {
	GOUT_BLK_EH_UID_SYSMMU_S0_EH_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_eh[] = {
	GOUT_BLK_EH_UID_SYSREG_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_eh[] = {
	GOUT_BLK_EH_UID_UASC_EH_IPCLKPORT_ACLK,
	GOUT_BLK_EH_UID_UASC_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_eh[] = {
	CLK_BLK_EH_UID_QE_EH_IPCLKPORT_ACLK,
	CLK_BLK_EH_UID_QE_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_eh[] = {
	CLK_BLK_EH_UID_SLH_AXI_MI_P_EH_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_eh_cu[] = {
	CLK_BLK_EH_UID_LH_AXI_SI_P_EH_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_eh[] = {
	CLK_BLK_EH_UID_LH_AXI_SI_IP_EH_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_eh[] = {
	CLK_BLK_EH_UID_LH_AXI_MI_IP_EH_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_eh[] = {
	CLK_BLK_EH_UID_SYSMMU_S0_PMMU0_EH_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_eh[] = {
	CLK_BLK_EH_UID_XIU_D_EH_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_blk_eh_frc_otp_deserial[] = {
	CLK_BLK_EH_UID_BLK_EH_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_eh_event[] = {
	CLK_BLK_EH_UID_PPC_EH_EVENT_IPCLKPORT_ACLK,
	CLK_BLK_EH_UID_PPC_EH_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_eh_cycle[] = {
	CLK_BLK_EH_UID_PPC_EH_CYCLE_IPCLKPORT_ACLK,
	CLK_BLK_EH_UID_PPC_EH_CYCLE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_g2d_cmu_g2d[] = {
	CLK_BLK_G2D_UID_G2D_CMU_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_g2d[] = {
	GOUT_BLK_G2D_UID_PPMU_D0_G2D_IPCLKPORT_ACLK,
	GOUT_BLK_G2D_UID_PPMU_D0_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_g2d[] = {
	GOUT_BLK_G2D_UID_PPMU_D1_G2D_IPCLKPORT_ACLK,
	GOUT_BLK_G2D_UID_PPMU_D1_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_g2d[] = {
	GOUT_BLK_G2D_UID_SYSMMU_S0_PMMU0_G2D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_g2d[] = {
	GOUT_BLK_G2D_UID_SYSREG_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d0_g2d[] = {
	GOUT_BLK_G2D_UID_LH_AXI_SI_D0_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d1_g2d[] = {
	GOUT_BLK_G2D_UID_LH_AXI_SI_D1_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu2_g2d[] = {
	GOUT_BLK_G2D_UID_SYSMMU_S0_PMMU2_G2D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_g2d[] = {
	GOUT_BLK_G2D_UID_PPMU_D2_G2D_IPCLKPORT_ACLK,
	GOUT_BLK_G2D_UID_PPMU_D2_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d2_g2d[] = {
	GOUT_BLK_G2D_UID_LH_ACEL_SI_D2_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_g2d[] = {
	GOUT_BLK_G2D_UID_SSMT_D0_G2D_IPCLKPORT_PCLK,
	GOUT_BLK_G2D_UID_SSMT_D0_G2D_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_g2d[] = {
	GOUT_BLK_G2D_UID_G2D_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_g2d[] = {
	GOUT_BLK_G2D_UID_SYSMMU_S0_PMMU1_G2D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_jpeg[] = {
	GOUT_BLK_G2D_UID_JPEG_IPCLKPORT_I_SMFC_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_g2d[] = {
	GOUT_BLK_G2D_UID_D_TZPC_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_g2d[] = {
	GOUT_BLK_G2D_UID_SSMT_D1_G2D_IPCLKPORT_ACLK,
	GOUT_BLK_G2D_UID_SSMT_D1_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d2_g2d[] = {
	GOUT_BLK_G2D_UID_SSMT_D2_G2D_IPCLKPORT_ACLK,
	GOUT_BLK_G2D_UID_SSMT_D2_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_g2d[] = {
	GOUT_BLK_G2D_UID_GPC_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_g2d[] = {
	GOUT_BLK_G2D_UID_SLH_AXI_MI_P_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_as_apb_g2d[] = {
	GOUT_BLK_G2D_UID_AS_APB_G2D_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_as_apb_jpeg[] = {
	GOUT_BLK_G2D_UID_AS_APB_JPEG_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_g2d[] = {
	CLK_BLK_G2D_UID_SYSMMU_S0_G2D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_g2d[] = {
	CLK_BLK_G2D_UID_XIU_D_G2D_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_id_g2d0_jpeg[] = {
	CLK_BLK_G2D_UID_LH_AST_SI_ID_G2D0_JPEG_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_id_g2d0_jpeg[] = {
	CLK_BLK_G2D_UID_LH_AST_MI_ID_G2D0_JPEG_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_id_g2d1_jpeg[] = {
	CLK_BLK_G2D_UID_LH_AST_SI_ID_G2D1_JPEG_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_id_g2d1_jpeg[] = {
	CLK_BLK_G2D_UID_LH_AST_MI_ID_G2D1_JPEG_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_id_jpeg_g2d0[] = {
	CLK_BLK_G2D_UID_LH_AST_SI_ID_JPEG_G2D0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_id_jpeg_g2d0[] = {
	CLK_BLK_G2D_UID_LH_AST_MI_ID_JPEG_G2D0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_id_jpeg_g2d1[] = {
	CLK_BLK_G2D_UID_LH_AST_SI_ID_JPEG_G2D1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_id_jpeg_g2d1[] = {
	CLK_BLK_G2D_UID_LH_AST_MI_ID_JPEG_G2D1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_blk_g2d_frc_otp_deserial[] = {
	CLK_BLK_G2D_UID_BLK_G2D_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_g3d_cu[] = {
	GOUT_BLK_G3D_UID_LH_AXI_MI_P_G3D_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_g3d[] = {
	GOUT_BLK_G3D_UID_SYSREG_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_g3d_cmu_g3d[] = {
	CLK_BLK_G3D_UID_G3D_CMU_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_g3d[] = {
	GOUT_BLK_G3D_UID_LH_AXI_SI_IP_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpu[] = {
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_STACKS,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_STACKS,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_COREGROUP,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_COREGROUP,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_TRACE,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_TRACE,
};
enum clk_id cmucal_vclk_ip_gray2bin_g3d[] = {
	GOUT_BLK_G3D_UID_GRAY2BIN_G3D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_g3d[] = {
	GOUT_BLK_G3D_UID_D_TZPC_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_g3d[] = {
	GOUT_BLK_G3D_UID_GPC_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_g3d[] = {
	GOUT_BLK_G3D_UID_UASC_G3D_IPCLKPORT_ACLK,
	GOUT_BLK_G3D_UID_UASC_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_add_apbif_g3d[] = {
	GOUT_BLK_G3D_UID_ADD_APBIF_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_add_g3d[] = {
	CLK_BLK_G3D_UID_ADD_G3D_IPCLKPORT_CH_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_g3d[] = {
	CLK_BLK_G3D_UID_SLH_AXI_MI_P_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_g3d_cu[] = {
	CLK_BLK_G3D_UID_LH_AXI_SI_P_G3D_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_g3d_cpucl0[] = {
	CLK_BLK_G3D_UID_LH_ATB_SI_LT_G3D_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppcfw_g3d0[] = {
	CLK_BLK_G3D_UID_PPCFW_G3D0_IPCLKPORT_PCLK,
	CLK_BLK_G3D_UID_PPCFW_G3D0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_blk_g3d_frc_otp_deserial[] = {
	CLK_BLK_G3D_UID_BLK_G3D_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_g3d0[] = {
	CLK_BLK_G3D_UID_SSMT_G3D0_IPCLKPORT_ACLK,
	CLK_BLK_G3D_UID_SSMT_G3D0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_adm_dap_g_gpu[] = {
	CLK_BLK_G3D_UID_ADM_DAP_G_GPU_IPCLKPORT_DAPCLKM,
};
enum clk_id cmucal_vclk_ip_dapahbap_gpu[] = {
	CLK_BLK_G3D_UID_DAPAHBAP_GPU_IPCLKPORT_DAPCLK,
	CLK_BLK_G3D_UID_DAPAHBAP_GPU_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_sysmmu_g3d[] = {
	CLK_BLK_G3D_UID_AD_APB_SYSMMU_G3D_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ppcfw_g3d1[] = {
	CLK_BLK_G3D_UID_PPCFW_G3D1_IPCLKPORT_ACLK,
	CLK_BLK_G3D_UID_PPCFW_G3D1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_g3d_d0[] = {
	CLK_BLK_G3D_UID_PPMU_G3D_D0_IPCLKPORT_ACLK,
	CLK_BLK_G3D_UID_PPMU_G3D_D0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d0_g3d[] = {
	CLK_BLK_G3D_UID_LH_ACEL_SI_D0_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d1_g3d[] = {
	CLK_BLK_G3D_UID_LH_ACEL_SI_D1_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d2_g3d[] = {
	CLK_BLK_G3D_UID_LH_ACEL_SI_D2_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d3_g3d[] = {
	CLK_BLK_G3D_UID_LH_ACEL_SI_D3_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_g3d[] = {
	CLK_BLK_G3D_UID_LH_AXI_MI_IP_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_g3d_d1[] = {
	CLK_BLK_G3D_UID_PPMU_G3D_D1_IPCLKPORT_ACLK,
	CLK_BLK_G3D_UID_PPMU_G3D_D1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_g3d_d2[] = {
	CLK_BLK_G3D_UID_PPMU_G3D_D2_IPCLKPORT_ACLK,
	CLK_BLK_G3D_UID_PPMU_G3D_D2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_g3d_d3[] = {
	CLK_BLK_G3D_UID_PPMU_G3D_D3_IPCLKPORT_ACLK,
	CLK_BLK_G3D_UID_PPMU_G3D_D3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_d_g3dmmu[] = {
	CLK_BLK_G3D_UID_SLH_AXI_SI_D_G3DMMU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_g3d1[] = {
	CLK_BLK_G3D_UID_SSMT_G3D1_IPCLKPORT_ACLK,
	CLK_BLK_G3D_UID_SSMT_G3D1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_g3d2[] = {
	CLK_BLK_G3D_UID_SSMT_G3D2_IPCLKPORT_ACLK,
	CLK_BLK_G3D_UID_SSMT_G3D2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_g3d3[] = {
	CLK_BLK_G3D_UID_SSMT_G3D3_IPCLKPORT_ACLK,
	CLK_BLK_G3D_UID_SSMT_G3D3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_g3d[] = {
	CLK_BLK_G3D_UID_SYSMMU_S0_G3D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_g3d[] = {
	CLK_BLK_G3D_UID_SYSMMU_S0_PMMU0_G3D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_g3d[] = {
	CLK_BLK_G3D_UID_SYSMMU_S0_PMMU1_G3D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu2_g3d[] = {
	CLK_BLK_G3D_UID_SYSMMU_S0_PMMU2_G3D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu3_g3d[] = {
	CLK_BLK_G3D_UID_SYSMMU_S0_PMMU3_G3D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_gdc_cmu_gdc[] = {
	CLK_BLK_GDC_UID_GDC_CMU_GDC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_gdc1[] = {
	GOUT_BLK_GDC_UID_AD_APB_GDC1_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_gdc[] = {
	GOUT_BLK_GDC_UID_D_TZPC_GDC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gdc1[] = {
	GOUT_BLK_GDC_UID_GDC1_IPCLKPORT_CLK,
	CLK_BLK_GDC_UID_GDC1_IPCLKPORT_C2CLK_M,
};
enum clk_id cmucal_vclk_ip_gpc_gdc[] = {
	GOUT_BLK_GDC_UID_GPC_GDC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_gdc0[] = {
	GOUT_BLK_GDC_UID_PPMU_D0_GDC0_IPCLKPORT_PCLK,
	CLK_BLK_GDC_UID_PPMU_D0_GDC0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_gdc1[] = {
	GOUT_BLK_GDC_UID_PPMU_D0_GDC1_IPCLKPORT_PCLK,
	GOUT_BLK_GDC_UID_PPMU_D0_GDC1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d_lme[] = {
	GOUT_BLK_GDC_UID_SSMT_D_LME_IPCLKPORT_PCLK,
	GOUT_BLK_GDC_UID_SSMT_D_LME_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_gdc1[] = {
	GOUT_BLK_GDC_UID_SSMT_D0_GDC1_IPCLKPORT_PCLK,
	GOUT_BLK_GDC_UID_SSMT_D0_GDC1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_gdc0[] = {
	GOUT_BLK_GDC_UID_SSMT_D0_GDC0_IPCLKPORT_ACLK,
	GOUT_BLK_GDC_UID_SSMT_D0_GDC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_gdc[] = {
	GOUT_BLK_GDC_UID_SYSMMU_S0_PMMU0_GDC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_gdc[] = {
	GOUT_BLK_GDC_UID_SYSREG_GDC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_id_gdc0_gdc1[] = {
	GOUT_BLK_GDC_UID_LH_AST_MI_ID_GDC0_GDC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_id_gdc1_gdc0[] = {
	GOUT_BLK_GDC_UID_LH_AST_MI_ID_GDC1_GDC0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_id_gdc1_lme[] = {
	GOUT_BLK_GDC_UID_LH_AST_MI_ID_GDC1_LME_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_id_lme_gdc1[] = {
	GOUT_BLK_GDC_UID_LH_AST_SI_ID_LME_GDC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_id_gdc1_lme[] = {
	GOUT_BLK_GDC_UID_LH_AST_SI_ID_GDC1_LME_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu2_gdc[] = {
	GOUT_BLK_GDC_UID_SYSMMU_S0_PMMU2_GDC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_gdc[] = {
	GOUT_BLK_GDC_UID_SYSMMU_S0_GDC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_qe_d2_gdc0[] = {
	GOUT_BLK_GDC_UID_QE_D2_GDC0_IPCLKPORT_ACLK,
	GOUT_BLK_GDC_UID_QE_D2_GDC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d0_gdc0[] = {
	GOUT_BLK_GDC_UID_QE_D0_GDC0_IPCLKPORT_ACLK,
	GOUT_BLK_GDC_UID_QE_D0_GDC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_gdc[] = {
	GOUT_BLK_GDC_UID_SLH_AXI_MI_P_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d1_gdc[] = {
	GOUT_BLK_GDC_UID_LH_AXI_SI_D1_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d_lme[] = {
	GOUT_BLK_GDC_UID_PPMU_D_LME_IPCLKPORT_PCLK,
	CLK_BLK_GDC_UID_PPMU_D_LME_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_gdc[] = {
	CLK_BLK_GDC_UID_XIU_D1_GDC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d4_gdc1[] = {
	CLK_BLK_GDC_UID_PPMU_D4_GDC1_IPCLKPORT_PCLK,
	CLK_BLK_GDC_UID_PPMU_D4_GDC1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_gdc1[] = {
	CLK_BLK_GDC_UID_PPMU_D2_GDC1_IPCLKPORT_ACLK,
	CLK_BLK_GDC_UID_PPMU_D2_GDC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_gdc0[] = {
	CLK_BLK_GDC_UID_PPMU_D2_GDC0_IPCLKPORT_ACLK,
	CLK_BLK_GDC_UID_PPMU_D2_GDC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d4_gdc0[] = {
	CLK_BLK_GDC_UID_PPMU_D4_GDC0_IPCLKPORT_ACLK,
	CLK_BLK_GDC_UID_PPMU_D4_GDC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d4_gdc1[] = {
	CLK_BLK_GDC_UID_SSMT_D4_GDC1_IPCLKPORT_PCLK,
	CLK_BLK_GDC_UID_SSMT_D4_GDC1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d2_gdc1[] = {
	CLK_BLK_GDC_UID_SSMT_D2_GDC1_IPCLKPORT_ACLK,
	CLK_BLK_GDC_UID_SSMT_D2_GDC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d2_gdc0[] = {
	CLK_BLK_GDC_UID_SSMT_D2_GDC0_IPCLKPORT_ACLK,
	CLK_BLK_GDC_UID_SSMT_D2_GDC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d4_gdc0[] = {
	CLK_BLK_GDC_UID_SSMT_D4_GDC0_IPCLKPORT_ACLK,
	CLK_BLK_GDC_UID_SSMT_D4_GDC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d0_gdc1[] = {
	CLK_BLK_GDC_UID_QE_D0_GDC1_IPCLKPORT_PCLK,
	CLK_BLK_GDC_UID_QE_D0_GDC1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d2_gdc1[] = {
	CLK_BLK_GDC_UID_QE_D2_GDC1_IPCLKPORT_ACLK,
	CLK_BLK_GDC_UID_QE_D2_GDC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d4_gdc0[] = {
	CLK_BLK_GDC_UID_QE_D4_GDC0_IPCLKPORT_ACLK,
	CLK_BLK_GDC_UID_QE_D4_GDC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d4_gdc1[] = {
	CLK_BLK_GDC_UID_QE_D4_GDC1_IPCLKPORT_ACLK,
	CLK_BLK_GDC_UID_QE_D4_GDC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_id_lme_gdc1[] = {
	CLK_BLK_GDC_UID_LH_AST_MI_ID_LME_GDC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_id_gdc0_gdc1[] = {
	CLK_BLK_GDC_UID_LH_AST_SI_ID_GDC0_GDC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_gdc0[] = {
	CLK_BLK_GDC_UID_AD_APB_GDC0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ad_apb_lme[] = {
	CLK_BLK_GDC_UID_AD_APB_LME_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_gdc[] = {
	CLK_BLK_GDC_UID_SYSMMU_S0_PMMU1_GDC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d0_gdc[] = {
	CLK_BLK_GDC_UID_LH_AXI_SI_D0_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d2_gdc[] = {
	CLK_BLK_GDC_UID_LH_AXI_SI_D2_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gdc0[] = {
	CLK_BLK_GDC_UID_GDC0_IPCLKPORT_C2CLK_M,
	CLK_BLK_GDC_UID_GDC0_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_gdc[] = {
	CLK_BLK_GDC_UID_XIU_D0_GDC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d2_gdc[] = {
	CLK_BLK_GDC_UID_XIU_D2_GDC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_id_gdc1_gdc0[] = {
	CLK_BLK_GDC_UID_LH_AST_SI_ID_GDC1_GDC0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ld_rgbp_gdc[] = {
	CLK_BLK_GDC_UID_LH_AXI_MI_LD_RGBP_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_blk_gdc_frc_otp_deserial[] = {
	CLK_BLK_GDC_UID_BLK_GDC_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lme[] = {
	CLK_BLK_GDC_UID_LME_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_gsacore_cmu_gsacore[] = {
	CLK_BLK_GSACORE_UID_GSACORE_CMU_GSACORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ca32_gsacore[] = {
	GOUT_BLK_GSACORE_UID_CA32_GSACORE_IPCLKPORT_CLKIN,
};
enum clk_id cmucal_vclk_ip_gpio_gsacore0[] = {
	GOUT_BLK_GSACORE_UID_GPIO_GSACORE0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_kdn_gsacore[] = {
	GOUT_BLK_GSACORE_UID_KDN_GSACORE_IPCLKPORT_PCLK,
	GOUT_BLK_GSACORE_UID_KDN_GSACORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_otp_con_gsacore[] = {
	GOUT_BLK_GSACORE_UID_OTP_CON_GSACORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_gsacore0[] = {
	GOUT_BLK_GSACORE_UID_PPMU_GSACORE0_IPCLKPORT_PCLK,
	GOUT_BLK_GSACORE_UID_PPMU_GSACORE0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_ca32_gsacore[] = {
	GOUT_BLK_GSACORE_UID_QE_CA32_GSACORE_IPCLKPORT_PCLK,
	GOUT_BLK_GSACORE_UID_QE_CA32_GSACORE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_dma_gsacore[] = {
	GOUT_BLK_GSACORE_UID_QE_DMA_GSACORE_IPCLKPORT_PCLK,
	GOUT_BLK_GSACORE_UID_QE_DMA_GSACORE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_sc_gsacore[] = {
	GOUT_BLK_GSACORE_UID_QE_SC_GSACORE_IPCLKPORT_PCLK,
	GOUT_BLK_GSACORE_UID_QE_SC_GSACORE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_resetmon_gsacore[] = {
	GOUT_BLK_GSACORE_UID_RESETMON_GSACORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_spi_fps_gsacore[] = {
	GOUT_BLK_GSACORE_UID_SPI_FPS_GSACORE_IPCLKPORT_PCLK,
	GOUT_BLK_GSACORE_UID_SPI_FPS_GSACORE_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_spi_gsc_gsacore[] = {
	GOUT_BLK_GSACORE_UID_SPI_GSC_GSACORE_IPCLKPORT_PCLK,
	GOUT_BLK_GSACORE_UID_SPI_GSC_GSACORE_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_sc_gsacore[] = {
	GOUT_BLK_GSACORE_UID_SC_GSACORE_IPCLKPORT_I_PCLK,
	CLK_BLK_GSACORE_UID_SC_GSACORE_IPCLKPORT_I_ACLK,
};
enum clk_id cmucal_vclk_ip_sysreg_gsacore[] = {
	GOUT_BLK_GSACORE_UID_SYSREG_GSACORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uart_gsacore[] = {
	GOUT_BLK_GSACORE_UID_UART_GSACORE_IPCLKPORT_PCLK,
	GOUT_BLK_GSACORE_UID_UART_GSACORE_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_wdt_gsacore[] = {
	GOUT_BLK_GSACORE_UID_WDT_GSACORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_baaw_gsacore[] = {
	GOUT_BLK_GSACORE_UID_BAAW_GSACORE_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_intmem_gsacore[] = {
	GOUT_BLK_GSACORE_UID_INTMEM_GSACORE_IPCLKPORT_I_ACLK,
	GOUT_BLK_GSACORE_UID_INTMEM_GSACORE_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_gsa[] = {
	GOUT_BLK_GSACORE_UID_LH_AXI_SI_IP_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_dma_gsacore[] = {
	GOUT_BLK_GSACORE_UID_DMA_GSACORE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_dma_gsacore_ns[] = {
	GOUT_BLK_GSACORE_UID_AD_APB_DMA_GSACORE_NS_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_puf_gsacore[] = {
	GOUT_BLK_GSACORE_UID_PUF_GSACORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_dp0_gsa_zm[] = {
	GOUT_BLK_GSACORE_UID_XIU_DP0_GSA_ZM_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_i_dap_gsa[] = {
	GOUT_BLK_GSACORE_UID_LH_AXI_MI_I_DAP_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_i_ca32_gic[] = {
	GOUT_BLK_GSACORE_UID_LH_AST_MI_I_CA32_GIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_i_gic_ca32[] = {
	GOUT_BLK_GSACORE_UID_LH_AST_MI_I_GIC_CA32_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gic_gsacore[] = {
	CLK_BLK_GSACORE_UID_GIC_GSACORE_IPCLKPORT_GICCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_i_gic_ca32[] = {
	GOUT_BLK_GSACORE_UID_LH_AST_SI_I_GIC_CA32_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_i_ca32_gic[] = {
	GOUT_BLK_GSACORE_UID_LH_AST_SI_I_CA32_GIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_gsa_cpucl0_cd[] = {
	CLK_BLK_GSACORE_UID_LH_ATB_SI_LT_GSA_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt_gsa_cpucl0[] = {
	CLK_BLK_GSACORE_UID_LH_ATB_SI_LT_GSA_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt_gsa_cpucl0_cd[] = {
	CLK_BLK_GSACORE_UID_LH_ATB_MI_LT_GSA_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_axi2apb1_gsacore[] = {
	CLK_BLK_GSACORE_UID_LH_AXI_SI_IP_AXI2APB1_GSACORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_axi2apb1_gsacore[] = {
	CLK_BLK_GSACORE_UID_LH_AXI_MI_IP_AXI2APB1_GSACORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_axi2apb2_gsacore[] = {
	CLK_BLK_GSACORE_UID_LH_AXI_SI_IP_AXI2APB2_GSACORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_axi2apb2_gsacore[] = {
	CLK_BLK_GSACORE_UID_LH_AXI_MI_IP_AXI2APB2_GSACORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_intmem_gsacore[] = {
	CLK_BLK_GSACORE_UID_AD_APB_INTMEM_GSACORE_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_gpio_gsacore1[] = {
	CLK_BLK_GSACORE_UID_GPIO_GSACORE1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpio_gsacore2[] = {
	CLK_BLK_GSACORE_UID_GPIO_GSACORE2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpio_gsacore3[] = {
	CLK_BLK_GSACORE_UID_GPIO_GSACORE3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_id_gme_gsa[] = {
	CLK_BLK_GSACORE_UID_LH_AXI_SI_ID_GME_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ugme[] = {
	CLK_BLK_GSACORE_UID_UGME_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_id_sc_gsacore[] = {
	CLK_BLK_GSACORE_UID_LH_AXI_SI_ID_SC_GSACORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_id_sc_gsacore[] = {
	CLK_BLK_GSACORE_UID_LH_AXI_MI_ID_SC_GSACORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_gsacore1[] = {
	CLK_BLK_GSACORE_UID_PPMU_GSACORE1_IPCLKPORT_ACLK,
	CLK_BLK_GSACORE_UID_PPMU_GSACORE1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_gsa_zm[] = {
	CLK_BLK_GSACORE_UID_XIU_D0_GSA_ZM_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_gsactrl_cmu_gsactrl[] = {
	CLK_BLK_GSACTRL_UID_GSACTRL_CMU_GSACTRL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_gsactrl[] = {
	GOUT_BLK_GSACTRL_UID_GPC_GSACTRL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_gsa2aoc[] = {
	GOUT_BLK_GSACTRL_UID_MAILBOX_GSA2AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_gsa2nontz[] = {
	GOUT_BLK_GSACTRL_UID_MAILBOX_GSA2NONTZ_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_gsa2tpu[] = {
	GOUT_BLK_GSACTRL_UID_MAILBOX_GSA2TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_gsa2aur[] = {
	GOUT_BLK_GSACTRL_UID_MAILBOX_GSA2AUR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_gsactrl[] = {
	GOUT_BLK_GSACTRL_UID_SYSREG_GSACTRL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_tzpc_gsactrl[] = {
	GOUT_BLK_GSACTRL_UID_TZPC_GSACTRL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_intmem_gsactrl[] = {
	GOUT_BLK_GSACTRL_UID_INTMEM_GSACTRL_IPCLKPORT_I_ACLK,
	GOUT_BLK_GSACTRL_UID_INTMEM_GSACTRL_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_gsa[] = {
	GOUT_BLK_GSACTRL_UID_LH_AXI_MI_IP_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mailbox_gsa2tz[] = {
	GOUT_BLK_GSACTRL_UID_MAILBOX_GSA2TZ_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_pmu_gsa[] = {
	GOUT_BLK_GSACTRL_UID_PMU_GSA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_apbif_gpio_gsactrl[] = {
	GOUT_BLK_GSACTRL_UID_APBIF_GPIO_GSACTRL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_timer_gsactrl[] = {
	GOUT_BLK_GSACTRL_UID_TIMER_GSACTRL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_dap_gsactrl[] = {
	GOUT_BLK_GSACTRL_UID_DAP_GSACTRL_IPCLKPORT_DAPCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_gsa_cu[] = {
	GOUT_BLK_GSACTRL_UID_LH_AXI_MI_P_GSA_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_gsactrlext[] = {
	GOUT_BLK_GSACTRL_UID_SYSREG_GSACTRLEXT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_secjtag_gsactrl[] = {
	GOUT_BLK_GSACTRL_UID_SECJTAG_GSACTRL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_i_dap_gsa[] = {
	GOUT_BLK_GSACTRL_UID_LH_AXI_SI_I_DAP_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_intmem_gsactrl[] = {
	GOUT_BLK_GSACTRL_UID_AD_APB_INTMEM_GSACTRL_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_gsa[] = {
	CLK_BLK_GSACTRL_UID_SLH_AXI_MI_P_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_gsa_cu[] = {
	CLK_BLK_GSACTRL_UID_LH_AXI_SI_P_GSA_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_axi2apb0_gsactrl[] = {
	CLK_BLK_GSACTRL_UID_LH_AXI_SI_IP_AXI2APB0_GSACTRL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_axi2apb0_gsactrl[] = {
	CLK_BLK_GSACTRL_UID_LH_AXI_MI_IP_AXI2APB0_GSACTRL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_dp1_gsa_zm[] = {
	CLK_BLK_GSACTRL_UID_XIU_DP1_GSA_ZM_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_gsa_zm[] = {
	CLK_BLK_GSACTRL_UID_XIU_D1_GSA_ZM_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_id_gme_gsa[] = {
	CLK_BLK_GSACTRL_UID_LH_AXI_MI_ID_GME_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d_gsa[] = {
	CLK_BLK_GSACTRL_UID_LH_AXI_SI_D_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_sysmmu_gsa_s1_ns[] = {
	CLK_BLK_GSACTRL_UID_AD_APB_SYSMMU_GSA_S1_NS_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ssmt_gsactrl[] = {
	CLK_BLK_GSACTRL_UID_SSMT_GSACTRL_IPCLKPORT_ACLK,
	CLK_BLK_GSACTRL_UID_SSMT_GSACTRL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_gsa_zm[] = {
	CLK_BLK_GSACTRL_UID_SYSMMU_S0_GSA_ZM_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_gsa_zm[] = {
	CLK_BLK_GSACTRL_UID_SYSMMU_S0_PMMU0_GSA_ZM_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_gse_cmu_gse[] = {
	CLK_BLK_GSE_UID_GSE_CMU_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_gse[] = {
	GOUT_BLK_GSE_UID_D_TZPC_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_gse[] = {
	GOUT_BLK_GSE_UID_SLH_AXI_MI_P_GSE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_gse[] = {
	GOUT_BLK_GSE_UID_SYSREG_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_gse[] = {
	GOUT_BLK_GSE_UID_AD_APB_GSE_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d_gse[] = {
	GOUT_BLK_GSE_UID_LH_AXI_SI_D_GSE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_gse[] = {
	GOUT_BLK_GSE_UID_PPMU_D0_GSE_IPCLKPORT_ACLK,
	GOUT_BLK_GSE_UID_PPMU_D0_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gse[] = {
	GOUT_BLK_GSE_UID_GSE_IPCLKPORT_CLK_GSE,
	CLK_BLK_GSE_UID_GSE_IPCLKPORT_CLK_VOTF,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_gse[] = {
	GOUT_BLK_GSE_UID_SYSMMU_S0_PMMU0_GSE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_gse[] = {
	GOUT_BLK_GSE_UID_GPC_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_gse[] = {
	GOUT_BLK_GSE_UID_SYSMMU_S0_GSE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_gse[] = {
	GOUT_BLK_GSE_UID_PPMU_D1_GSE_IPCLKPORT_ACLK,
	GOUT_BLK_GSE_UID_PPMU_D1_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d0_gse[] = {
	GOUT_BLK_GSE_UID_QE_D0_GSE_IPCLKPORT_ACLK,
	GOUT_BLK_GSE_UID_QE_D0_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d1_gse[] = {
	GOUT_BLK_GSE_UID_QE_D1_GSE_IPCLKPORT_ACLK,
	GOUT_BLK_GSE_UID_QE_D1_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_gse[] = {
	GOUT_BLK_GSE_UID_SSMT_D0_GSE_IPCLKPORT_ACLK,
	GOUT_BLK_GSE_UID_SSMT_D0_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_gse[] = {
	GOUT_BLK_GSE_UID_SSMT_D1_GSE_IPCLKPORT_ACLK,
	GOUT_BLK_GSE_UID_SSMT_D1_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_gse[] = {
	GOUT_BLK_GSE_UID_XIU_D1_GSE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_otf_yuvp_gse[] = {
	GOUT_BLK_GSE_UID_LH_AST_MI_L_OTF_YUVP_GSE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_otf_tnr_gse[] = {
	GOUT_BLK_GSE_UID_LH_AST_MI_L_OTF_TNR_GSE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_gse[] = {
	GOUT_BLK_GSE_UID_XIU_D0_GSE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d2_gse[] = {
	GOUT_BLK_GSE_UID_SSMT_D2_GSE_IPCLKPORT_ACLK,
	GOUT_BLK_GSE_UID_SSMT_D2_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d2_gse[] = {
	GOUT_BLK_GSE_UID_QE_D2_GSE_IPCLKPORT_ACLK,
	GOUT_BLK_GSE_UID_QE_D2_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_gse[] = {
	GOUT_BLK_GSE_UID_PPMU_D2_GSE_IPCLKPORT_ACLK,
	GOUT_BLK_GSE_UID_PPMU_D2_GSE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_blk_gse_frc_otp_deserial[] = {
	CLK_BLK_GSE_UID_BLK_GSE_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_axi_us_128to256_qe_d2_gse[] = {
	CLK_BLK_GSE_UID_AXI_US_128TO256_QE_D2_GSE_IPCLKPORT_MAINCLK,
};
enum clk_id cmucal_vclk_ip_hsi0_cmu_hsi0[] = {
	CLK_BLK_HSI0_UID_HSI0_CMU_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usb32drd[] = {
	GOUT_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_USB32DRD_REF_CLK_40,
	GOUT_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_USBSUBCTL_APB_PCLK,
	GOUT_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_USBDPPHY_CTRL_PCLK,
	GOUT_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_EUSB_CTRL_PCLK,
	GOUT_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_USBLINK_ACLK,
	GOUT_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_USBDPPHY_TCA_APB_CLK,
	CLK_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_EUSB_APB_CLK,
	CLK_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_EUSB_PHY_REFCLK_26,
	CLK_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_RTC_CLK_HSI0__ALV,
	CLK_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_USB32DRD_U3REWA_ALV_CLK,
	CLK_BLK_HSI0_UID_USB32DRD_IPCLKPORT_I_USB32DRD_SUSPEND_CLK_26,
};
enum clk_id cmucal_vclk_ip_dp_link[] = {
	GOUT_BLK_HSI0_UID_DP_LINK_IPCLKPORT_I_DP_GTC_CLK,
	GOUT_BLK_HSI0_UID_DP_LINK_IPCLKPORT_I_PCLK,
	CLK_BLK_HSI0_UID_DP_LINK_IPCLKPORT_I_DP_OSC_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_hsi0[] = {
	GOUT_BLK_HSI0_UID_XIU_D0_HSI0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_etr_miu[] = {
	GOUT_BLK_HSI0_UID_ETR_MIU_IPCLKPORT_I_ACLK,
	GOUT_BLK_HSI0_UID_ETR_MIU_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_hsi0[] = {
	GOUT_BLK_HSI0_UID_PPMU_HSI0_IPCLKPORT_ACLK,
	GOUT_BLK_HSI0_UID_PPMU_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ld_hsi0_aoc[] = {
	GOUT_BLK_HSI0_UID_LH_AXI_SI_LD_HSI0_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d_hsi0[] = {
	GOUT_BLK_HSI0_UID_LH_ACEL_SI_D_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_hsi0[] = {
	GOUT_BLK_HSI0_UID_GPC_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_hsi0[] = {
	GOUT_BLK_HSI0_UID_D_TZPC_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_hsi0[] = {
	GOUT_BLK_HSI0_UID_SSMT_HSI0_IPCLKPORT_PCLK,
	GOUT_BLK_HSI0_UID_SSMT_HSI0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysreg_hsi0[] = {
	GOUT_BLK_HSI0_UID_SYSREG_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_p_hsi0[] = {
	GOUT_BLK_HSI0_UID_XIU_P_HSI0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d2_hsi0[] = {
	GOUT_BLK_HSI0_UID_XIU_D2_HSI0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_uasc_hsi0_link[] = {
	GOUT_BLK_HSI0_UID_UASC_HSI0_LINK_IPCLKPORT_ACLK,
	GOUT_BLK_HSI0_UID_UASC_HSI0_LINK_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_hsi0[] = {
	CLK_BLK_HSI0_UID_SYSMMU_S0_HSI0_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_lg_etr_hsi0[] = {
	CLK_BLK_HSI0_UID_SLH_AXI_MI_LG_ETR_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_lp_aoc_hsi0[] = {
	CLK_BLK_HSI0_UID_SLH_AXI_MI_LP_AOC_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_hsi0[] = {
	CLK_BLK_HSI0_UID_SLH_AXI_MI_P_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lg_etr_hsi0_cu[] = {
	CLK_BLK_HSI0_UID_LH_AXI_SI_LG_ETR_HSI0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lg_etr_hsi0_cu[] = {
	CLK_BLK_HSI0_UID_LH_AXI_MI_LG_ETR_HSI0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_aoc_hsi0_cu[] = {
	CLK_BLK_HSI0_UID_LH_AXI_SI_LP_AOC_HSI0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_aoc_hsi0_cu[] = {
	CLK_BLK_HSI0_UID_LH_AXI_MI_LP_AOC_HSI0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_hsi0_cu[] = {
	CLK_BLK_HSI0_UID_LH_AXI_SI_P_HSI0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_hsi0_cu[] = {
	CLK_BLK_HSI0_UID_LH_AXI_MI_P_HSI0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_hsi0[] = {
	CLK_BLK_HSI0_UID_SYSMMU_S0_PMMU0_HSI0_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_usi0_hsi0[] = {
	CLK_BLK_HSI0_UID_USI0_HSI0_IPCLKPORT_IPCLK,
	CLK_BLK_HSI0_UID_USI0_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi1_hsi0[] = {
	CLK_BLK_HSI0_UID_USI1_HSI0_IPCLKPORT_IPCLK,
	CLK_BLK_HSI0_UID_USI1_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi2_hsi0[] = {
	CLK_BLK_HSI0_UID_USI2_HSI0_IPCLKPORT_IPCLK,
	CLK_BLK_HSI0_UID_USI2_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi3_hsi0[] = {
	CLK_BLK_HSI0_UID_USI3_HSI0_IPCLKPORT_IPCLK,
	CLK_BLK_HSI0_UID_USI3_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_eusbphy_hsi0[] = {
	CLK_BLK_HSI0_UID_AD_APB_EUSBPHY_HSI0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_blk_hsi0_frc_otp_deserial[] = {
	CLK_BLK_HSI0_UID_BLK_HSI0_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_usi4_hsi0[] = {
	CLK_BLK_HSI0_UID_USI4_HSI0_IPCLKPORT_PCLK,
	CLK_BLK_HSI0_UID_USI4_HSI0_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_i3c2_hsi0[] = {
	CLK_BLK_HSI0_UID_I3C2_HSI0_IPCLKPORT_I_PCLK,
	CLK_BLK_HSI0_UID_I3C2_HSI0_IPCLKPORT_I_SCLK,
};
enum clk_id cmucal_vclk_ip_i3c3_hsi0[] = {
	CLK_BLK_HSI0_UID_I3C3_HSI0_IPCLKPORT_I_PCLK,
	CLK_BLK_HSI0_UID_I3C3_HSI0_IPCLKPORT_I_SCLK,
};
enum clk_id cmucal_vclk_ip_hsi1_cmu_hsi1[] = {
	CLK_BLK_HSI1_UID_HSI1_CMU_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d_hsi1[] = {
	GOUT_BLK_HSI1_UID_LH_ACEL_SI_D_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_hsi1_cu[] = {
	GOUT_BLK_HSI1_UID_LH_AXI_MI_P_HSI1_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_hsi1[] = {
	GOUT_BLK_HSI1_UID_SYSREG_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_hsi1[] = {
	GOUT_BLK_HSI1_UID_PPMU_HSI1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_PPMU_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_p_hsi1[] = {
	GOUT_BLK_HSI1_UID_XIU_P_HSI1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_pcie_gen3_0[] = {
	GOUT_BLK_HSI1_UID_PCIE_GEN3_0_IPCLKPORT_PCIE_PHY_TOP_INST_0_PIPE_PAL_PCIE_INST_0_I_APB_PCLK,
	GOUT_BLK_HSI1_UID_PCIE_GEN3_0_IPCLKPORT_PCIE_PAMIR_G3X2_DWC_PCIE_CTL_INST_0_DBI_ACLK_UG,
	GOUT_BLK_HSI1_UID_PCIE_GEN3_0_IPCLKPORT_PCIE_PAMIR_G3X2_DWC_PCIE_CTL_INST_0_MSTR_ACLK_UG,
	GOUT_BLK_HSI1_UID_PCIE_GEN3_0_IPCLKPORT_PCIE_003_PCIE_SUB_CTRL_INST_0_I_DRIVER_APB_CLK,
	GOUT_BLK_HSI1_UID_PCIE_GEN3_0_IPCLKPORT_PCIE_PAMIR_G3X2_DWC_PCIE_CTL_INST_0_SLV_ACLK_UG,
	GOUT_BLK_HSI1_UID_PCIE_GEN3_0_IPCLKPORT_PCIE_SUB_CTRL_A_G3X2_PHY_REFCLK_IN,
	GOUT_BLK_HSI1_UID_PCIE_GEN3_0_IPCLKPORT_PCIE_PHY_TOP_INST_0_PHY_UDBG_I_APB_PCLK,
	GOUT_BLK_HSI1_UID_PCIE_GEN3_0_IPCLKPORT_PCIE_PHY_TOP_INST_0_SF_PCIEPHY_X2_QCH_TM_WRAPPER_INST_0_I_APB_PCLK,
};
enum clk_id cmucal_vclk_ip_pcie_ia_gen3a_0[] = {
	GOUT_BLK_HSI1_UID_PCIE_IA_GEN3A_0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_hsi1[] = {
	GOUT_BLK_HSI1_UID_D_TZPC_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_hsi1[] = {
	GOUT_BLK_HSI1_UID_GPC_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_hsi1[] = {
	GOUT_BLK_HSI1_UID_SSMT_HSI1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_SSMT_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpio_hsi1[] = {
	GOUT_BLK_HSI1_UID_GPIO_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen3a_dbi_0[] = {
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN3A_DBI_0_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN3A_DBI_0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen3a_slv_0[] = {
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN3A_SLV_0_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN3A_SLV_0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pcie_ia_gen3a_0[] = {
	CLK_BLK_HSI1_UID_SSMT_PCIE_IA_GEN3A_0_IPCLKPORT_PCLK,
	CLK_BLK_HSI1_UID_SSMT_PCIE_IA_GEN3A_0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_as_apb_pciephy_hsi1[] = {
	GOUT_BLK_HSI1_UID_AS_APB_PCIEPHY_HSI1_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_hsi1[] = {
	CLK_BLK_HSI1_UID_SYSMMU_S0_HSI1_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_hsi1[] = {
	CLK_BLK_HSI1_UID_SLH_AXI_MI_P_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_hsi1_cu[] = {
	CLK_BLK_HSI1_UID_LH_AXI_SI_P_HSI1_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi1[] = {
	CLK_BLK_HSI1_UID_LH_AXI_MI_LP_CPUCL0_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_lp_aoc_hsi1[] = {
	CLK_BLK_HSI1_UID_SLH_AXI_MI_LP_AOC_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi1_cu[] = {
	CLK_BLK_HSI1_UID_LH_AXI_SI_LP_CPUCL0_HSI1_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_aoc_hsi1_cu[] = {
	CLK_BLK_HSI1_UID_LH_AXI_SI_LP_AOC_HSI1_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi1_cu[] = {
	CLK_BLK_HSI1_UID_LH_AXI_MI_LP_CPUCL0_HSI1_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_aoc_hsi1_cu[] = {
	CLK_BLK_HSI1_UID_LH_AXI_MI_LP_AOC_HSI1_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ld_hsi1_aoc[] = {
	CLK_BLK_HSI1_UID_LH_AXI_SI_LD_HSI1_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_hsi1[] = {
	CLK_BLK_HSI1_UID_SYSMMU_S0_PMMU0_HSI1_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_hsi1[] = {
	CLK_BLK_HSI1_UID_XIU_D1_HSI1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_blk_hsi1_frc_otp_deserial[] = {
	CLK_BLK_HSI1_UID_BLK_HSI1_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_hsi2_cmu_hsi2[] = {
	GOUT_BLK_HSI2_UID_HSI2_CMU_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_hsi2[] = {
	GOUT_BLK_HSI2_UID_SYSREG_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpio_hsi2[] = {
	GOUT_BLK_HSI2_UID_GPIO_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d_hsi2[] = {
	GOUT_BLK_HSI2_UID_LH_ACEL_SI_D_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_hsi2_cu[] = {
	GOUT_BLK_HSI2_UID_LH_AXI_MI_P_HSI2_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_hsi2[] = {
	GOUT_BLK_HSI2_UID_XIU_D0_HSI2_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_p_hsi2[] = {
	GOUT_BLK_HSI2_UID_XIU_P_HSI2_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_hsi2[] = {
	GOUT_BLK_HSI2_UID_PPMU_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_PPMU_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_pcie_gen3a_1[] = {
	GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_IPCLKPORT_PCIE_PHY_TOP_X1_INST_0_PIPE_PAL_PCIE_X1_INST_0_I_APB_PCLK,
	CLK_BLK_HSI2_UID_PCIE_GEN3A_1_IPCLKPORT_PCIE_SUB_CTRL_A_G3X1_PHY_REFCLK_IN,
	GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_IPCLKPORT_PCIE_QUADRA_G3X1_DWC_PCIE_CTL_INST_0_DBI_ACLK_UG,
	GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_IPCLKPORT_PCIE_QUADRA_G3X1_DWC_PCIE_CTL_INST_0_SLV_ACLK_UG,
	GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_IPCLKPORT_PCIE_QUADRA_G3X1_DWC_PCIE_CTL_INST_0_MSTR_ACLK_UG,
	GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_IPCLKPORT_PCIE_003_PCIE_SUB_CTRL_G3X1_INST_0_I_DRIVER_APB_CLK,
	GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_IPCLKPORT_PCIE_PHY_TOP_X1_INST_0_PHY_UDBG_I_APB_PCLK,
	CLK_BLK_HSI2_UID_PCIE_GEN3A_1_IPCLKPORT_PCIE_PHY_TOP_X1_INST_0_SF_PCIEPHY_X1_QCH_TM_WRAPPER_INST_0_I_APB_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_hsi2[] = {
	GOUT_BLK_HSI2_UID_SSMT_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_SSMT_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_pcie_ia_gen3a_1[] = {
	GOUT_BLK_HSI2_UID_PCIE_IA_GEN3A_1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_hsi2[] = {
	GOUT_BLK_HSI2_UID_D_TZPC_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ufs_embd[] = {
	GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_ACLK,
	GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_FMP_CLK,
	GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_CLK_UNIPRO,
};
enum clk_id cmucal_vclk_ip_pcie_ia_gen3b_1[] = {
	GOUT_BLK_HSI2_UID_PCIE_IA_GEN3B_1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_hsi2[] = {
	GOUT_BLK_HSI2_UID_GPC_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mmc_card[] = {
	GOUT_BLK_HSI2_UID_MMC_CARD_IPCLKPORT_I_ACLK,
	GOUT_BLK_HSI2_UID_MMC_CARD_IPCLKPORT_SDCLKIN,
};
enum clk_id cmucal_vclk_ip_qe_pcie_gen3a_hsi2[] = {
	GOUT_BLK_HSI2_UID_QE_PCIE_GEN3A_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_QE_PCIE_GEN3A_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_pcie_gen3b_hsi2[] = {
	GOUT_BLK_HSI2_UID_QE_PCIE_GEN3B_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_QE_PCIE_GEN3B_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_ufs_embd_hsi2[] = {
	GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen3a_dbi_1[] = {
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN3A_DBI_1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN3A_DBI_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen3a_slv_1[] = {
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN3A_SLV_1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN3A_SLV_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen3b_dbi_1[] = {
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN3B_DBI_1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN3B_DBI_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen3b_slv_1[] = {
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN3B_SLV_1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN3B_SLV_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_mmc_card_hsi2[] = {
	GOUT_BLK_HSI2_UID_QE_MMC_CARD_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_QE_MMC_CARD_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pcie_ia_gen3a_1[] = {
	CLK_BLK_HSI2_UID_SSMT_PCIE_IA_GEN3A_1_IPCLKPORT_ACLK,
	CLK_BLK_HSI2_UID_SSMT_PCIE_IA_GEN3A_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pcie_ia_gen3b_1[] = {
	CLK_BLK_HSI2_UID_SSMT_PCIE_IA_GEN3B_1_IPCLKPORT_ACLK,
	CLK_BLK_HSI2_UID_SSMT_PCIE_IA_GEN3B_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpio_hsi2ufs[] = {
	CLK_BLK_HSI2_UID_GPIO_HSI2UFS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_hsi2[] = {
	CLK_BLK_HSI2_UID_SYSMMU_S0_HSI2_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_hsi2[] = {
	CLK_BLK_HSI2_UID_SLH_AXI_MI_P_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_hsi2_cu[] = {
	CLK_BLK_HSI2_UID_LH_AXI_SI_P_HSI2_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_hsi2[] = {
	CLK_BLK_HSI2_UID_XIU_D1_HSI2_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi2[] = {
	CLK_BLK_HSI2_UID_LH_AXI_MI_LP_CPUCL0_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi2_cu[] = {
	CLK_BLK_HSI2_UID_LH_AXI_SI_LP_CPUCL0_HSI2_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_hsi2[] = {
	CLK_BLK_HSI2_UID_SYSMMU_S0_PMMU0_HSI2_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_pcie_gen3b_1[] = {
	CLK_BLK_HSI2_UID_PCIE_GEN3B_1_IPCLKPORT_PCIE_SUB_CTRL_A_G3X1_PHY_REFCLK_IN,
	CLK_BLK_HSI2_UID_PCIE_GEN3B_1_IPCLKPORT_PCIE_003_PCIE_SUB_CTRL_G3X1_INST_0_I_DRIVER_APB_CLK,
	CLK_BLK_HSI2_UID_PCIE_GEN3B_1_IPCLKPORT_PCIE_QUADRA_G3X1_DWC_PCIE_CTL_INST_0_DBI_ACLK_UG,
	CLK_BLK_HSI2_UID_PCIE_GEN3B_1_IPCLKPORT_PCIE_QUADRA_G3X1_DWC_PCIE_CTL_INST_0_MSTR_ACLK_UG,
	CLK_BLK_HSI2_UID_PCIE_GEN3B_1_IPCLKPORT_PCIE_QUADRA_G3X1_DWC_PCIE_CTL_INST_0_SLV_ACLK_UG,
	CLK_BLK_HSI2_UID_PCIE_GEN3B_1_IPCLKPORT_PCIE_PHY_TOP_X1_INST_0_PHY_UDBG_I_APB_PCLK,
	CLK_BLK_HSI2_UID_PCIE_GEN3B_1_IPCLKPORT_PCIE_PHY_TOP_X1_INST_0_PIPE_PAL_PCIE_X1_INST_0_I_APB_PCLK,
	CLK_BLK_HSI2_UID_PCIE_GEN3B_1_IPCLKPORT_PCIE_PHY_TOP_X1_INST_0_SF_PCIEPHY_X1_QCH_TM_WRAPPER_INST_0_I_APB_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi2_cu[] = {
	CLK_BLK_HSI2_UID_LH_AXI_MI_LP_CPUCL0_HSI2_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_as_apb_pciephy_0_hsi2[] = {
	CLK_BLK_HSI2_UID_AS_APB_PCIEPHY_0_HSI2_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_blk_hsi2_frc_otp_deserial[] = {
	CLK_BLK_HSI2_UID_BLK_HSI2_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d0_ispfe[] = {
	GOUT_BLK_ISPFE_UID_LH_AXI_SI_D0_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_ispfe[] = {
	GOUT_BLK_ISPFE_UID_SLH_AXI_MI_P_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_ispfe[] = {
	GOUT_BLK_ISPFE_UID_SYSREG_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ispfe_cmu_ispfe[] = {
	CLK_BLK_ISPFE_UID_ISPFE_CMU_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mipi_phy_link_wrap[] = {
	GOUT_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS1,
	GOUT_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS2,
	GOUT_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS3,
	GOUT_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS5,
	GOUT_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS4,
	GOUT_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS6,
	GOUT_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS7,
	GOUT_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS0,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS8,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS9,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS10,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS10,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS11,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_ACLK_CSIS11,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS0,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS1,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS2,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS3,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS4,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS5,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS6,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS7,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS8,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS9,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS10,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS10,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS11,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_LINK_PCLK_CSIS11,
	CLK_BLK_ISPFE_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_CSIS_PHY_S_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_ispfe[] = {
	GOUT_BLK_ISPFE_UID_D_TZPC_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_ispfe[] = {
	GOUT_BLK_ISPFE_UID_PPMU_D0_ISPFE_IPCLKPORT_PCLK,
	GOUT_BLK_ISPFE_UID_PPMU_D0_ISPFE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d3_ispfe[] = {
	GOUT_BLK_ISPFE_UID_LH_AXI_SI_D3_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_ispfe[] = {
	GOUT_BLK_ISPFE_UID_GPC_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_sysmmu_s0_ispfe_s1_ns[] = {
	GOUT_BLK_ISPFE_UID_AD_APB_SYSMMU_S0_ISPFE_S1_NS_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_ispfe[] = {
	GOUT_BLK_ISPFE_UID_PPMU_D1_ISPFE_IPCLKPORT_ACLK,
	GOUT_BLK_ISPFE_UID_PPMU_D1_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_pmmu0_ispfe[] = {
	GOUT_BLK_ISPFE_UID_SYSMMU_S1_PMMU0_ISPFE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_ispfe[] = {
	GOUT_BLK_ISPFE_UID_SSMT_D1_ISPFE_IPCLKPORT_ACLK,
	GOUT_BLK_ISPFE_UID_SSMT_D1_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_ispfe[] = {
	GOUT_BLK_ISPFE_UID_SSMT_D0_ISPFE_IPCLKPORT_ACLK,
	GOUT_BLK_ISPFE_UID_SSMT_D0_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_ispfe[] = {
	GOUT_BLK_ISPFE_UID_SYSMMU_S0_PMMU1_ISPFE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_ispfe[] = {
	GOUT_BLK_ISPFE_UID_SYSMMU_S0_ISPFE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s2_pmmu0_ispfe[] = {
	GOUT_BLK_ISPFE_UID_SYSMMU_S2_PMMU0_ISPFE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_ispfe[] = {
	GOUT_BLK_ISPFE_UID_XIU_D0_ISPFE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d1_ispfe[] = {
	GOUT_BLK_ISPFE_UID_LH_AXI_SI_D1_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_ispfe[] = {
	GOUT_BLK_ISPFE_UID_SYSMMU_S0_PMMU0_ISPFE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d2_ispfe[] = {
	GOUT_BLK_ISPFE_UID_LH_AXI_SI_D2_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ispfe[] = {
	GOUT_BLK_ISPFE_UID_ISPFE_IPCLKPORT_CLK_ISPFE,
	CLK_BLK_ISPFE_UID_ISPFE_IPCLKPORT_CLK_REF,
};
enum clk_id cmucal_vclk_ip_qe_d0_ispfe[] = {
	GOUT_BLK_ISPFE_UID_QE_D0_ISPFE_IPCLKPORT_ACLK,
	GOUT_BLK_ISPFE_UID_QE_D0_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d1_ispfe[] = {
	GOUT_BLK_ISPFE_UID_QE_D1_ISPFE_IPCLKPORT_ACLK,
	GOUT_BLK_ISPFE_UID_QE_D1_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d3_ispfe[] = {
	GOUT_BLK_ISPFE_UID_QE_D3_ISPFE_IPCLKPORT_ACLK,
	GOUT_BLK_ISPFE_UID_QE_D3_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d2_ispfe[] = {
	GOUT_BLK_ISPFE_UID_QE_D2_ISPFE_IPCLKPORT_ACLK,
	GOUT_BLK_ISPFE_UID_QE_D2_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d2_ispfe[] = {
	CLK_BLK_ISPFE_UID_SSMT_D2_ISPFE_IPCLKPORT_ACLK,
	CLK_BLK_ISPFE_UID_SSMT_D2_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d3_ispfe[] = {
	CLK_BLK_ISPFE_UID_SSMT_D3_ISPFE_IPCLKPORT_ACLK,
	CLK_BLK_ISPFE_UID_SSMT_D3_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_ispfe[] = {
	CLK_BLK_ISPFE_UID_PPMU_D2_ISPFE_IPCLKPORT_ACLK,
	CLK_BLK_ISPFE_UID_PPMU_D2_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d3_ispfe[] = {
	CLK_BLK_ISPFE_UID_PPMU_D3_ISPFE_IPCLKPORT_ACLK,
	CLK_BLK_ISPFE_UID_PPMU_D3_ISPFE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_ispfe[] = {
	CLK_BLK_ISPFE_UID_UASC_ISPFE_IPCLKPORT_PCLK,
	CLK_BLK_ISPFE_UID_UASC_ISPFE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_ispfe[] = {
	CLK_BLK_ISPFE_UID_XIU_D1_ISPFE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d2_ispfe[] = {
	CLK_BLK_ISPFE_UID_XIU_D2_ISPFE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_ispfe[] = {
	CLK_BLK_ISPFE_UID_SYSMMU_S1_ISPFE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s2_ispfe[] = {
	CLK_BLK_ISPFE_UID_SYSMMU_S2_ISPFE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ip_ispfe[] = {
	CLK_BLK_ISPFE_UID_LH_AXI_SI_IP_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_ip_ispfe[] = {
	CLK_BLK_ISPFE_UID_LH_AXI_MI_IP_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_mipi_phy[] = {
	CLK_BLK_ISPFE_UID_AD_APB_MIPI_PHY_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_baaw_ispfe[] = {
	CLK_BLK_ISPFE_UID_BAAW_ISPFE_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_blk_ispfe_frc_otp_deserial[] = {
	CLK_BLK_ISPFE_UID_BLK_ISPFE_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_mcsc[] = {
	GOUT_BLK_MCSC_UID_SLH_AXI_MI_P_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d0_mcsc[] = {
	GOUT_BLK_MCSC_UID_LH_AXI_SI_D0_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_mcsc[] = {
	GOUT_BLK_MCSC_UID_SYSREG_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mcsc_cmu_mcsc[] = {
	CLK_BLK_MCSC_UID_MCSC_CMU_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_otf_yuvp_mcsc[] = {
	GOUT_BLK_MCSC_UID_LH_AST_MI_L_OTF_YUVP_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_mcsc[] = {
	GOUT_BLK_MCSC_UID_D_TZPC_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_mcsc[] = {
	GOUT_BLK_MCSC_UID_GPC_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_mcsc[] = {
	GOUT_BLK_MCSC_UID_SSMT_D0_MCSC_IPCLKPORT_PCLK,
	CLK_BLK_MCSC_UID_SSMT_D0_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_mcsc[] = {
	GOUT_BLK_MCSC_UID_SYSMMU_S0_MCSC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d3_mcsc[] = {
	GOUT_BLK_MCSC_UID_PPMU_D3_MCSC_IPCLKPORT_PCLK,
	CLK_BLK_MCSC_UID_PPMU_D3_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d2_mcsc[] = {
	GOUT_BLK_MCSC_UID_SSMT_D2_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_SSMT_D2_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_mcsc[] = {
	GOUT_BLK_MCSC_UID_PPMU_D2_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_PPMU_D2_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_mcsc[] = {
	GOUT_BLK_MCSC_UID_PPMU_D0_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_PPMU_D0_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_otf_tnr_mcsc[] = {
	GOUT_BLK_MCSC_UID_LH_AST_MI_L_OTF_TNR_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d3_mcsc[] = {
	GOUT_BLK_MCSC_UID_SSMT_D3_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_SSMT_D3_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_mcsc[] = {
	GOUT_BLK_MCSC_UID_PPMU_D1_MCSC_IPCLKPORT_PCLK,
	CLK_BLK_MCSC_UID_PPMU_D1_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_mcsc[] = {
	GOUT_BLK_MCSC_UID_SSMT_D1_MCSC_IPCLKPORT_PCLK,
	CLK_BLK_MCSC_UID_SSMT_D1_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d6_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D6_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_QE_D6_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d0_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D0_MCSC_IPCLKPORT_PCLK,
	CLK_BLK_MCSC_UID_QE_D0_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d1_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D1_MCSC_IPCLKPORT_PCLK,
	CLK_BLK_MCSC_UID_QE_D1_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d2_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D2_MCSC_IPCLKPORT_PCLK,
	CLK_BLK_MCSC_UID_QE_D2_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d4_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D4_MCSC_IPCLKPORT_PCLK,
	CLK_BLK_MCSC_UID_QE_D4_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d3_mcsc[] = {
	CLK_BLK_MCSC_UID_QE_D3_MCSC_IPCLKPORT_ACLK,
	CLK_BLK_MCSC_UID_QE_D3_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d5_mcsc[] = {
	CLK_BLK_MCSC_UID_QE_D5_MCSC_IPCLKPORT_PCLK,
	CLK_BLK_MCSC_UID_QE_D5_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_mcsc[] = {
	CLK_BLK_MCSC_UID_AD_APB_MCSC_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ppmu_d4_mcsc[] = {
	CLK_BLK_MCSC_UID_PPMU_D4_MCSC_IPCLKPORT_ACLK,
	CLK_BLK_MCSC_UID_PPMU_D4_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d5_mcsc[] = {
	CLK_BLK_MCSC_UID_PPMU_D5_MCSC_IPCLKPORT_ACLK,
	CLK_BLK_MCSC_UID_PPMU_D5_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d6_mcsc[] = {
	CLK_BLK_MCSC_UID_PPMU_D6_MCSC_IPCLKPORT_ACLK,
	CLK_BLK_MCSC_UID_PPMU_D6_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_mcsc[] = {
	CLK_BLK_MCSC_UID_SYSMMU_S0_PMMU0_MCSC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d4_mcsc[] = {
	CLK_BLK_MCSC_UID_SSMT_D4_MCSC_IPCLKPORT_ACLK,
	CLK_BLK_MCSC_UID_SSMT_D4_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d5_mcsc[] = {
	CLK_BLK_MCSC_UID_SSMT_D5_MCSC_IPCLKPORT_ACLK,
	CLK_BLK_MCSC_UID_SSMT_D5_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d6_mcsc[] = {
	CLK_BLK_MCSC_UID_SSMT_D6_MCSC_IPCLKPORT_ACLK,
	CLK_BLK_MCSC_UID_SSMT_D6_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mcsc[] = {
	CLK_BLK_MCSC_UID_MCSC_IPCLKPORT_C2R_CLK,
	CLK_BLK_MCSC_UID_MCSC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d1_mcsc[] = {
	CLK_BLK_MCSC_UID_LH_AXI_SI_D1_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_mcsc[] = {
	CLK_BLK_MCSC_UID_SYSMMU_S0_PMMU1_MCSC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_mcsc[] = {
	CLK_BLK_MCSC_UID_XIU_D1_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d2_mcsc[] = {
	CLK_BLK_MCSC_UID_XIU_D2_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_blk_mcsc_frc_otp_deserial[] = {
	CLK_BLK_MCSC_UID_BLK_MCSC_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_mcsc[] = {
	CLK_BLK_MCSC_UID_XIU_D0_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_mfc_cmu_mfc[] = {
	CLK_BLK_MFC_UID_MFC_CMU_MFC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_as_apb_mfc[] = {
	GOUT_BLK_MFC_UID_AS_APB_MFC_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_sysreg_mfc[] = {
	GOUT_BLK_MFC_UID_SYSREG_MFC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d0_mfc[] = {
	GOUT_BLK_MFC_UID_LH_AXI_SI_D0_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d1_mfc[] = {
	GOUT_BLK_MFC_UID_LH_AXI_SI_D1_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_mfc[] = {
	GOUT_BLK_MFC_UID_SLH_AXI_MI_P_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_mfc[] = {
	GOUT_BLK_MFC_UID_SYSMMU_S0_PMMU0_MFC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_mfc[] = {
	GOUT_BLK_MFC_UID_SYSMMU_S0_PMMU1_MFC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_mfc[] = {
	GOUT_BLK_MFC_UID_PPMU_D0_MFC_IPCLKPORT_ACLK,
	GOUT_BLK_MFC_UID_PPMU_D0_MFC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_mfc[] = {
	GOUT_BLK_MFC_UID_PPMU_D1_MFC_IPCLKPORT_ACLK,
	GOUT_BLK_MFC_UID_PPMU_D1_MFC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_mfc[] = {
	GOUT_BLK_MFC_UID_SSMT_D0_MFC_IPCLKPORT_PCLK,
	GOUT_BLK_MFC_UID_SSMT_D0_MFC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_mfc[] = {
	GOUT_BLK_MFC_UID_MFC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_mfc[] = {
	GOUT_BLK_MFC_UID_D_TZPC_MFC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_mfc[] = {
	GOUT_BLK_MFC_UID_SSMT_D1_MFC_IPCLKPORT_ACLK,
	GOUT_BLK_MFC_UID_SSMT_D1_MFC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_mfc[] = {
	GOUT_BLK_MFC_UID_GPC_MFC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_mfc[] = {
	CLK_BLK_MFC_UID_SYSMMU_S0_MFC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_mfc[] = {
	CLK_BLK_MFC_UID_XIU_D_MFC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_blk_mfc_frc_otp_deserial[] = {
	CLK_BLK_MFC_UID_BLK_MFC_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mif_cmu_mif[] = {
	CLK_BLK_MIF_UID_MIF_CMU_MIF_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_mif[] = {
	GOUT_BLK_MIF_UID_SYSREG_MIF_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_mif_cu[] = {
	GOUT_BLK_MIF_UID_LH_AXI_MI_P_MIF_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_axi2apb_p_mif[] = {
	GOUT_BLK_MIF_UID_AXI2APB_P_MIF_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qch_adapter_ppc_debug[] = {
	GOUT_BLK_MIF_UID_QCH_ADAPTER_PPC_DEBUG_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_mif[] = {
	GOUT_BLK_MIF_UID_GPC_MIF_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_mif[] = {
	GOUT_BLK_MIF_UID_D_TZPC_MIF_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_debug[] = {
	CLK_BLK_MIF_UID_PPC_DEBUG_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_gen_wren_secure[] = {
	GOUT_BLK_MIF_UID_GEN_WREN_SECURE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_mif[] = {
	CLK_BLK_MIF_UID_SLH_AXI_MI_P_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_mif_cu[] = {
	CLK_BLK_MIF_UID_LH_AXI_SI_P_MIF_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_qch_adapter_ddrphy[] = {
	CLK_BLK_MIF_UID_QCH_ADAPTER_DDRPHY_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qch_adapter_smc[] = {
	CLK_BLK_MIF_UID_QCH_ADAPTER_SMC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_blk_mif_frc_otp_deserial[] = {
	CLK_BLK_MIF_UID_BLK_MIF_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_misc[] = {
	GOUT_BLK_MISC_UID_SYSREG_MISC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_wdt_cluster1[] = {
	GOUT_BLK_MISC_UID_WDT_CLUSTER1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_wdt_cluster0[] = {
	GOUT_BLK_MISC_UID_WDT_CLUSTER0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_otp_con_bira[] = {
	GOUT_BLK_MISC_UID_OTP_CON_BIRA_IPCLKPORT_PCLK,
	CLK_BLK_MISC_UID_OTP_CON_BIRA_IPCLKPORT_I_OSCCLK,
};
enum clk_id cmucal_vclk_ip_gic[] = {
	GOUT_BLK_MISC_UID_GIC_IPCLKPORT_GICCLK,
};
enum clk_id cmucal_vclk_ip_mct[] = {
	GOUT_BLK_MISC_UID_MCT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_otp_con_top[] = {
	GOUT_BLK_MISC_UID_OTP_CON_TOP_IPCLKPORT_PCLK,
	CLK_BLK_MISC_UID_OTP_CON_TOP_IPCLKPORT_I_OSCCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_misc[] = {
	GOUT_BLK_MISC_UID_D_TZPC_MISC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_tmu_sub[] = {
	GOUT_BLK_MISC_UID_TMU_SUB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_tmu_top[] = {
	GOUT_BLK_MISC_UID_TMU_TOP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_dit[] = {
	GOUT_BLK_MISC_UID_DIT_IPCLKPORT_ICLKL2A,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_misc_cu[] = {
	GOUT_BLK_MISC_UID_LH_AXI_MI_P_MISC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d_misc[] = {
	GOUT_BLK_MISC_UID_LH_ACEL_SI_D_MISC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_pdma0[] = {
	GOUT_BLK_MISC_UID_PDMA0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_misc[] = {
	GOUT_BLK_MISC_UID_PPMU_MISC_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_PPMU_MISC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_dit[] = {
	GOUT_BLK_MISC_UID_QE_DIT_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_QE_DIT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_pdma0[] = {
	GOUT_BLK_MISC_UID_QE_PDMA0_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_QE_PDMA0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_misc_cmu_misc[] = {
	CLK_BLK_MISC_UID_MISC_CMU_MISC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_rtic[] = {
	GOUT_BLK_MISC_UID_QE_RTIC_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_QE_RTIC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_spdma0[] = {
	GOUT_BLK_MISC_UID_QE_SPDMA0_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_QE_SPDMA0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_sc[] = {
	GOUT_BLK_MISC_UID_QE_SC_IPCLKPORT_PCLK,
	GOUT_BLK_MISC_UID_QE_SC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_rtic[] = {
	GOUT_BLK_MISC_UID_RTIC_IPCLKPORT_I_ACLK,
	GOUT_BLK_MISC_UID_RTIC_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_spdma0[] = {
	GOUT_BLK_MISC_UID_SPDMA0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sc[] = {
	GOUT_BLK_MISC_UID_SC_IPCLKPORT_I_PCLK,
	CLK_BLK_MISC_UID_SC_IPCLKPORT_I_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_sc[] = {
	GOUT_BLK_MISC_UID_SSMT_SC_IPCLKPORT_PCLK,
	GOUT_BLK_MISC_UID_SSMT_SC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_gpc_misc[] = {
	GOUT_BLK_MISC_UID_GPC_MISC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_dit[] = {
	GOUT_BLK_MISC_UID_AD_APB_DIT_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ad_apb_puf[] = {
	GOUT_BLK_MISC_UID_AD_APB_PUF_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_icc_cluster0_gic_cu[] = {
	GOUT_BLK_MISC_UID_LH_AST_MI_L_ICC_CLUSTER0_GIC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_id_sc[] = {
	GOUT_BLK_MISC_UID_LH_AXI_MI_ID_SC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_iri_gic_cluster0_cd[] = {
	GOUT_BLK_MISC_UID_LH_AST_SI_L_IRI_GIC_CLUSTER0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_id_sc[] = {
	GOUT_BLK_MISC_UID_LH_AXI_SI_ID_SC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_puf[] = {
	GOUT_BLK_MISC_UID_PUF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_misc[] = {
	GOUT_BLK_MISC_UID_XIU_D0_MISC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_misc[] = {
	GOUT_BLK_MISC_UID_SYSMMU_S0_PMMU0_MISC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_misc_gic_cu[] = {
	GOUT_BLK_MISC_UID_LH_AXI_MI_P_MISC_GIC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_rtic[] = {
	GOUT_BLK_MISC_UID_SSMT_RTIC_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_SSMT_RTIC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_spdma0[] = {
	GOUT_BLK_MISC_UID_SSMT_SPDMA0_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_SSMT_SPDMA0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pdma0[] = {
	GOUT_BLK_MISC_UID_SSMT_PDMA0_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_SSMT_PDMA0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_dit[] = {
	GOUT_BLK_MISC_UID_SSMT_DIT_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_SSMT_DIT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_iri_gic_cluster0_cd[] = {
	CLK_BLK_MISC_UID_LH_AST_MI_L_IRI_GIC_CLUSTER0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_iri_gic_cluster0[] = {
	CLK_BLK_MISC_UID_LH_AST_SI_L_IRI_GIC_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_icc_cluster0_gic[] = {
	CLK_BLK_MISC_UID_LH_AST_MI_L_ICC_CLUSTER0_GIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_icc_cluster0_gic_cu[] = {
	CLK_BLK_MISC_UID_LH_AST_SI_L_ICC_CLUSTER0_GIC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_misc[] = {
	CLK_BLK_MISC_UID_SLH_AXI_MI_P_MISC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_misc_cu[] = {
	CLK_BLK_MISC_UID_LH_AXI_SI_P_MISC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_spdma1[] = {
	CLK_BLK_MISC_UID_SPDMA1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_pdma1[] = {
	CLK_BLK_MISC_UID_QE_PDMA1_IPCLKPORT_PCLK,
	CLK_BLK_MISC_UID_QE_PDMA1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_spdma1[] = {
	CLK_BLK_MISC_UID_QE_SPDMA1_IPCLKPORT_ACLK,
	CLK_BLK_MISC_UID_QE_SPDMA1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pdma1[] = {
	CLK_BLK_MISC_UID_SSMT_PDMA1_IPCLKPORT_ACLK,
	CLK_BLK_MISC_UID_SSMT_PDMA1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_spdma1[] = {
	CLK_BLK_MISC_UID_SSMT_SPDMA1_IPCLKPORT_ACLK,
	CLK_BLK_MISC_UID_SSMT_SPDMA1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_pdma1[] = {
	CLK_BLK_MISC_UID_PDMA1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_misc_gic[] = {
	CLK_BLK_MISC_UID_SLH_AXI_MI_P_MISC_GIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_misc_gic_cu[] = {
	CLK_BLK_MISC_UID_LH_AXI_SI_P_MISC_GIC_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_misc[] = {
	CLK_BLK_MISC_UID_XIU_D1_MISC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_misc[] = {
	CLK_BLK_MISC_UID_SYSMMU_S0_MISC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_otp_con_bisr[] = {
	CLK_BLK_MISC_UID_OTP_CON_BISR_IPCLKPORT_PCLK,
	CLK_BLK_MISC_UID_OTP_CON_BISR_IPCLKPORT_I_OSCCLK,
};
enum clk_id cmucal_vclk_ip_blk_misc_frc_otp_deserial[] = {
	CLK_BLK_MISC_UID_BLK_MISC_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mct_sub[] = {
	CLK_BLK_MISC_UID_MCT_SUB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mct_v41[] = {
	CLK_BLK_MISC_UID_MCT_V41_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_nocl0[] = {
	GOUT_BLK_NOCL0_UID_SYSREG_NOCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_p_nocl0[] = {
	GOUT_BLK_NOCL0_UID_TREX_P_NOCL0_IPCLKPORT_PCLK,
	GOUT_BLK_NOCL0_UID_TREX_P_NOCL0_IPCLKPORT_ACLK_D_NOCL0,
	GOUT_BLK_NOCL0_UID_TREX_P_NOCL0_IPCLKPORT_ACLK_P_NOCL0,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d1_cpucl0[] = {
	GOUT_BLK_NOCL0_UID_LH_ACEL_MI_D1_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_trex_d_nocl0[] = {
	GOUT_BLK_NOCL0_UID_TREX_D_NOCL0_IPCLKPORT_PCLK,
	GOUT_BLK_NOCL0_UID_TREX_D_NOCL0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_nocl0[] = {
	GOUT_BLK_NOCL0_UID_D_TZPC_NOCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_bdu[] = {
	GOUT_BLK_NOCL0_UID_BDU_IPCLKPORT_I_CLK,
	GOUT_BLK_NOCL0_UID_BDU_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_nocl0[] = {
	GOUT_BLK_NOCL0_UID_GPC_NOCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl0_alive_p[] = {
	GOUT_BLK_NOCL0_UID_PPMU_NOCL0_ALIVE_P_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPMU_NOCL0_ALIVE_P_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl0_cpucl0_p[] = {
	GOUT_BLK_NOCL0_UID_PPMU_NOCL0_CPUCL0_P_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPMU_NOCL0_CPUCL0_P_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sfr_apbif_cmu_topc[] = {
	GOUT_BLK_NOCL0_UID_SFR_APBIF_CMU_TOPC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl1a_m0_event[] = {
	GOUT_BLK_NOCL0_UID_PPC_NOCL1A_M0_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPC_NOCL1A_M0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl1a_m1_event[] = {
	GOUT_BLK_NOCL0_UID_PPC_NOCL1A_M1_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPC_NOCL1A_M1_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl1a_m2_event[] = {
	GOUT_BLK_NOCL0_UID_PPC_NOCL1A_M2_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPC_NOCL1A_M2_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl1a_m3_event[] = {
	GOUT_BLK_NOCL0_UID_PPC_NOCL1A_M3_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPC_NOCL1A_M3_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl1b_m0_event[] = {
	GOUT_BLK_NOCL0_UID_PPC_NOCL1B_M0_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPC_NOCL1B_M0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cpucl0_d0_cycle[] = {
	GOUT_BLK_NOCL0_UID_PPC_CPUCL0_D0_CYCLE_IPCLKPORT_PCLK,
	CLK_BLK_NOCL0_UID_PPC_CPUCL0_D0_CYCLE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_slc_cb_top[] = {
	GOUT_BLK_NOCL0_UID_SLC_CB_TOP_IPCLKPORT_I_ACLK,
};
enum clk_id cmucal_vclk_ip_ppc_cpucl0_d0_event[] = {
	GOUT_BLK_NOCL0_UID_PPC_CPUCL0_D0_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPC_CPUCL0_D0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cpucl0_d2_event[] = {
	GOUT_BLK_NOCL0_UID_PPC_CPUCL0_D2_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPC_CPUCL0_D2_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cpucl0_d3_event[] = {
	GOUT_BLK_NOCL0_UID_PPC_CPUCL0_D3_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL0_UID_PPC_CPUCL0_D3_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl1a_m0_cycle[] = {
	GOUT_BLK_NOCL0_UID_PPC_NOCL1A_M0_CYCLE_IPCLKPORT_PCLK,
	CLK_BLK_NOCL0_UID_PPC_NOCL1A_M0_CYCLE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppc_dbg_cc[] = {
	GOUT_BLK_NOCL0_UID_PPC_DBG_CC_IPCLKPORT_PCLK,
	GOUT_BLK_NOCL0_UID_PPC_DBG_CC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_mpace_asb_d0_mif[] = {
	GOUT_BLK_NOCL0_UID_MPACE_ASB_D0_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mpace_asb_d1_mif[] = {
	GOUT_BLK_NOCL0_UID_MPACE_ASB_D1_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mpace_asb_d2_mif[] = {
	GOUT_BLK_NOCL0_UID_MPACE_ASB_D2_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mpace_asb_d3_mif[] = {
	GOUT_BLK_NOCL0_UID_MPACE_ASB_D3_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_cpucl0_d1_event[] = {
	GOUT_BLK_NOCL0_UID_PPC_CPUCL0_D1_EVENT_IPCLKPORT_PCLK,
	GOUT_BLK_NOCL0_UID_PPC_CPUCL0_D1_EVENT_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_slc_ch_top[] = {
	GOUT_BLK_NOCL0_UID_SLC_CH_TOP_IPCLKPORT_I_ACLK,
	GOUT_BLK_NOCL0_UID_SLC_CH_TOP_IPCLKPORT_I_DCLK,
};
enum clk_id cmucal_vclk_ip_gray2bin_atb_tsvalue[] = {
	GOUT_BLK_NOCL0_UID_GRAY2BIN_ATB_TSVALUE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_g_nocl0[] = {
	GOUT_BLK_NOCL0_UID_SLH_AXI_MI_G_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl1a_cu[] = {
	CLK_BLK_NOCL0_UID_LH_AST_MI_G_NOCL1A_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl1b_cu[] = {
	CLK_BLK_NOCL0_UID_LH_AST_MI_G_NOCL1B_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl2aa_cu[] = {
	CLK_BLK_NOCL0_UID_LH_AST_MI_G_NOCL2AA_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_alive[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_ALIVE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_cpucl0[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_eh[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_EH_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_misc_gic[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_MISC_GIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_mif0[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_MIF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_mif1[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_MIF1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_mif2[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_MIF2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_mif3[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_MIF3_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_misc[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_MISC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_peric0[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_PERIC0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_peric1[] = {
	CLK_BLK_NOCL0_UID_SLH_AXI_SI_P_PERIC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_t_bdu[] = {
	CLK_BLK_NOCL0_UID_LH_ATB_SI_T_BDU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_t_slc[] = {
	CLK_BLK_NOCL0_UID_LH_ATB_SI_T_SLC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_alive_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_ALIVE_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_cpucl0_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_misc_gic_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_MISC_GIC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_mif0_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_MIF0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_mif1_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_MIF1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_mif2_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_MIF2_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_mif3_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_MIF3_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_misc_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_MISC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_peric0_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_PERIC0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_peric1_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_PERIC1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_t_bdu_cd[] = {
	CLK_BLK_NOCL0_UID_LH_ATB_SI_T_BDU_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_t_slc_cd[] = {
	CLK_BLK_NOCL0_UID_LH_ATB_SI_T_SLC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_alive_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_ALIVE_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_cpucl0_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_misc_gic_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_MISC_GIC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_mif0_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_MIF0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_mif1_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_MIF1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_mif2_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_MIF2_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_mif3_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_MIF3_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_misc_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_MISC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_peric0_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_PERIC0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_peric1_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_PERIC1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_t_bdu_cd[] = {
	CLK_BLK_NOCL0_UID_LH_ATB_MI_T_BDU_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_t_slc_cd[] = {
	CLK_BLK_NOCL0_UID_LH_ATB_MI_T_SLC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl1a[] = {
	CLK_BLK_NOCL0_UID_LH_AST_MI_G_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl1b[] = {
	CLK_BLK_NOCL0_UID_LH_AST_MI_G_NOCL1B_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl2aa[] = {
	CLK_BLK_NOCL0_UID_LH_AST_MI_G_NOCL2AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl1a_cu[] = {
	CLK_BLK_NOCL0_UID_LH_AST_SI_G_NOCL1A_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl1b_cu[] = {
	CLK_BLK_NOCL0_UID_LH_AST_SI_G_NOCL1B_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl2aa_cu[] = {
	CLK_BLK_NOCL0_UID_LH_AST_SI_G_NOCL2AA_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d2_cpucl0[] = {
	CLK_BLK_NOCL0_UID_LH_ACEL_MI_D2_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d3_cpucl0[] = {
	CLK_BLK_NOCL0_UID_LH_ACEL_MI_D3_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl0_ioc0[] = {
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_IOC0_IPCLKPORT_PCLK,
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_IOC0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl0_ioc1[] = {
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_IOC1_IPCLKPORT_PCLK,
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_IOC1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl0_s0[] = {
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_S0_IPCLKPORT_PCLK,
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_S0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl0_s1[] = {
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_S1_IPCLKPORT_PCLK,
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_S1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl0_s2[] = {
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_S2_IPCLKPORT_PCLK,
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_S2_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl0_s3[] = {
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_S3_IPCLKPORT_PCLK,
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_S3_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl2ab[] = {
	CLK_BLK_NOCL0_UID_LH_AST_MI_G_NOCL2AB_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_nocl0_cmu_nocl0[] = {
	CLK_BLK_NOCL0_UID_NOCL0_CMU_NOCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl2ab_cu[] = {
	CLK_BLK_NOCL0_UID_LH_AST_SI_G_NOCL2AB_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl2ab_cu[] = {
	CLK_BLK_NOCL0_UID_LH_AST_MI_G_NOCL2AB_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d0_nocl0_cpucl0[] = {
	CLK_BLK_NOCL0_UID_LH_ACEL_SI_D0_NOCL0_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d1_nocl0_cpucl0[] = {
	CLK_BLK_NOCL0_UID_LH_ACEL_SI_D1_NOCL0_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_d0_nocl1a_nocl0[] = {
	CLK_BLK_NOCL0_UID_LH_TAXI_MI_D0_NOCL1A_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_p_nocl0_nocl1a[] = {
	CLK_BLK_NOCL0_UID_LH_TAXI_SI_P_NOCL0_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_d_nocl1b_nocl0[] = {
	CLK_BLK_NOCL0_UID_LH_TAXI_MI_D_NOCL1B_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_d1_nocl1a_nocl0[] = {
	CLK_BLK_NOCL0_UID_LH_TAXI_MI_D1_NOCL1A_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_d2_nocl1a_nocl0[] = {
	CLK_BLK_NOCL0_UID_LH_TAXI_MI_D2_NOCL1A_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_d3_nocl1a_nocl0[] = {
	CLK_BLK_NOCL0_UID_LH_TAXI_MI_D3_NOCL1A_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_p_nocl0_nocl1b[] = {
	CLK_BLK_NOCL0_UID_LH_TAXI_SI_P_NOCL0_NOCL1B_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_p_nocl0_nocl2aa[] = {
	CLK_BLK_NOCL0_UID_LH_TAXI_SI_P_NOCL0_NOCL2AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_p_nocl0_nocl2ab[] = {
	CLK_BLK_NOCL0_UID_LH_TAXI_SI_P_NOCL0_NOCL2AB_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl1b_m0_cycle[] = {
	CLK_BLK_NOCL0_UID_PPC_NOCL1B_M0_CYCLE_IPCLKPORT_ACLK,
	CLK_BLK_NOCL0_UID_PPC_NOCL1B_M0_CYCLE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl0_dp[] = {
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_DP_IPCLKPORT_PCLK,
	CLK_BLK_NOCL0_UID_PPMU_NOCL0_DP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_eh_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_SI_P_EH_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_eh_cd[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_EH_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_cpucl0_nocl0[] = {
	CLK_BLK_NOCL0_UID_LH_AXI_MI_P_CPUCL0_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slc_ch1[] = {
	CLK_BLK_NOCL0_UID_SLC_CH1_IPCLKPORT_I_DCLK,
	CLK_BLK_NOCL0_UID_SLC_CH1_IPCLKPORT_I_ACLK,
};
enum clk_id cmucal_vclk_ip_slc_ch2[] = {
	CLK_BLK_NOCL0_UID_SLC_CH2_IPCLKPORT_I_ACLK,
	CLK_BLK_NOCL0_UID_SLC_CH2_IPCLKPORT_I_DCLK,
};
enum clk_id cmucal_vclk_ip_slc_ch3[] = {
	CLK_BLK_NOCL0_UID_SLC_CH3_IPCLKPORT_I_ACLK,
	CLK_BLK_NOCL0_UID_SLC_CH3_IPCLKPORT_I_DCLK,
};
enum clk_id cmucal_vclk_ip_blk_nocl0_frc_otp_deserial[] = {
	CLK_BLK_NOCL0_UID_BLK_NOCL0_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ld_slc_frc_otp_deserial[] = {
	CLK_BLK_NOCL0_UID_LD_SLC_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ld_slc1_frc_otp_deserial[] = {
	CLK_BLK_NOCL0_UID_LD_SLC1_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ld_slc2_frc_otp_deserial[] = {
	CLK_BLK_NOCL0_UID_LD_SLC2_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ld_slc3_frc_otp_deserial[] = {
	CLK_BLK_NOCL0_UID_LD_SLC3_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl0_io0_cycle[] = {
	CLK_BLK_NOCL0_UID_PPC_NOCL0_IO0_CYCLE_IPCLKPORT_ACLK,
	CLK_BLK_NOCL0_UID_PPC_NOCL0_IO0_CYCLE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl0_io0_event[] = {
	CLK_BLK_NOCL0_UID_PPC_NOCL0_IO0_EVENT_IPCLKPORT_ACLK,
	CLK_BLK_NOCL0_UID_PPC_NOCL0_IO0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl0_io1_event[] = {
	CLK_BLK_NOCL0_UID_PPC_NOCL0_IO1_EVENT_IPCLKPORT_ACLK,
	CLK_BLK_NOCL0_UID_PPC_NOCL0_IO1_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d0_cpucl0[] = {
	CLK_BLK_NOCL0_UID_LH_ACEL_MI_D0_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_trex_d_nocl1a[] = {
	GOUT_BLK_NOCL1A_UID_TREX_D_NOCL1A_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_TREX_D_NOCL1A_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_nocl1a[] = {
	GOUT_BLK_NOCL1A_UID_SYSREG_NOCL1A_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d0_g3d[] = {
	GOUT_BLK_NOCL1A_UID_LH_ACEL_MI_D0_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_nocl1a[] = {
	GOUT_BLK_NOCL1A_UID_D_TZPC_NOCL1A_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d1_g3d[] = {
	GOUT_BLK_NOCL1A_UID_LH_ACEL_MI_D1_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d2_g3d[] = {
	GOUT_BLK_NOCL1A_UID_LH_ACEL_MI_D2_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d3_g3d[] = {
	GOUT_BLK_NOCL1A_UID_LH_ACEL_MI_D3_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_g3d_cd[] = {
	GOUT_BLK_NOCL1A_UID_LH_AXI_SI_P_G3D_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_nocl1a[] = {
	GOUT_BLK_NOCL1A_UID_GPC_NOCL1A_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_p_nocl1a[] = {
	GOUT_BLK_NOCL1A_UID_TREX_P_NOCL1A_IPCLKPORT_PCLK,
	GOUT_BLK_NOCL1A_UID_TREX_P_NOCL1A_IPCLKPORT_ACLK_P_NOCL1A,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl1a_cd[] = {
	GOUT_BLK_NOCL1A_UID_LH_AST_SI_G_NOCL1A_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl2aa_s0_event[] = {
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AA_S0_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AA_S0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl2aa_s1_event[] = {
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AA_S1_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AA_S1_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl2ab_s0_event[] = {
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AB_S0_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AB_S0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl2ab_s1_event[] = {
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AB_S1_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AB_S1_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_g3d_d0_event[] = {
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D0_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_g3d_d1_event[] = {
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D1_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D1_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_g3d_d2_event[] = {
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D2_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D2_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_g3d_d3_event[] = {
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D3_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D3_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_tpu_d0_event[] = {
	GOUT_BLK_NOCL1A_UID_PPC_TPU_D0_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_TPU_D0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl2aa_s0_cycle[] = {
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AA_S0_CYCLE_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_NOCL2AA_S0_CYCLE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_g3d_d0_cycle[] = {
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D0_CYCLE_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_G3D_D0_CYCLE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_tpu_d0_cycle[] = {
	GOUT_BLK_NOCL1A_UID_PPC_TPU_D0_CYCLE_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1A_UID_PPC_TPU_D0_CYCLE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_aur_cd[] = {
	CLK_BLK_NOCL1A_UID_LH_AXI_SI_P_AUR_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_aur_d0_event[] = {
	CLK_BLK_NOCL1A_UID_PPC_AUR_D0_EVENT_IPCLKPORT_PCLK,
	CLK_BLK_NOCL1A_UID_PPC_AUR_D0_EVENT_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppc_aur_d1_event[] = {
	CLK_BLK_NOCL1A_UID_PPC_AUR_D1_EVENT_IPCLKPORT_PCLK,
	CLK_BLK_NOCL1A_UID_PPC_AUR_D1_EVENT_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppc_aur_d0_cycle[] = {
	CLK_BLK_NOCL1A_UID_PPC_AUR_D0_CYCLE_IPCLKPORT_PCLK,
	CLK_BLK_NOCL1A_UID_PPC_AUR_D0_CYCLE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl1a_cd[] = {
	CLK_BLK_NOCL1A_UID_LH_AST_MI_G_NOCL1A_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl1a[] = {
	CLK_BLK_NOCL1A_UID_LH_AST_SI_G_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_aur_cd[] = {
	CLK_BLK_NOCL1A_UID_LH_AXI_MI_P_AUR_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_aur[] = {
	CLK_BLK_NOCL1A_UID_SLH_AXI_SI_P_AUR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_g3d_cd[] = {
	CLK_BLK_NOCL1A_UID_LH_AXI_MI_P_G3D_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_g3d[] = {
	CLK_BLK_NOCL1A_UID_SLH_AXI_SI_P_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_tpu_cd[] = {
	CLK_BLK_NOCL1A_UID_LH_AXI_SI_P_TPU_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_tpu_cd[] = {
	CLK_BLK_NOCL1A_UID_LH_AXI_MI_P_TPU_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_tpu[] = {
	CLK_BLK_NOCL1A_UID_SLH_AXI_SI_P_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d0_aur[] = {
	CLK_BLK_NOCL1A_UID_LH_ACEL_MI_D0_AUR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d0_tpu[] = {
	CLK_BLK_NOCL1A_UID_LH_ACEL_MI_D0_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d1_aur[] = {
	CLK_BLK_NOCL1A_UID_LH_ACEL_MI_D1_AUR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d1_tpu[] = {
	CLK_BLK_NOCL1A_UID_LH_ACEL_MI_D1_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_d_g3dmmu[] = {
	CLK_BLK_NOCL1A_UID_SLH_AXI_MI_D_G3DMMU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_d0_nocl2aa_nocl1a[] = {
	CLK_BLK_NOCL1A_UID_LH_TAXI_MI_D0_NOCL2AA_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_d0_nocl2ab_nocl1a[] = {
	CLK_BLK_NOCL1A_UID_LH_TAXI_MI_D0_NOCL2AB_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_d1_nocl2aa_nocl1a[] = {
	CLK_BLK_NOCL1A_UID_LH_TAXI_MI_D1_NOCL2AA_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_d1_nocl2ab_nocl1a[] = {
	CLK_BLK_NOCL1A_UID_LH_TAXI_MI_D1_NOCL2AB_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_d0_nocl1a_nocl0[] = {
	CLK_BLK_NOCL1A_UID_LH_TAXI_SI_D0_NOCL1A_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_d1_nocl1a_nocl0[] = {
	CLK_BLK_NOCL1A_UID_LH_TAXI_SI_D1_NOCL1A_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_d2_nocl1a_nocl0[] = {
	CLK_BLK_NOCL1A_UID_LH_TAXI_SI_D2_NOCL1A_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_d3_nocl1a_nocl0[] = {
	CLK_BLK_NOCL1A_UID_LH_TAXI_SI_D3_NOCL1A_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_nocl2ab_s0_cycle[] = {
	CLK_BLK_NOCL1A_UID_PPC_NOCL2AB_S0_CYCLE_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1A_UID_PPC_NOCL2AB_S0_CYCLE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_tpu_d1_event[] = {
	CLK_BLK_NOCL1A_UID_PPC_TPU_D1_EVENT_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1A_UID_PPC_TPU_D1_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl1a_m0[] = {
	CLK_BLK_NOCL1A_UID_PPMU_NOCL1A_M0_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1A_UID_PPMU_NOCL1A_M0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl1a_m1[] = {
	CLK_BLK_NOCL1A_UID_PPMU_NOCL1A_M1_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1A_UID_PPMU_NOCL1A_M1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl1a_m2[] = {
	CLK_BLK_NOCL1A_UID_PPMU_NOCL1A_M2_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1A_UID_PPMU_NOCL1A_M2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl1a_m3[] = {
	CLK_BLK_NOCL1A_UID_PPMU_NOCL1A_M3_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1A_UID_PPMU_NOCL1A_M3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_nocl1a_cmu_nocl1a[] = {
	CLK_BLK_NOCL1A_UID_NOCL1A_CMU_NOCL1A_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_g3dmmu_d_event[] = {
	CLK_BLK_NOCL1A_UID_PPC_G3DMMU_D_EVENT_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1A_UID_PPC_G3DMMU_D_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_bw_d_event[] = {
	CLK_BLK_NOCL1A_UID_PPC_BW_D_EVENT_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1A_UID_PPC_BW_D_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_bw_d_cycle[] = {
	CLK_BLK_NOCL1A_UID_PPC_BW_D_CYCLE_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1A_UID_PPC_BW_D_CYCLE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_p_nocl0_nocl1a[] = {
	CLK_BLK_NOCL1A_UID_LH_TAXI_MI_P_NOCL0_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d_bw[] = {
	CLK_BLK_NOCL1A_UID_LH_AXI_MI_D_BW_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_bw[] = {
	CLK_BLK_NOCL1A_UID_SLH_AXI_SI_P_BW_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_nocl1b_cmu_nocl1b[] = {
	CLK_BLK_NOCL1B_UID_NOCL1B_CMU_NOCL1B_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_d_nocl1b[] = {
	GOUT_BLK_NOCL1B_UID_TREX_D_NOCL1B_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1B_UID_TREX_D_NOCL1B_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_nocl1b[] = {
	GOUT_BLK_NOCL1B_UID_D_TZPC_NOCL1B_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d_hsi0[] = {
	GOUT_BLK_NOCL1B_UID_LH_ACEL_MI_D_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d_hsi1[] = {
	GOUT_BLK_NOCL1B_UID_LH_ACEL_MI_D_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d_aoc[] = {
	GOUT_BLK_NOCL1B_UID_LH_AXI_MI_D_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d_alive[] = {
	GOUT_BLK_NOCL1B_UID_LH_AXI_MI_D_ALIVE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d_gsa[] = {
	GOUT_BLK_NOCL1B_UID_LH_AXI_MI_D_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_aoc_cd[] = {
	GOUT_BLK_NOCL1B_UID_LH_AXI_SI_P_AOC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_gsa_cd[] = {
	GOUT_BLK_NOCL1B_UID_LH_AXI_SI_P_GSA_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_hsi0_cd[] = {
	GOUT_BLK_NOCL1B_UID_LH_AXI_SI_P_HSI0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_hsi1_cd[] = {
	GOUT_BLK_NOCL1B_UID_LH_AXI_SI_P_HSI1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_nocl1b[] = {
	GOUT_BLK_NOCL1B_UID_SYSREG_NOCL1B_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_p_nocl1b[] = {
	GOUT_BLK_NOCL1B_UID_TREX_P_NOCL1B_IPCLKPORT_ACLK_P_NOCL1B,
	GOUT_BLK_NOCL1B_UID_TREX_P_NOCL1B_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_nocl1b[] = {
	GOUT_BLK_NOCL1B_UID_GPC_NOCL1B_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_g_cssys_cu[] = {
	GOUT_BLK_NOCL1B_UID_LH_AXI_MI_G_CSSYS_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl1b_cd[] = {
	GOUT_BLK_NOCL1B_UID_LH_AST_SI_G_NOCL1B_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_aoc_event[] = {
	GOUT_BLK_NOCL1B_UID_PPC_AOC_EVENT_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1B_UID_PPC_AOC_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_aoc_cycle[] = {
	GOUT_BLK_NOCL1B_UID_PPC_AOC_CYCLE_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL1B_UID_PPC_AOC_CYCLE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl1b_cd[] = {
	CLK_BLK_NOCL1B_UID_LH_AST_MI_G_NOCL1B_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl1b[] = {
	CLK_BLK_NOCL1B_UID_LH_AST_SI_G_NOCL1B_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_aoc_cd[] = {
	CLK_BLK_NOCL1B_UID_LH_AXI_MI_P_AOC_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_aoc[] = {
	CLK_BLK_NOCL1B_UID_SLH_AXI_SI_P_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_gsa_cd[] = {
	CLK_BLK_NOCL1B_UID_LH_AXI_MI_P_GSA_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_gsa[] = {
	CLK_BLK_NOCL1B_UID_SLH_AXI_SI_P_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_hsi0_cd[] = {
	CLK_BLK_NOCL1B_UID_LH_AXI_MI_P_HSI0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_hsi0[] = {
	CLK_BLK_NOCL1B_UID_SLH_AXI_SI_P_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_hsi1_cd[] = {
	CLK_BLK_NOCL1B_UID_LH_AXI_MI_P_HSI1_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_hsi1[] = {
	CLK_BLK_NOCL1B_UID_SLH_AXI_SI_P_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_g_cssys[] = {
	CLK_BLK_NOCL1B_UID_SLH_AXI_MI_G_CSSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_g_cssys_cu[] = {
	CLK_BLK_NOCL1B_UID_LH_AXI_SI_G_CSSYS_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl1b_m0[] = {
	CLK_BLK_NOCL1B_UID_PPMU_NOCL1B_M0_IPCLKPORT_ACLK,
	CLK_BLK_NOCL1B_UID_PPMU_NOCL1B_M0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_d_nocl1b_nocl0[] = {
	CLK_BLK_NOCL1B_UID_LH_TAXI_SI_D_NOCL1B_NOCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_p_nocl0_nocl1b[] = {
	CLK_BLK_NOCL1B_UID_LH_TAXI_MI_P_NOCL0_NOCL1B_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_blk_nocl1b_frc_otp_deserial[] = {
	CLK_BLK_NOCL1B_UID_BLK_NOCL1B_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_nocl2aa_cmu_nocl2aa[] = {
	CLK_BLK_NOCL2AA_UID_NOCL2AA_CMU_NOCL2AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_nocl2aa[] = {
	GOUT_BLK_NOCL2AA_UID_SYSREG_NOCL2AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d_hsi2[] = {
	GOUT_BLK_NOCL2AA_UID_LH_ACEL_MI_D_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d1_ispfe[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D1_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d1_dpuf0[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D1_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d0_dpuf0[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D0_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d0_mfc[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D0_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d1_mfc[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D1_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_hsi2_cd[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_SI_P_HSI2_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d0_ispfe[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D0_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d3_ispfe[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D3_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_nocl2aa[] = {
	GOUT_BLK_NOCL2AA_UID_D_TZPC_NOCL2AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_d_nocl2aa[] = {
	GOUT_BLK_NOCL2AA_UID_TREX_D_NOCL2AA_IPCLKPORT_ACLK,
	GOUT_BLK_NOCL2AA_UID_TREX_D_NOCL2AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_nocl2aa[] = {
	GOUT_BLK_NOCL2AA_UID_GPC_NOCL2AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d6_rgbp[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D6_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d5_rgbp[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D5_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d0_rgbp[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D0_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d1_rgbp[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D1_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_trex_p_nocl2aa[] = {
	GOUT_BLK_NOCL2AA_UID_TREX_P_NOCL2AA_IPCLKPORT_PCLK,
	GOUT_BLK_NOCL2AA_UID_TREX_P_NOCL2AA_IPCLKPORT_ACLK_P_NOCL2AA,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d2_ispfe[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D2_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d2_rgbp[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D2_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d3_rgbp[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D3_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl2aa_cd[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AST_SI_G_NOCL2AA_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d4_rgbp[] = {
	GOUT_BLK_NOCL2AA_UID_LH_AXI_MI_D4_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl2aa_cd[] = {
	CLK_BLK_NOCL2AA_UID_LH_AST_MI_G_NOCL2AA_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl2aa[] = {
	CLK_BLK_NOCL2AA_UID_LH_AST_SI_G_NOCL2AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_hsi2_cd[] = {
	CLK_BLK_NOCL2AA_UID_LH_AXI_MI_P_HSI2_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_hsi2[] = {
	CLK_BLK_NOCL2AA_UID_SLH_AXI_SI_P_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_dpuf0[] = {
	CLK_BLK_NOCL2AA_UID_SLH_AXI_SI_P_DPUF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_ispfe[] = {
	CLK_BLK_NOCL2AA_UID_SLH_AXI_SI_P_ISPFE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_rgbp[] = {
	CLK_BLK_NOCL2AA_UID_SLH_AXI_SI_P_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_mfc[] = {
	CLK_BLK_NOCL2AA_UID_SLH_AXI_SI_P_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_dpuf1[] = {
	CLK_BLK_NOCL2AA_UID_SLH_AXI_SI_P_DPUF1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_dpub[] = {
	CLK_BLK_NOCL2AA_UID_SLH_AXI_SI_P_DPUB_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl2aa_m0[] = {
	CLK_BLK_NOCL2AA_UID_PPMU_NOCL2AA_M0_IPCLKPORT_ACLK,
	CLK_BLK_NOCL2AA_UID_PPMU_NOCL2AA_M0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl2aa_m1[] = {
	CLK_BLK_NOCL2AA_UID_PPMU_NOCL2AA_M1_IPCLKPORT_ACLK,
	CLK_BLK_NOCL2AA_UID_PPMU_NOCL2AA_M1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_p_nocl0_nocl2aa[] = {
	CLK_BLK_NOCL2AA_UID_LH_TAXI_MI_P_NOCL0_NOCL2AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_d0_nocl2aa_nocl1a[] = {
	CLK_BLK_NOCL2AA_UID_LH_TAXI_SI_D0_NOCL2AA_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_d1_nocl2aa_nocl1a[] = {
	CLK_BLK_NOCL2AA_UID_LH_TAXI_SI_D1_NOCL2AA_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_blk_nocl2aa_frc_otp_deserial[] = {
	CLK_BLK_NOCL2AA_UID_BLK_NOCL2AA_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_nocl2ab_cmu_nocl2ab[] = {
	CLK_BLK_NOCL2AB_UID_NOCL2AB_CMU_NOCL2AB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_nocl2ab[] = {
	CLK_BLK_NOCL2AB_UID_D_TZPC_NOCL2AB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_nocl2ab[] = {
	CLK_BLK_NOCL2AB_UID_GPC_NOCL2AB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_nocl2ab[] = {
	CLK_BLK_NOCL2AB_UID_SYSREG_NOCL2AB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_d_nocl2ab[] = {
	CLK_BLK_NOCL2AB_UID_TREX_D_NOCL2AB_IPCLKPORT_ACLK,
	CLK_BLK_NOCL2AB_UID_TREX_D_NOCL2AB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_p_nocl2ab[] = {
	CLK_BLK_NOCL2AB_UID_TREX_P_NOCL2AB_IPCLKPORT_ACLK_P_NOCL2AB,
	CLK_BLK_NOCL2AB_UID_TREX_P_NOCL2AB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl2ab_m0[] = {
	CLK_BLK_NOCL2AB_UID_PPMU_NOCL2AB_M0_IPCLKPORT_ACLK,
	CLK_BLK_NOCL2AB_UID_PPMU_NOCL2AB_M0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_nocl2ab_m1[] = {
	CLK_BLK_NOCL2AB_UID_PPMU_NOCL2AB_M1_IPCLKPORT_ACLK,
	CLK_BLK_NOCL2AB_UID_PPMU_NOCL2AB_M1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d0_gdc[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D0_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d1_gdc[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D1_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d_gse[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D_GSE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d0_g2d[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D0_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d1_g2d[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D1_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d0_mcsc[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D0_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d1_mcsc[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D1_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d0_tnr[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D0_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d1_tnr[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D1_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d2_tnr[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D2_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d3_tnr[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D3_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d_yuvp[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D_YUVP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_mi_p_nocl0_nocl2ab[] = {
	CLK_BLK_NOCL2AB_UID_LH_TAXI_MI_P_NOCL0_NOCL2AB_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_d0_nocl2ab_nocl1a[] = {
	CLK_BLK_NOCL2AB_UID_LH_TAXI_SI_D0_NOCL2AB_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_taxi_si_d1_nocl2ab_nocl1a[] = {
	CLK_BLK_NOCL2AB_UID_LH_TAXI_SI_D1_NOCL2AB_NOCL1A_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl2ab_cd[] = {
	CLK_BLK_NOCL2AB_UID_LH_AST_SI_G_NOCL2AB_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_g_nocl2ab_cd[] = {
	CLK_BLK_NOCL2AB_UID_LH_AST_MI_G_NOCL2AB_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_g_nocl2ab[] = {
	CLK_BLK_NOCL2AB_UID_LH_AST_SI_G_NOCL2AB_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_gdc[] = {
	CLK_BLK_NOCL2AB_UID_SLH_AXI_SI_P_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_gse[] = {
	CLK_BLK_NOCL2AB_UID_SLH_AXI_SI_P_GSE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_mcsc[] = {
	CLK_BLK_NOCL2AB_UID_SLH_AXI_SI_P_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_g2d[] = {
	CLK_BLK_NOCL2AB_UID_SLH_AXI_SI_P_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_tnr[] = {
	CLK_BLK_NOCL2AB_UID_SLH_AXI_SI_P_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_si_p_yuvp[] = {
	CLK_BLK_NOCL2AB_UID_SLH_AXI_SI_P_YUVP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d_misc[] = {
	CLK_BLK_NOCL2AB_UID_LH_ACEL_MI_D_MISC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d5_tnr[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D5_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d4_tnr[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D4_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_mi_d2_g2d[] = {
	CLK_BLK_NOCL2AB_UID_LH_ACEL_MI_D2_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_d2_gdc[] = {
	CLK_BLK_NOCL2AB_UID_LH_AXI_MI_D2_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_blk_nocl2ab_frc_otp_deserial[] = {
	CLK_BLK_NOCL2AB_UID_BLK_NOCL2AB_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpio_peric0[] = {
	GOUT_BLK_PERIC0_UID_GPIO_PERIC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_peric0[] = {
	GOUT_BLK_PERIC0_UID_SYSREG_PERIC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_peric0_cmu_peric0[] = {
	CLK_BLK_PERIC0_UID_PERIC0_CMU_PERIC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_peric0_cu[] = {
	GOUT_BLK_PERIC0_UID_LH_AXI_MI_P_PERIC0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_peric0[] = {
	GOUT_BLK_PERIC0_UID_D_TZPC_PERIC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_peric0[] = {
	GOUT_BLK_PERIC0_UID_GPC_PERIC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi1_usi[] = {
	CLK_BLK_PERIC0_UID_USI1_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC0_UID_USI1_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi2_usi[] = {
	CLK_BLK_PERIC0_UID_USI2_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC0_UID_USI2_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi3_usi[] = {
	CLK_BLK_PERIC0_UID_USI3_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC0_UID_USI3_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi4_usi[] = {
	CLK_BLK_PERIC0_UID_USI4_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC0_UID_USI4_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi5_usi[] = {
	CLK_BLK_PERIC0_UID_USI5_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC0_UID_USI5_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi6_usi[] = {
	CLK_BLK_PERIC0_UID_USI6_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC0_UID_USI6_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_i3c1[] = {
	CLK_BLK_PERIC0_UID_I3C1_IPCLKPORT_I_SCLK,
	CLK_BLK_PERIC0_UID_I3C1_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_i3c2[] = {
	CLK_BLK_PERIC0_UID_I3C2_IPCLKPORT_I_SCLK,
	CLK_BLK_PERIC0_UID_I3C2_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_i3c3[] = {
	CLK_BLK_PERIC0_UID_I3C3_IPCLKPORT_I_SCLK,
	CLK_BLK_PERIC0_UID_I3C3_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_i3c4[] = {
	CLK_BLK_PERIC0_UID_I3C4_IPCLKPORT_I_SCLK,
	CLK_BLK_PERIC0_UID_I3C4_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_i3c5[] = {
	CLK_BLK_PERIC0_UID_I3C5_IPCLKPORT_I_SCLK,
	CLK_BLK_PERIC0_UID_I3C5_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_i3c6[] = {
	CLK_BLK_PERIC0_UID_I3C6_IPCLKPORT_I_SCLK,
	CLK_BLK_PERIC0_UID_I3C6_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_usi0_uart[] = {
	CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi14_usi[] = {
	CLK_BLK_PERIC0_UID_USI14_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC0_UID_USI14_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_peric0[] = {
	CLK_BLK_PERIC0_UID_SLH_AXI_MI_P_PERIC0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_peric0_cu[] = {
	CLK_BLK_PERIC0_UID_LH_AXI_SI_P_PERIC0_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpio_peric1[] = {
	GOUT_BLK_PERIC1_UID_GPIO_PERIC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_peric1[] = {
	GOUT_BLK_PERIC1_UID_SYSREG_PERIC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_peric1_cmu_peric1[] = {
	CLK_BLK_PERIC1_UID_PERIC1_CMU_PERIC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_peric1_cu[] = {
	GOUT_BLK_PERIC1_UID_LH_AXI_MI_P_PERIC1_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_peric1[] = {
	GOUT_BLK_PERIC1_UID_D_TZPC_PERIC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_peric1[] = {
	GOUT_BLK_PERIC1_UID_GPC_PERIC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi0_usi[] = {
	CLK_BLK_PERIC1_UID_USI0_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC1_UID_USI0_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi9_usi[] = {
	CLK_BLK_PERIC1_UID_USI9_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC1_UID_USI9_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi10_usi[] = {
	CLK_BLK_PERIC1_UID_USI10_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC1_UID_USI10_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi11_usi[] = {
	CLK_BLK_PERIC1_UID_USI11_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC1_UID_USI11_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi12_usi[] = {
	CLK_BLK_PERIC1_UID_USI12_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC1_UID_USI12_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usi13_usi[] = {
	CLK_BLK_PERIC1_UID_USI13_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC1_UID_USI13_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_i3c0[] = {
	CLK_BLK_PERIC1_UID_I3C0_IPCLKPORT_I_SCLK,
	CLK_BLK_PERIC1_UID_I3C0_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_pwm[] = {
	CLK_BLK_PERIC1_UID_PWM_IPCLKPORT_I_PCLK_S0,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_peric1[] = {
	CLK_BLK_PERIC1_UID_SLH_AXI_MI_P_PERIC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_peric1_cu[] = {
	CLK_BLK_PERIC1_UID_LH_AXI_SI_P_PERIC1_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_usi15_usi[] = {
	CLK_BLK_PERIC1_UID_USI15_USI_IPCLKPORT_IPCLK,
	CLK_BLK_PERIC1_UID_USI15_USI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_rgbp_cmu_rgbp[] = {
	CLK_BLK_RGBP_UID_RGBP_CMU_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_rgbp[] = {
	GOUT_BLK_RGBP_UID_AD_APB_RGBP_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_rgbp[] = {
	GOUT_BLK_RGBP_UID_D_TZPC_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_rgbp[] = {
	GOUT_BLK_RGBP_UID_GPC_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_rgbp[] = {
	GOUT_BLK_RGBP_UID_RGBP_IPCLKPORT_CLK,
	CLK_BLK_RGBP_UID_RGBP_IPCLKPORT_CLK_VOTF0,
	CLK_BLK_RGBP_UID_RGBP_IPCLKPORT_CLK_VOTF0,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_rgbp[] = {
	GOUT_BLK_RGBP_UID_SLH_AXI_MI_P_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_rgbp[] = {
	GOUT_BLK_RGBP_UID_SYSREG_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d0_rgbp[] = {
	GOUT_BLK_RGBP_UID_LH_AXI_SI_D0_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d1_rgbp[] = {
	GOUT_BLK_RGBP_UID_LH_AXI_SI_D1_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_rgbp[] = {
	CLK_BLK_RGBP_UID_SSMT_D0_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_SSMT_D0_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d0_rgbp[] = {
	CLK_BLK_RGBP_UID_QE_D0_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D0_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_rgbp[] = {
	CLK_BLK_RGBP_UID_PPMU_D0_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_PPMU_D0_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_mcfp[] = {
	CLK_BLK_RGBP_UID_AD_APB_MCFP_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_rgbp[] = {
	CLK_BLK_RGBP_UID_PPMU_D1_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_PPMU_D1_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_rgbp[] = {
	CLK_BLK_RGBP_UID_PPMU_D2_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_PPMU_D2_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_mcfp[] = {
	CLK_BLK_RGBP_UID_PPMU_D0_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_PPMU_D0_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_mcfp[] = {
	CLK_BLK_RGBP_UID_PPMU_D2_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_PPMU_D2_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d3_mcfp[] = {
	CLK_BLK_RGBP_UID_PPMU_D3_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_PPMU_D3_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d4_mcfp[] = {
	CLK_BLK_RGBP_UID_PPMU_D4_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_PPMU_D4_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d5_mcfp[] = {
	CLK_BLK_RGBP_UID_PPMU_D5_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_PPMU_D5_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_rgbp[] = {
	CLK_BLK_RGBP_UID_SYSMMU_S0_RGBP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_rgbp[] = {
	CLK_BLK_RGBP_UID_SYSMMU_S0_PMMU0_RGBP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_rgbp[] = {
	CLK_BLK_RGBP_UID_SYSMMU_S0_PMMU1_RGBP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_rgbp[] = {
	CLK_BLK_RGBP_UID_SYSMMU_S1_RGBP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_pmmu0_rgbp[] = {
	CLK_BLK_RGBP_UID_SYSMMU_S1_PMMU0_RGBP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_pmmu1_rgbp[] = {
	CLK_BLK_RGBP_UID_SYSMMU_S1_PMMU1_RGBP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_pmmu2_rgbp[] = {
	CLK_BLK_RGBP_UID_SYSMMU_S1_PMMU2_RGBP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_pmmu3_rgbp[] = {
	CLK_BLK_RGBP_UID_SYSMMU_S1_PMMU3_RGBP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_pmmu4_rgbp[] = {
	CLK_BLK_RGBP_UID_SYSMMU_S1_PMMU4_RGBP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d2_rgbp[] = {
	CLK_BLK_RGBP_UID_LH_AXI_SI_D2_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d3_rgbp[] = {
	CLK_BLK_RGBP_UID_LH_AXI_SI_D3_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d4_rgbp[] = {
	CLK_BLK_RGBP_UID_LH_AXI_SI_D4_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d5_rgbp[] = {
	CLK_BLK_RGBP_UID_LH_AXI_SI_D5_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d6_rgbp[] = {
	CLK_BLK_RGBP_UID_LH_AXI_SI_D6_RGBP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_ld_rgbp_gdc[] = {
	CLK_BLK_RGBP_UID_LH_AXI_SI_LD_RGBP_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mcfp[] = {
	CLK_BLK_RGBP_UID_MCFP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_rgbp[] = {
	CLK_BLK_RGBP_UID_SSMT_D1_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_SSMT_D1_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d2_rgbp[] = {
	CLK_BLK_RGBP_UID_SSMT_D2_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_SSMT_D2_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_mcfp[] = {
	CLK_BLK_RGBP_UID_SSMT_D0_MCFP_IPCLKPORT_PCLK,
	CLK_BLK_RGBP_UID_SSMT_D0_MCFP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d2_mcfp[] = {
	CLK_BLK_RGBP_UID_SSMT_D2_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_SSMT_D2_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d3_mcfp[] = {
	CLK_BLK_RGBP_UID_SSMT_D3_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_SSMT_D3_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d4_mcfp[] = {
	CLK_BLK_RGBP_UID_SSMT_D4_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_SSMT_D4_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d5_mcfp[] = {
	CLK_BLK_RGBP_UID_SSMT_D5_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_SSMT_D5_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d1_rgbp[] = {
	CLK_BLK_RGBP_UID_QE_D1_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D1_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d2_rgbp[] = {
	CLK_BLK_RGBP_UID_QE_D2_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D2_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d3_rgbp[] = {
	CLK_BLK_RGBP_UID_QE_D3_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D3_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d4_rgbp[] = {
	CLK_BLK_RGBP_UID_QE_D4_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D4_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d5_rgbp[] = {
	CLK_BLK_RGBP_UID_QE_D5_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D5_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d6_rgbp[] = {
	CLK_BLK_RGBP_UID_QE_D6_RGBP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D6_RGBP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d4_mcfp[] = {
	CLK_BLK_RGBP_UID_QE_D4_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D4_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d5_mcfp[] = {
	CLK_BLK_RGBP_UID_QE_D5_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D5_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d6_mcfp[] = {
	CLK_BLK_RGBP_UID_QE_D6_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D6_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d7_mcfp[] = {
	CLK_BLK_RGBP_UID_QE_D7_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D7_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d8_mcfp[] = {
	CLK_BLK_RGBP_UID_QE_D8_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D8_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d9_mcfp[] = {
	CLK_BLK_RGBP_UID_QE_D9_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D9_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d11_mcfp[] = {
	CLK_BLK_RGBP_UID_QE_D11_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D11_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_i_rgbp_mcfp[] = {
	CLK_BLK_RGBP_UID_LH_AST_SI_I_RGBP_MCFP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_i_rgbp_mcfp[] = {
	CLK_BLK_RGBP_UID_LH_AST_MI_I_RGBP_MCFP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_otf_rgbp_yuvp[] = {
	CLK_BLK_RGBP_UID_LH_AST_SI_L_OTF_RGBP_YUVP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_qe_d10_mcfp[] = {
	CLK_BLK_RGBP_UID_QE_D10_MCFP_IPCLKPORT_ACLK,
	CLK_BLK_RGBP_UID_QE_D10_MCFP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_blk_rgbp_frc_otp_deserial[] = {
	CLK_BLK_RGBP_UID_BLK_RGBP_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_rgbp[] = {
	CLK_BLK_RGBP_UID_XIU_D1_RGBP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d4_rgbp[] = {
	CLK_BLK_RGBP_UID_XIU_D4_RGBP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d5_rgbp[] = {
	CLK_BLK_RGBP_UID_XIU_D5_RGBP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_s2d_cmu_s2d[] = {
	CLK_BLK_S2D_UID_S2D_CMU_S2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_bis_s2d[] = {
	GOUT_BLK_S2D_UID_BIS_S2D_IPCLKPORT_SCLK,
	GOUT_BLK_S2D_UID_BIS_S2D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_lg_scan2dram_cu[] = {
	GOUT_BLK_S2D_UID_LH_AXI_MI_LG_SCAN2DRAM_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_lg_scan2dram[] = {
	CLK_BLK_S2D_UID_SLH_AXI_MI_LG_SCAN2DRAM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_lg_scan2dram_cu[] = {
	CLK_BLK_S2D_UID_LH_AXI_SI_LG_SCAN2DRAM_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_gtnr_merge[] = {
	GOUT_BLK_TNR_UID_AD_APB_GTNR_MERGE_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_tnr[] = {
	GOUT_BLK_TNR_UID_D_TZPC_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_tnr[] = {
	GOUT_BLK_TNR_UID_SLH_AXI_MI_P_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_otf_tnr_mcsc[] = {
	GOUT_BLK_TNR_UID_LH_AST_SI_L_OTF_TNR_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d0_tnr[] = {
	GOUT_BLK_TNR_UID_LH_AXI_SI_D0_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d1_tnr[] = {
	GOUT_BLK_TNR_UID_LH_AXI_SI_D1_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D0_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D0_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D1_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D1_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_tnr[] = {
	GOUT_BLK_TNR_UID_SYSMMU_S0_PMMU0_TNR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_tnr[] = {
	GOUT_BLK_TNR_UID_SYSREG_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_otf_yuvp_tnr[] = {
	GOUT_BLK_TNR_UID_LH_AST_MI_L_OTF_YUVP_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d2_tnr[] = {
	GOUT_BLK_TNR_UID_LH_AXI_SI_D2_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d3_tnr[] = {
	GOUT_BLK_TNR_UID_LH_AXI_SI_D3_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D2_TNR_IPCLKPORT_PCLK,
	GOUT_BLK_TNR_UID_PPMU_D2_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d3_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D3_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D3_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_pmmu2_tnr[] = {
	GOUT_BLK_TNR_UID_SYSMMU_S1_PMMU2_TNR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_pmmu1_tnr[] = {
	GOUT_BLK_TNR_UID_SYSMMU_S1_PMMU1_TNR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_qe_d10_tnra[] = {
	GOUT_BLK_TNR_UID_QE_D10_TNRA_IPCLKPORT_PCLK,
	CLK_BLK_TNR_UID_QE_D10_TNRA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_tnr[] = {
	GOUT_BLK_TNR_UID_XIU_D0_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_tnr[] = {
	GOUT_BLK_TNR_UID_XIU_D1_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d0_tnr[] = {
	GOUT_BLK_TNR_UID_QE_D0_TNR_IPCLKPORT_PCLK,
	GOUT_BLK_TNR_UID_QE_D0_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d1_tnr[] = {
	GOUT_BLK_TNR_UID_QE_D1_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_QE_D1_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d5_tnr[] = {
	GOUT_BLK_TNR_UID_QE_D5_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_QE_D5_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d6_tnr[] = {
	GOUT_BLK_TNR_UID_QE_D6_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_QE_D6_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d7_tnr[] = {
	GOUT_BLK_TNR_UID_QE_D7_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_QE_D7_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_tnr[] = {
	GOUT_BLK_TNR_UID_SSMT_D0_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_SSMT_D0_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_tnr[] = {
	GOUT_BLK_TNR_UID_SSMT_D1_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_SSMT_D1_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d2_tnr[] = {
	GOUT_BLK_TNR_UID_SSMT_D2_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_SSMT_D2_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d3_tnr[] = {
	GOUT_BLK_TNR_UID_SSMT_D3_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_SSMT_D3_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_pmmu0_tnr[] = {
	GOUT_BLK_TNR_UID_SYSMMU_S1_PMMU0_TNR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_tnr[] = {
	GOUT_BLK_TNR_UID_GPC_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_otf_tnr_gse[] = {
	GOUT_BLK_TNR_UID_LH_AST_SI_L_OTF_TNR_GSE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_qe_d8_tnr[] = {
	CLK_BLK_TNR_UID_QE_D8_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_QE_D8_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d9_tnr[] = {
	CLK_BLK_TNR_UID_QE_D9_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_QE_D9_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_tnr[] = {
	CLK_BLK_TNR_UID_SYSMMU_S0_PMMU1_TNR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d4_tnr[] = {
	CLK_BLK_TNR_UID_XIU_D4_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d11_tnra[] = {
	CLK_BLK_TNR_UID_QE_D11_TNRA_IPCLKPORT_PCLK,
	CLK_BLK_TNR_UID_QE_D11_TNRA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d2_tnr[] = {
	CLK_BLK_TNR_UID_XIU_D2_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d3_tnr[] = {
	CLK_BLK_TNR_UID_XIU_D3_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_gtnr_merge[] = {
	CLK_BLK_TNR_UID_GTNR_MERGE_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_GTNR_MERGE_IPCLKPORT_C2CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d4_tnr[] = {
	CLK_BLK_TNR_UID_LH_AXI_SI_D4_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d10_tnra[] = {
	CLK_BLK_TNR_UID_PPMU_D10_TNRA_IPCLKPORT_PCLK,
	CLK_BLK_TNR_UID_PPMU_D10_TNRA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d4_tnr[] = {
	CLK_BLK_TNR_UID_PPMU_D4_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_PPMU_D4_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d5_tnr[] = {
	CLK_BLK_TNR_UID_PPMU_D5_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_PPMU_D5_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d6_tnr[] = {
	CLK_BLK_TNR_UID_PPMU_D6_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_PPMU_D6_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d7_tnr[] = {
	CLK_BLK_TNR_UID_PPMU_D7_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_PPMU_D7_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d8_tnr[] = {
	CLK_BLK_TNR_UID_PPMU_D8_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_PPMU_D8_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d9_tnr[] = {
	CLK_BLK_TNR_UID_PPMU_D9_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_PPMU_D9_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d10_tnra[] = {
	CLK_BLK_TNR_UID_SSMT_D10_TNRA_IPCLKPORT_PCLK,
	CLK_BLK_TNR_UID_SSMT_D10_TNRA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d4_tnr[] = {
	CLK_BLK_TNR_UID_SSMT_D4_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_SSMT_D4_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d5_tnr[] = {
	CLK_BLK_TNR_UID_SSMT_D5_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_SSMT_D5_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d6_tnr[] = {
	CLK_BLK_TNR_UID_SSMT_D6_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_SSMT_D6_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d7_tnr[] = {
	CLK_BLK_TNR_UID_SSMT_D7_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_SSMT_D7_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d8_tnr[] = {
	CLK_BLK_TNR_UID_SSMT_D8_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_SSMT_D8_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d9_tnr[] = {
	CLK_BLK_TNR_UID_SSMT_D9_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_SSMT_D9_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_tnr[] = {
	CLK_BLK_TNR_UID_SYSMMU_S0_TNR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s1_tnr[] = {
	CLK_BLK_TNR_UID_SYSMMU_S1_TNR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d6_tnr[] = {
	CLK_BLK_TNR_UID_XIU_D6_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_tnr_cmu_tnr[] = {
	CLK_BLK_TNR_UID_TNR_CMU_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d11_tnra[] = {
	CLK_BLK_TNR_UID_SSMT_D11_TNRA_IPCLKPORT_PCLK,
	CLK_BLK_TNR_UID_SSMT_D11_TNRA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d11_tnra[] = {
	CLK_BLK_TNR_UID_PPMU_D11_TNRA_IPCLKPORT_PCLK,
	CLK_BLK_TNR_UID_PPMU_D11_TNRA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_gtnr_align[] = {
	CLK_BLK_TNR_UID_AD_APB_GTNR_ALIGN_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_gtnr_align[] = {
	CLK_BLK_TNR_UID_GTNR_ALIGN_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d10_tnr[] = {
	CLK_BLK_TNR_UID_XIU_D10_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d2_tnr[] = {
	CLK_BLK_TNR_UID_QE_D2_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_QE_D2_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d3_tnr[] = {
	CLK_BLK_TNR_UID_QE_D3_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_QE_D3_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d4_tnr[] = {
	CLK_BLK_TNR_UID_QE_D4_TNR_IPCLKPORT_ACLK,
	CLK_BLK_TNR_UID_QE_D4_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s2_pmmu0_tnr[] = {
	CLK_BLK_TNR_UID_SYSMMU_S2_PMMU0_TNR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s2_tnr[] = {
	CLK_BLK_TNR_UID_SYSMMU_S2_TNR_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d7_tnr[] = {
	CLK_BLK_TNR_UID_XIU_D7_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d11_tnr[] = {
	CLK_BLK_TNR_UID_XIU_D11_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d5_tnr[] = {
	CLK_BLK_TNR_UID_LH_AXI_SI_D5_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_blk_tnr_frc_otp_deserial[] = {
	CLK_BLK_TNR_UID_BLK_TNR_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_tpu_cmu_tpu[] = {
	CLK_BLK_TPU_UID_TPU_CMU_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_mi_p_tpu_cu[] = {
	GOUT_BLK_TPU_UID_LH_AXI_MI_P_TPU_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_tpu[] = {
	GOUT_BLK_TPU_UID_D_TZPC_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d0_tpu[] = {
	GOUT_BLK_TPU_UID_LH_ACEL_SI_D0_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_tpu[] = {
	GOUT_BLK_TPU_UID_SYSREG_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_tpu[] = {
	GOUT_BLK_TPU_UID_SYSMMU_S0_PMMU0_TPU_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_tpu[] = {
	GOUT_BLK_TPU_UID_PPMU_D0_TPU_IPCLKPORT_ACLK,
	GOUT_BLK_TPU_UID_PPMU_D0_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_tpu[] = {
	GOUT_BLK_TPU_UID_SSMT_D0_TPU_IPCLKPORT_PCLK,
	GOUT_BLK_TPU_UID_SSMT_D0_TPU_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_gpc_tpu[] = {
	GOUT_BLK_TPU_UID_GPC_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_as_apb_sysmmu_s1_ns_tpu[] = {
	GOUT_BLK_TPU_UID_AS_APB_SYSMMU_S1_NS_TPU_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_tpu[] = {
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_TPU_CLK,
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_APB_PCLK,
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_AXI_CLK,
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_TPU_CTL_CLK,
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_DBG_UART_SCLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt0_tpu_cpucl0[] = {
	GOUT_BLK_TPU_UID_LH_ATB_SI_LT0_TPU_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt1_tpu_cpucl0[] = {
	GOUT_BLK_TPU_UID_LH_ATB_SI_LT1_TPU_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_adm_dap_g_tpu[] = {
	GOUT_BLK_TPU_UID_ADM_DAP_G_TPU_IPCLKPORT_DAPCLKM,
};
enum clk_id cmucal_vclk_ip_async_apb_int_tpu[] = {
	GOUT_BLK_TPU_UID_ASYNC_APB_INT_TPU_IPCLKPORT_PCLKS,
	GOUT_BLK_TPU_UID_ASYNC_APB_INT_TPU_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt0_tpu_cpucl0_cd[] = {
	GOUT_BLK_TPU_UID_LH_ATB_MI_LT0_TPU_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_mi_lt1_tpu_cpucl0_cd[] = {
	GOUT_BLK_TPU_UID_LH_ATB_MI_LT1_TPU_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt0_tpu_cpucl0_cd[] = {
	GOUT_BLK_TPU_UID_LH_ATB_SI_LT0_TPU_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_atb_si_lt1_tpu_cpucl0_cd[] = {
	GOUT_BLK_TPU_UID_LH_ATB_SI_LT1_TPU_CPUCL0_CD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_tpu[] = {
	CLK_BLK_TPU_UID_SLH_AXI_MI_P_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_p_tpu_cu[] = {
	CLK_BLK_TPU_UID_LH_AXI_SI_P_TPU_CU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_tpu[] = {
	CLK_BLK_TPU_UID_XIU_D_TPU_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_tpu[] = {
	CLK_BLK_TPU_UID_SSMT_D1_TPU_IPCLKPORT_ACLK,
	CLK_BLK_TPU_UID_SSMT_D1_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_tpu[] = {
	CLK_BLK_TPU_UID_PPMU_D1_TPU_IPCLKPORT_ACLK,
	CLK_BLK_TPU_UID_PPMU_D1_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu1_tpu[] = {
	CLK_BLK_TPU_UID_SYSMMU_S0_PMMU1_TPU_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_tpu[] = {
	CLK_BLK_TPU_UID_SYSMMU_S0_TPU_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lh_acel_si_d1_tpu[] = {
	CLK_BLK_TPU_UID_LH_ACEL_SI_D1_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_add_apbif_tpu[] = {
	CLK_BLK_TPU_UID_ADD_APBIF_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_add_tpu[] = {
	CLK_BLK_TPU_UID_ADD_TPU_IPCLKPORT_CH_CLK,
};
enum clk_id cmucal_vclk_ip_dapapbap_tpu[] = {
	CLK_BLK_TPU_UID_DAPAPBAP_TPU_IPCLKPORT_DAPCLK,
};
enum clk_id cmucal_vclk_ip_blk_tpu_frc_otp_deserial[] = {
	CLK_BLK_TPU_UID_BLK_TPU_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_yuvp[] = {
	GOUT_BLK_YUVP_UID_AD_APB_YUVP_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_yuvp[] = {
	GOUT_BLK_YUVP_UID_D_TZPC_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_yuvp[] = {
	GOUT_BLK_YUVP_UID_YUVP_IPCLKPORT_CLK,
	CLK_BLK_YUVP_UID_YUVP_IPCLKPORT_CLK_VOTF0,
	CLK_BLK_YUVP_UID_YUVP_IPCLKPORT_CLK_VOTF0,
};
enum clk_id cmucal_vclk_ip_gpc_yuvp[] = {
	GOUT_BLK_YUVP_UID_GPC_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_slh_axi_mi_p_yuvp[] = {
	GOUT_BLK_YUVP_UID_SLH_AXI_MI_P_YUVP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_axi_si_d_yuvp[] = {
	GOUT_BLK_YUVP_UID_LH_AXI_SI_D_YUVP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_yuvp[] = {
	GOUT_BLK_YUVP_UID_PPMU_D0_YUVP_IPCLKPORT_ACLK,
	GOUT_BLK_YUVP_UID_PPMU_D0_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_yuvp[] = {
	GOUT_BLK_YUVP_UID_SSMT_D0_YUVP_IPCLKPORT_ACLK,
	GOUT_BLK_YUVP_UID_SSMT_D0_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_pmmu0_yuvp[] = {
	GOUT_BLK_YUVP_UID_SYSMMU_S0_PMMU0_YUVP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_yuvp[] = {
	GOUT_BLK_YUVP_UID_SYSREG_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_otf_yuvp_mcsc[] = {
	GOUT_BLK_YUVP_UID_LH_AST_SI_L_OTF_YUVP_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_mi_l_otf_rgbp_yuvp[] = {
	GOUT_BLK_YUVP_UID_LH_AST_MI_L_OTF_RGBP_YUVP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_otf_yuvp_gse[] = {
	GOUT_BLK_YUVP_UID_LH_AST_SI_L_OTF_YUVP_GSE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lh_ast_si_l_otf_yuvp_tnr[] = {
	GOUT_BLK_YUVP_UID_LH_AST_SI_L_OTF_YUVP_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_qe_d0_yuvp[] = {
	GOUT_BLK_YUVP_UID_QE_D0_YUVP_IPCLKPORT_ACLK,
	GOUT_BLK_YUVP_UID_QE_D0_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_yuvp_cmu_yuvp[] = {
	CLK_BLK_YUVP_UID_YUVP_CMU_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_yuvp[] = {
	CLK_BLK_YUVP_UID_SSMT_D1_YUVP_IPCLKPORT_ACLK,
	CLK_BLK_YUVP_UID_SSMT_D1_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_yuvp[] = {
	CLK_BLK_YUVP_UID_PPMU_D1_YUVP_IPCLKPORT_ACLK,
	CLK_BLK_YUVP_UID_PPMU_D1_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d1_yuvp[] = {
	CLK_BLK_YUVP_UID_QE_D1_YUVP_IPCLKPORT_ACLK,
	CLK_BLK_YUVP_UID_QE_D1_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d4_yuvp[] = {
	CLK_BLK_YUVP_UID_PPMU_D4_YUVP_IPCLKPORT_ACLK,
	CLK_BLK_YUVP_UID_PPMU_D4_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d4_yuvp[] = {
	CLK_BLK_YUVP_UID_SSMT_D4_YUVP_IPCLKPORT_ACLK,
	CLK_BLK_YUVP_UID_SSMT_D4_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d4_yuvp[] = {
	CLK_BLK_YUVP_UID_QE_D4_YUVP_IPCLKPORT_ACLK,
	CLK_BLK_YUVP_UID_QE_D4_YUVP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s0_yuvp[] = {
	CLK_BLK_YUVP_UID_SYSMMU_S0_YUVP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_yuvp[] = {
	CLK_BLK_YUVP_UID_XIU_D1_YUVP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_yuvp[] = {
	CLK_BLK_YUVP_UID_XIU_D0_YUVP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_blk_yuvp_frc_otp_deserial[] = {
	CLK_BLK_YUVP_UID_BLK_YUVP_FRC_OTP_DESERIAL_IPCLKPORT_I_CLK,
};

/* DVFS VCLK -> LUT List */
struct vclk_lut cmucal_vclk_vdd_int_lut[] = {
	{2133000, vdd_int_nm_lut_params},
	{2133000, vdd_int_ud_lut_params},
	{1716000, vdd_int_sud_lut_params},
	{842000, vdd_int_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_mif_lut[] = {
	{7500000, vdd_mif_od_lut_params},
	{4266000, vdd_mif_nm_lut_params},
	{2984000, vdd_mif_ud_lut_params},
	{1716000, vdd_mif_sud_lut_params},
	{842000, vdd_mif_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_g3d_lut[] = {
	{900000, vdd_g3d_nm_lut_params},
	{700000, vdd_g3d_ud_lut_params},
	{470000, vdd_g3d_sud_lut_params},
	{150000, vdd_g3d_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_cam_lut[] = {
	{1066000, vdd_cam_nm_lut_params},
	{711000, vdd_cam_ud_lut_params},
	{355000, vdd_cam_sud_lut_params},
	{108000, vdd_cam_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_cpucl0_lut[] = {
	{2115000, vdd_cpucl0_sod_lut_params},
	{1818000, vdd_cpucl0_od_lut_params},
	{1470000, vdd_cpucl0_nm_lut_params},
	{975000, vdd_cpucl0_ud_lut_params},
	{610000, vdd_cpucl0_sud_lut_params},
	{325000, vdd_cpucl0_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_cpucl1_lut[] = {
	{2600000, vdd_cpucl1_sod_lut_params},
	{2250000, vdd_cpucl1_od_lut_params},
	{1700000, vdd_cpucl1_nm_lut_params},
	{1225000, vdd_cpucl1_ud_lut_params},
	{800000, vdd_cpucl1_sud_lut_params},
	{475000, vdd_cpucl1_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_tpu_lut[] = {
	{1120000, vdd_tpu_nm_lut_params},
	{845000, vdd_tpu_ud_lut_params},
	{625000, vdd_tpu_sud_lut_params},
	{225000, vdd_tpu_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_cpucl2_lut[] = {
	{3300000, vdd_cpucl2_sod_lut_params},
	{2700000, vdd_cpucl2_od_lut_params},
	{2150000, vdd_cpucl2_nm_lut_params},
	{1750000, vdd_cpucl2_ud_lut_params},
	{1125000, vdd_cpucl2_sud_lut_params},
	{725000, vdd_cpucl2_uud_lut_params},
};

/* SPECIAL VCLK -> LUT List */
struct vclk_lut cmucal_vclk_mux_cmu_cmuref_lut[] = {
	{533250, mux_cmu_cmuref_ud_lut_params},
	{266625, mux_cmu_cmuref_sud_lut_params},
	{133313, mux_cmu_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_cpucl0_cmuref_lut[] = {
	{200000, mux_cpucl0_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_cpucl1_cmuref_lut[] = {
	{200000, mux_cpucl1_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_cpucl2_cmuref_lut[] = {
	{200000, mux_cpucl2_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clk_hsi0_usb20_ref_lut[] = {
	{40000, mux_clk_hsi0_usb20_ref_nm_lut_params},
};
struct vclk_lut cmucal_vclk_clkcmu_hsi0_usb32drd_lut[] = {
	{40000, clkcmu_hsi0_usb32drd_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_mif_cmuref_lut[] = {
	{200000, mux_mif_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_nocl0_cmuref_lut[] = {
	{200000, mux_nocl0_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_nocl1b_cmuref_lut[] = {
	{200000, mux_nocl1b_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_nocl2aa_cmuref_lut[] = {
	{200000, mux_nocl2aa_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_nocl2ab_cmuref_lut[] = {
	{200000, mux_nocl2ab_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_clkcmu_dpub_dsim_lut[] = {
	{76179, clkcmu_dpub_dsim_uud_lut_params},
};
struct vclk_lut cmucal_vclk_clkcmu_hsi0_dpgtc_lut[] = {
	{133313, clkcmu_hsi0_dpgtc_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_apm_usi0_usi_lut[] = {
	{400000, div_clk_apm_usi0_usi_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_apm_usi0_uart_lut[] = {
	{200000, div_clk_apm_usi0_uart_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_apm_usi1_uart_lut[] = {
	{200000, div_clk_apm_usi1_uart_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_apm_i3c_pmic_lut[] = {
	{200000, div_clk_apm_i3c_pmic_nm_lut_params},
};
struct vclk_lut cmucal_vclk_clk_aur_add_ch_clk_lut[] = {
	{2167, clk_aur_add_ch_clk_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_cis_clk0_lut[] = {
	{400000, mux_clkcmu_cis_clk0_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_cis_clk1_lut[] = {
	{400000, mux_clkcmu_cis_clk1_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_cis_clk2_lut[] = {
	{400000, mux_clkcmu_cis_clk2_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_cis_clk3_lut[] = {
	{400000, mux_clkcmu_cis_clk3_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_cis_clk4_lut[] = {
	{400000, mux_clkcmu_cis_clk4_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_cis_clk5_lut[] = {
	{400000, mux_clkcmu_cis_clk5_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_cis_clk6_lut[] = {
	{400000, mux_clkcmu_cis_clk6_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_cis_clk7_lut[] = {
	{400000, mux_clkcmu_cis_clk7_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_cpucl0_cmuref_lut[] = {
	{1057500, div_clk_cpucl0_cmuref_sod_lut_params},
	{909000, div_clk_cpucl0_cmuref_od_lut_params},
	{735000, div_clk_cpucl0_cmuref_nm_lut_params},
	{487500, div_clk_cpucl0_cmuref_ud_lut_params},
	{305000, div_clk_cpucl0_cmuref_sud_lut_params},
	{162500, div_clk_cpucl0_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_cpucl0_add_ch_clk_lut[] = {
	{2167, div_clk_cpucl0_add_ch_clk_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_cpucl1_cmuref_lut[] = {
	{1300000, div_clk_cpucl1_cmuref_sod_lut_params},
	{1125000, div_clk_cpucl1_cmuref_od_lut_params},
	{850000, div_clk_cpucl1_cmuref_nm_lut_params},
	{612500, div_clk_cpucl1_cmuref_ud_lut_params},
	{400000, div_clk_cpucl1_cmuref_sud_lut_params},
	{237500, div_clk_cpucl1_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_cpucl2_cmuref_lut[] = {
	{1650000, div_clk_cpucl2_cmuref_sod_lut_params},
	{1350000, div_clk_cpucl2_cmuref_od_lut_params},
	{1075000, div_clk_cpucl2_cmuref_nm_lut_params},
	{875000, div_clk_cpucl2_cmuref_ud_lut_params},
	{562500, div_clk_cpucl2_cmuref_sud_lut_params},
	{362500, div_clk_cpucl2_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_clk_g3d_add_ch_clk_lut[] = {
	{2167, clk_g3d_add_ch_clk_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_gsacore_spi_fps_lut[] = {
	{393500, div_clk_gsacore_spi_fps_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_gsacore_spi_gsc_lut[] = {
	{393500, div_clk_gsacore_spi_gsc_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_gsacore_uart_lut[] = {
	{196750, div_clk_gsacore_uart_nm_lut_params},
};
struct vclk_lut cmucal_vclk_clkcmu_hsi0_peri_lut[] = {
	{533250, clkcmu_hsi0_peri_uud_lut_params},
	{400000, clkcmu_hsi0_peri_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_hsi0_usix_lut[] = {
	{400000, div_clk_hsi0_usix_nm_lut_params},
	{200000, div_clk_hsi0_usix_200mhz_lut_params},
	{100000, div_clk_hsi0_usix_100mhz_lut_params},
	{80000, div_clk_hsi0_usix_80mhz_lut_params},
	{50000, div_clk_hsi0_usix_50mhz_lut_params},
	{40000, div_clk_hsi0_usix_40mhz_lut_params},
	{24000, div_clk_hsi0_usix_24mhz_lut_params},
	{12000, div_clk_hsi0_usix_12mhz_lut_params},
	{8000, div_clk_hsi0_usix_8mhz_lut_params},
	{4000, div_clk_hsi0_usix_4mhz_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_slc_dclk_lut[] = {
	{580000, div_clk_slc_dclk_od_lut_params},
	{533250, div_clk_slc_dclk_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_slc1_dclk_lut[] = {
	{580000, div_clk_slc1_dclk_od_lut_params},
	{533250, div_clk_slc1_dclk_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_slc2_dclk_lut[] = {
	{580000, div_clk_slc2_dclk_od_lut_params},
	{533250, div_clk_slc2_dclk_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_slc3_dclk_lut[] = {
	{580000, div_clk_slc3_dclk_od_lut_params},
	{533250, div_clk_slc3_dclk_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_peric0_ip_lut[] = {
	{400000, mux_clkcmu_peric0_ip_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_pericx_usix_usi_lut[] = {
	{400000, div_clk_pericx_usix_usi_nm_lut_params},
	{200000, div_clk_pericx_usix_usi_200mhz_lut_params},
	{100000, div_clk_pericx_usix_usi_100mhz_lut_params},
	{50000, div_clk_pericx_usix_usi_50mhz_lut_params},
	{40000, div_clk_pericx_usix_usi_40mhz_lut_params},
	{24000, div_clk_pericx_usix_usi_24mhz_lut_params},
	{12000, div_clk_pericx_usix_usi_12mhz_lut_params},
	{8000, div_clk_pericx_usix_usi_8mhz_lut_params},
	{4000, div_clk_pericx_usix_usi_4mhz_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_peric0_usi0_uart_lut[] = {
	{200000, div_clk_peric0_usi0_uart_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_peric1_ip_lut[] = {
	{400000, mux_clkcmu_peric1_ip_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_peric1_i3c_lut[] = {
	{200000, div_clk_peric1_i3c_uud_lut_params},
};
struct vclk_lut cmucal_vclk_clk_tpu_add_ch_clk_lut[] = {
	{2167, clk_tpu_add_ch_clk_uud_lut_params},
};

/* COMMON VCLK -> LUT List */
struct vclk_lut cmucal_vclk_blk_cmu_lut[] = {
	{1066500, blk_cmu_nm_lut_params},
	{1066500, blk_cmu_ud_lut_params},
	{1066500, blk_cmu_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_gsactrl_lut[] = {
	{1200000, blk_gsactrl_nm_lut_params},
};
struct vclk_lut cmucal_vclk_blk_s2d_lut[] = {
	{100000, blk_s2d_nm_lut_params},
};
struct vclk_lut cmucal_vclk_blk_apm_lut[] = {
	{400000, blk_apm_nm_lut_params},
};
struct vclk_lut cmucal_vclk_blk_cpucl0_lut[] = {
	{2250000, blk_cpucl0_sod_lut_params},
	{1950000, blk_cpucl0_od_lut_params},
	{1550000, blk_cpucl0_nm_lut_params},
	{1000000, blk_cpucl0_ud_lut_params},
	{667000, blk_cpucl0_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_cpucl1_lut[] = {
	{2600000, blk_cpucl1_sod_lut_params},
	{2250000, blk_cpucl1_od_lut_params},
	{1700000, blk_cpucl1_nm_lut_params},
	{1225000, blk_cpucl1_ud_lut_params},
	{800000, blk_cpucl1_sud_lut_params},
	{475000, blk_cpucl1_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_cpucl2_lut[] = {
	{3300000, blk_cpucl2_sod_lut_params},
	{2700000, blk_cpucl2_od_lut_params},
	{2150000, blk_cpucl2_nm_lut_params},
	{1750000, blk_cpucl2_ud_lut_params},
	{1125000, blk_cpucl2_sud_lut_params},
	{725000, blk_cpucl2_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_gsacore_lut[] = {
	{1200000, blk_gsacore_nm_lut_params},
};
struct vclk_lut cmucal_vclk_blk_hsi0_lut[] = {
	{266625, blk_hsi0_nm_lut_params},
};
struct vclk_lut cmucal_vclk_blk_hsi1_lut[] = {
	{400000, blk_hsi1_ud_lut_params},
	{400000, blk_hsi1_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_nocl0_lut[] = {
	{1160000, blk_nocl0_od_lut_params},
	{1066500, blk_nocl0_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_nocl1b_lut[] = {
	{800000, blk_nocl1b_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_aoc_lut[] = {
	{787000, blk_aoc_nm_lut_params},
	{394000, blk_aoc_ud_lut_params},
	{197000, blk_aoc_sud_lut_params},
	{99000, blk_aoc_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_aur_lut[] = {
	{800000, blk_aur_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_bw_lut[] = {
	{83375, blk_bw_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_dpub_lut[] = {
	{333500, blk_dpub_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_dpuf0_lut[] = {
	{333500, blk_dpuf0_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_dpuf1_lut[] = {
	{333500, blk_dpuf1_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_eh_lut[] = {
	{533250, blk_eh_uud_lut_params},
	{355500, blk_eh_ud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_g2d_lut[] = {
	{266625, blk_g2d_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_g3d_lut[] = {
	{900000, blk_g3d_nm_lut_params},
	{800000, blk_g3d_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_gdc_lut[] = {
	{333500, blk_gdc_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_gse_lut[] = {
	{333500, blk_gse_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_hsi2_lut[] = {
	{533250, blk_hsi2_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_ispfe_lut[] = {
	{355500, blk_ispfe_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_mcsc_lut[] = {
	{333500, blk_mcsc_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_mif_lut[] = {
	{266625, blk_mif_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_misc_lut[] = {
	{266625, blk_misc_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_nocl1a_lut[] = {
	{1066500, blk_nocl1a_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_nocl2aa_lut[] = {
	{800000, blk_nocl2aa_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_nocl2ab_lut[] = {
	{800000, blk_nocl2ab_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_peric0_lut[] = {
	{200000, blk_peric0_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_peric1_lut[] = {
	{66656, blk_peric1_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_rgbp_lut[] = {
	{333500, blk_rgbp_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_tnr_lut[] = {
	{333500, blk_tnr_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_tpu_lut[] = {
	{266625, blk_tpu_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_yuvp_lut[] = {
	{333500, blk_yuvp_uud_lut_params},
};

/* Switch VCLK -> LUT Parameter List */
struct switch_lut mux_clk_aur_aur_lut[] = {
	{1066500, 0, 0},
	{711000, 4, 0},
	{355500, 4, 1},
	{106650, 6, 4},
};
struct switch_lut mux_clk_g3d_stacks_lut[] = {
	{800000, 0, 0},
	{666000, 2, 0},
	{400000, 0, 1},
	{133313, 4, 3},
};
struct switch_lut mux_clk_g3d_l2_glb_lut[] = {
	{800000, 2, 0},
	{711000, 4, 0},
	{400000, 2, 1},
	{133313, 6, 3},
};
struct switch_lut mux_clk_nocl0_noc_lut[] = {
	{1066500, 0, 0},
	{711000, 3, 0},
	{355500, 3, 1},
	{213300, 6, 1},
};
struct switch_lut mux_clk_tpu_tpu_lut[] = {
	{1066500, 0, 0},
	{800000, 2, 0},
	{622000, 5, 0},
	{200000, 2, 3},
};
struct switch_lut mux_clk_tpu_tpuctl_lut[] = {
	{1066500, 0, 0},
};
/*================================ SWPLL List =================================*/
struct vclk_switch switch_vdd_mif[] = {
	{MUX_CLK_NOCL0_NOC, MUX_CLKCMU_NOCL0_NOC, CLKCMU_NOCL0_NOC, GATE_CLKCMU_NOCL0_NOC, MUX_CLKCMU_NOCL0_NOC_USER, mux_clk_nocl0_noc_lut, 4},
};
struct vclk_switch switch_vdd_g3d[] = {
	{MUX_CLK_G3D_STACKS, MUX_CLKCMU_G3D_SWITCH, CLKCMU_G3D_SWITCH, GATE_CLKCMU_G3D_SWITCH, MUX_CLKCMU_G3D_SWITCH_USER, mux_clk_g3d_stacks_lut, 4},
	{MUX_CLK_G3D_L2_GLB, MUX_CLKCMU_G3D_GLB, CLKCMU_G3D_GLB, GATE_CLKCMU_G3D_GLB, MUX_CLKCMU_G3D_GLB_USER, mux_clk_g3d_l2_glb_lut, 4},
};
struct vclk_switch switch_vdd_cam[] = {
	{MUX_CLK_AUR_AUR, MUX_CLKCMU_AUR_AUR, CLKCMU_AUR_AUR, GATE_CLKCMU_AUR_AUR, MUX_CLKCMU_AUR_SWITCH_USER, mux_clk_aur_aur_lut, 4},
};
struct vclk_switch switch_vdd_tpu[] = {
	{MUX_CLK_TPU_TPU, MUX_CLKCMU_TPU_TPU, CLKCMU_TPU_TPU, GATE_CLKCMU_TPU_TPU, MUX_CLKCMU_TPU_TPU_USER, mux_clk_tpu_tpu_lut, 4},
	{MUX_CLK_TPU_TPUCTL, MUX_CLKCMU_TPU_TPUCTL, CLKCMU_TPU_TPUCTL, GATE_CLKCMU_TPU_TPUCTL, MUX_CLKCMU_TPU_TPUCTL_USER, mux_clk_tpu_tpuctl_lut, 1},
};

/*================================ VCLK List =================================*/
struct vclk cmucal_vclk_list[] = {
/* DVFS VCLK*/
	CMUCAL_VCLK(VCLK_VDD_INT, cmucal_vclk_vdd_int_lut, cmucal_vclk_vdd_int, NULL, NULL),
	CMUCAL_VCLK(VCLK_VDD_MIF, cmucal_vclk_vdd_mif_lut, cmucal_vclk_vdd_mif, NULL, switch_vdd_mif),
	CMUCAL_VCLK(VCLK_VDD_G3D, cmucal_vclk_vdd_g3d_lut, cmucal_vclk_vdd_g3d, NULL, switch_vdd_g3d),
	CMUCAL_VCLK(VCLK_VDD_CAM, cmucal_vclk_vdd_cam_lut, cmucal_vclk_vdd_cam, NULL, switch_vdd_cam),
	CMUCAL_VCLK(VCLK_VDD_CPUCL0, cmucal_vclk_vdd_cpucl0_lut, cmucal_vclk_vdd_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_VDD_CPUCL1, cmucal_vclk_vdd_cpucl1_lut, cmucal_vclk_vdd_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_VDD_TPU, cmucal_vclk_vdd_tpu_lut, cmucal_vclk_vdd_tpu, NULL, switch_vdd_tpu),
	CMUCAL_VCLK(VCLK_VDD_CPUCL2, cmucal_vclk_vdd_cpucl2_lut, cmucal_vclk_vdd_cpucl2, NULL, NULL),

/* SPECIAL VCLK*/
	CMUCAL_VCLK(VCLK_MUX_CMU_CMUREF, cmucal_vclk_mux_cmu_cmuref_lut, cmucal_vclk_mux_cmu_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CPUCL0_CMUREF, cmucal_vclk_mux_cpucl0_cmuref_lut, cmucal_vclk_mux_cpucl0_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CPUCL1_CMUREF, cmucal_vclk_mux_cpucl1_cmuref_lut, cmucal_vclk_mux_cpucl1_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CPUCL2_CMUREF, cmucal_vclk_mux_cpucl2_cmuref_lut, cmucal_vclk_mux_cpucl2_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLK_HSI0_USB20_REF, cmucal_vclk_mux_clk_hsi0_usb20_ref_lut, cmucal_vclk_mux_clk_hsi0_usb20_ref, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLKCMU_HSI0_USB32DRD, cmucal_vclk_clkcmu_hsi0_usb32drd_lut, cmucal_vclk_clkcmu_hsi0_usb32drd, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_MIF_CMUREF, cmucal_vclk_mux_mif_cmuref_lut, cmucal_vclk_mux_mif_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_NOCL0_CMUREF, cmucal_vclk_mux_nocl0_cmuref_lut, cmucal_vclk_mux_nocl0_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_NOCL1B_CMUREF, cmucal_vclk_mux_nocl1b_cmuref_lut, cmucal_vclk_mux_nocl1b_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_NOCL2AA_CMUREF, cmucal_vclk_mux_nocl2aa_cmuref_lut, cmucal_vclk_mux_nocl2aa_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_NOCL2AB_CMUREF, cmucal_vclk_mux_nocl2ab_cmuref_lut, cmucal_vclk_mux_nocl2ab_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLKCMU_DPUB_DSIM, cmucal_vclk_clkcmu_dpub_dsim_lut, cmucal_vclk_clkcmu_dpub_dsim, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLKCMU_HSI0_DPGTC, cmucal_vclk_clkcmu_hsi0_dpgtc_lut, cmucal_vclk_clkcmu_hsi0_dpgtc, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_APM_USI0_USI, cmucal_vclk_div_clk_apm_usi0_usi_lut, cmucal_vclk_div_clk_apm_usi0_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_APM_USI0_UART, cmucal_vclk_div_clk_apm_usi0_uart_lut, cmucal_vclk_div_clk_apm_usi0_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_APM_USI1_UART, cmucal_vclk_div_clk_apm_usi1_uart_lut, cmucal_vclk_div_clk_apm_usi1_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_APM_I3C_PMIC, cmucal_vclk_div_clk_apm_i3c_pmic_lut, cmucal_vclk_div_clk_apm_i3c_pmic, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLK_AUR_ADD_CH_CLK, cmucal_vclk_clk_aur_add_ch_clk_lut, cmucal_vclk_clk_aur_add_ch_clk, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK0, cmucal_vclk_mux_clkcmu_cis_clk0_lut, cmucal_vclk_mux_clkcmu_cis_clk0, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK1, cmucal_vclk_mux_clkcmu_cis_clk1_lut, cmucal_vclk_mux_clkcmu_cis_clk1, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK2, cmucal_vclk_mux_clkcmu_cis_clk2_lut, cmucal_vclk_mux_clkcmu_cis_clk2, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK3, cmucal_vclk_mux_clkcmu_cis_clk3_lut, cmucal_vclk_mux_clkcmu_cis_clk3, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK4, cmucal_vclk_mux_clkcmu_cis_clk4_lut, cmucal_vclk_mux_clkcmu_cis_clk4, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK5, cmucal_vclk_mux_clkcmu_cis_clk5_lut, cmucal_vclk_mux_clkcmu_cis_clk5, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK6, cmucal_vclk_mux_clkcmu_cis_clk6_lut, cmucal_vclk_mux_clkcmu_cis_clk6, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK7, cmucal_vclk_mux_clkcmu_cis_clk7_lut, cmucal_vclk_mux_clkcmu_cis_clk7, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_CPUCL0_CMUREF, cmucal_vclk_div_clk_cpucl0_cmuref_lut, cmucal_vclk_div_clk_cpucl0_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_CPUCL0_ADD_CH_CLK, cmucal_vclk_div_clk_cpucl0_add_ch_clk_lut, cmucal_vclk_div_clk_cpucl0_add_ch_clk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_CPUCL1_CMUREF, cmucal_vclk_div_clk_cpucl1_cmuref_lut, cmucal_vclk_div_clk_cpucl1_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_CPUCL2_CMUREF, cmucal_vclk_div_clk_cpucl2_cmuref_lut, cmucal_vclk_div_clk_cpucl2_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLK_G3D_ADD_CH_CLK, cmucal_vclk_clk_g3d_add_ch_clk_lut, cmucal_vclk_clk_g3d_add_ch_clk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_GSACORE_SPI_FPS, cmucal_vclk_div_clk_gsacore_spi_fps_lut, cmucal_vclk_div_clk_gsacore_spi_fps, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_GSACORE_SPI_GSC, cmucal_vclk_div_clk_gsacore_spi_gsc_lut, cmucal_vclk_div_clk_gsacore_spi_gsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_GSACORE_UART, cmucal_vclk_div_clk_gsacore_uart_lut, cmucal_vclk_div_clk_gsacore_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLKCMU_HSI0_PERI, cmucal_vclk_clkcmu_hsi0_peri_lut, cmucal_vclk_clkcmu_hsi0_peri, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_HSI0_USI0, cmucal_vclk_div_clk_hsi0_usix_lut, cmucal_vclk_div_clk_hsi0_usi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_HSI0_USI1, cmucal_vclk_div_clk_hsi0_usix_lut, cmucal_vclk_div_clk_hsi0_usi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_HSI0_USI2, cmucal_vclk_div_clk_hsi0_usix_lut, cmucal_vclk_div_clk_hsi0_usi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_HSI0_USI3, cmucal_vclk_div_clk_hsi0_usix_lut, cmucal_vclk_div_clk_hsi0_usi3, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_HSI0_USI4, cmucal_vclk_div_clk_hsi0_usix_lut, cmucal_vclk_div_clk_hsi0_usi4, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_SLC_DCLK, cmucal_vclk_div_clk_slc_dclk_lut, cmucal_vclk_div_clk_slc_dclk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_SLC1_DCLK, cmucal_vclk_div_clk_slc1_dclk_lut, cmucal_vclk_div_clk_slc1_dclk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_SLC2_DCLK, cmucal_vclk_div_clk_slc2_dclk_lut, cmucal_vclk_div_clk_slc2_dclk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_SLC3_DCLK, cmucal_vclk_div_clk_slc3_dclk_lut, cmucal_vclk_div_clk_slc3_dclk, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_PERIC0_IP, cmucal_vclk_mux_clkcmu_peric0_ip_lut, cmucal_vclk_mux_clkcmu_peric0_ip, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI0_UART, cmucal_vclk_div_clk_peric0_usi0_uart_lut, cmucal_vclk_div_clk_peric0_usi0_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI1_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric0_usi1_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI2_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric0_usi2_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI3_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric0_usi3_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI4_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric0_usi4_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI5_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric0_usi5_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI6_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric0_usi6_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI14_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric0_usi14_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_PERIC1_IP, cmucal_vclk_mux_clkcmu_peric1_ip_lut, cmucal_vclk_mux_clkcmu_peric1_ip, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_I3C, cmucal_vclk_div_clk_peric1_i3c_lut, cmucal_vclk_div_clk_peric1_i3c, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI0_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric1_usi0_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI9_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric1_usi9_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI10_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric1_usi10_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI11_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric1_usi11_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI12_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric1_usi12_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI13_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric1_usi13_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI15_USI, cmucal_vclk_div_clk_pericx_usix_usi_lut, cmucal_vclk_div_clk_peric1_usi15_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLK_TPU_ADD_CH_CLK, cmucal_vclk_clk_tpu_add_ch_clk_lut, cmucal_vclk_clk_tpu_add_ch_clk, NULL, NULL),

/* COMMON VCLK*/
	CMUCAL_VCLK(VCLK_BLK_CMU, cmucal_vclk_blk_cmu_lut, cmucal_vclk_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_GSACTRL, cmucal_vclk_blk_gsactrl_lut, cmucal_vclk_blk_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_S2D, cmucal_vclk_blk_s2d_lut, cmucal_vclk_blk_s2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_APM, cmucal_vclk_blk_apm_lut, cmucal_vclk_blk_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CPUCL0, cmucal_vclk_blk_cpucl0_lut, cmucal_vclk_blk_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CPUCL1, cmucal_vclk_blk_cpucl1_lut, cmucal_vclk_blk_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CPUCL2, cmucal_vclk_blk_cpucl2_lut, cmucal_vclk_blk_cpucl2, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_GSACORE, cmucal_vclk_blk_gsacore_lut, cmucal_vclk_blk_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_HSI0, cmucal_vclk_blk_hsi0_lut, cmucal_vclk_blk_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_HSI1, cmucal_vclk_blk_hsi1_lut, cmucal_vclk_blk_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_NOCL0, cmucal_vclk_blk_nocl0_lut, cmucal_vclk_blk_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_NOCL1B, cmucal_vclk_blk_nocl1b_lut, cmucal_vclk_blk_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_AOC, cmucal_vclk_blk_aoc_lut, cmucal_vclk_blk_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_AUR, cmucal_vclk_blk_aur_lut, cmucal_vclk_blk_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_BW, cmucal_vclk_blk_bw_lut, cmucal_vclk_blk_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_DPUB, cmucal_vclk_blk_dpub_lut, cmucal_vclk_blk_dpub, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_DPUF0, cmucal_vclk_blk_dpuf0_lut, cmucal_vclk_blk_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_DPUF1, cmucal_vclk_blk_dpuf1_lut, cmucal_vclk_blk_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_EH, cmucal_vclk_blk_eh_lut, cmucal_vclk_blk_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_G2D, cmucal_vclk_blk_g2d_lut, cmucal_vclk_blk_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_G3D, cmucal_vclk_blk_g3d_lut, cmucal_vclk_blk_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_GDC, cmucal_vclk_blk_gdc_lut, cmucal_vclk_blk_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_GSE, cmucal_vclk_blk_gse_lut, cmucal_vclk_blk_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_HSI2, cmucal_vclk_blk_hsi2_lut, cmucal_vclk_blk_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_ISPFE, cmucal_vclk_blk_ispfe_lut, cmucal_vclk_blk_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_MCSC, cmucal_vclk_blk_mcsc_lut, cmucal_vclk_blk_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_MIF, cmucal_vclk_blk_mif_lut, cmucal_vclk_blk_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_MISC, cmucal_vclk_blk_misc_lut, cmucal_vclk_blk_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_NOCL1A, cmucal_vclk_blk_nocl1a_lut, cmucal_vclk_blk_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_NOCL2AA, cmucal_vclk_blk_nocl2aa_lut, cmucal_vclk_blk_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_NOCL2AB, cmucal_vclk_blk_nocl2ab_lut, cmucal_vclk_blk_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_PERIC0, cmucal_vclk_blk_peric0_lut, cmucal_vclk_blk_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_PERIC1, cmucal_vclk_blk_peric1_lut, cmucal_vclk_blk_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_RGBP, cmucal_vclk_blk_rgbp_lut, cmucal_vclk_blk_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_TNR, cmucal_vclk_blk_tnr_lut, cmucal_vclk_blk_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_TPU, cmucal_vclk_blk_tpu_lut, cmucal_vclk_blk_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_YUVP, cmucal_vclk_blk_yuvp_lut, cmucal_vclk_blk_yuvp, NULL, NULL),

/* GATE VCLK*/
	CMUCAL_VCLK(VCLK_IP_AOC_CMU_AOC, NULL, cmucal_vclk_ip_aoc_cmu_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BAAW_AOC, NULL, cmucal_vclk_ip_baaw_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_AOC, NULL, cmucal_vclk_ip_d_tzpc_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_AOC, NULL, cmucal_vclk_ip_gpc_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LD_HSI0_AOC, NULL, cmucal_vclk_ip_lh_axi_mi_ld_hsi0_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D_AOC, NULL, cmucal_vclk_ip_lh_axi_si_d_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_AOC, NULL, cmucal_vclk_ip_ppmu_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_USB, NULL, cmucal_vclk_ip_ppmu_usb, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_AOC, NULL, cmucal_vclk_ip_ssmt_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU_AOC, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_AOC, NULL, cmucal_vclk_ip_sysreg_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_AOC, NULL, cmucal_vclk_ip_uasc_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_DP_AOC, NULL, cmucal_vclk_ip_xiu_dp_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P_AOC, NULL, cmucal_vclk_ip_xiu_p_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AOC_SYSCTRL_APB, NULL, cmucal_vclk_ip_aoc_sysctrl_apb, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_AOC_ALIVE_CD, NULL, cmucal_vclk_ip_lh_axi_si_lp_aoc_alive_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_AOC_ALIVE_CD, NULL, cmucal_vclk_ip_lh_axi_mi_lp_aoc_alive_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_AOC_HSI0_CD, NULL, cmucal_vclk_ip_lh_axi_si_lp_aoc_hsi0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_AOC_HSI0_CD, NULL, cmucal_vclk_ip_lh_axi_mi_lp_aoc_hsi0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_LP_AOC_ALIVE, NULL, cmucal_vclk_ip_slh_axi_si_lp_aoc_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_LP_AOC_HSI0, NULL, cmucal_vclk_ip_slh_axi_si_lp_aoc_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_AOC, NULL, cmucal_vclk_ip_lh_atb_si_lt_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_AOC_CD, NULL, cmucal_vclk_ip_lh_atb_mi_lt_aoc_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_AOC, NULL, cmucal_vclk_ip_slh_axi_mi_p_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_AOC_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_aoc_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_AOC_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_aoc_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_LG_ALIVE_AOC, NULL, cmucal_vclk_ip_slh_axi_mi_lg_alive_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_AOC_CD, NULL, cmucal_vclk_ip_lh_atb_si_lt_aoc_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LD_HSI1_AOC, NULL, cmucal_vclk_ip_lh_axi_mi_ld_hsi1_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_PCIE, NULL, cmucal_vclk_ip_ppmu_pcie, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_AOC, NULL, cmucal_vclk_ip_sysmmu_s0_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_AOC_HSI1_CD, NULL, cmucal_vclk_ip_lh_axi_si_lp_aoc_hsi1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_AOC_HSI1_CD, NULL, cmucal_vclk_ip_lh_axi_mi_lp_aoc_hsi1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_LP_AOC_HSI1, NULL, cmucal_vclk_ip_slh_axi_si_lp_aoc_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D_ALIVE, NULL, cmucal_vclk_ip_lh_axi_si_d_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_WDT_APM, NULL, cmucal_vclk_ip_wdt_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_APM, NULL, cmucal_vclk_ip_sysreg_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_AP, NULL, cmucal_vclk_ip_mailbox_apm_ap, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_PMU_ALIVE, NULL, cmucal_vclk_ip_apbif_pmu_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_INTMEM, NULL, cmucal_vclk_ip_intmem, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PMU_INTR_GEN, NULL, cmucal_vclk_ip_pmu_intr_gen, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_DP_ALIVE, NULL, cmucal_vclk_ip_xiu_dp_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_CMU_APM, NULL, cmucal_vclk_ip_apm_cmu_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GREBEINTEGRATION, NULL, cmucal_vclk_ip_grebeintegration, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_GPIO_ALIVE, NULL, cmucal_vclk_ip_apbif_gpio_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TRTC, NULL, cmucal_vclk_ip_trtc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_APM, NULL, cmucal_vclk_ip_d_tzpc_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_AOC, NULL, cmucal_vclk_ip_mailbox_apm_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_DBGCORE, NULL, cmucal_vclk_ip_mailbox_ap_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_RTC, NULL, cmucal_vclk_ip_rtc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_GSA, NULL, cmucal_vclk_ip_mailbox_apm_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D_ALIVE, NULL, cmucal_vclk_ip_ssmt_d_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_LP_ALIVE_CPUCL0, NULL, cmucal_vclk_ip_ssmt_lp_alive_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_ALIVE, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_APM, NULL, cmucal_vclk_ip_gpc_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_GPIO_FAR_ALIVE, NULL, cmucal_vclk_ip_apbif_gpio_far_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ROM_CRC32_HOST, NULL, cmucal_vclk_ip_rom_crc32_host, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SS_DBGCORE, NULL, cmucal_vclk_ip_ss_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_SWD, NULL, cmucal_vclk_ip_mailbox_apm_swd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_TPU, NULL, cmucal_vclk_ip_mailbox_apm_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IG_SWD, NULL, cmucal_vclk_ip_lh_axi_mi_ig_swd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_USI0_UART, NULL, cmucal_vclk_ip_apm_usi0_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_USI1_UART_INT, NULL, cmucal_vclk_ip_apm_usi1_uart_int, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_USI0_USI, NULL, cmucal_vclk_ip_apm_usi0_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AOCA32, NULL, cmucal_vclk_ip_mailbox_ap_aoca32, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AOCF1, NULL, cmucal_vclk_ip_mailbox_ap_aocf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AOCP6, NULL, cmucal_vclk_ip_mailbox_ap_aocp6, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AURMCUTZ, NULL, cmucal_vclk_ip_mailbox_ap_aurmcutz, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AURCORE0, NULL, cmucal_vclk_ip_mailbox_ap_aurcore0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AURCORE1, NULL, cmucal_vclk_ip_mailbox_ap_aurcore1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AURCORE2, NULL, cmucal_vclk_ip_mailbox_ap_aurcore2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_I3C_PMIC, NULL, cmucal_vclk_ip_apm_i3c_pmic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_INTCOMB_VGPIO2PMU, NULL, cmucal_vclk_ip_apbif_intcomb_vgpio2pmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_INTCOMB_VGPIO2AP, NULL, cmucal_vclk_ip_apbif_intcomb_vgpio2ap, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_INTCOMB_VGPIO2APM, NULL, cmucal_vclk_ip_apbif_intcomb_vgpio2apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_AUR, NULL, cmucal_vclk_ip_mailbox_apm_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_LP_ALIVE_CPUCL0, NULL, cmucal_vclk_ip_slh_axi_si_lp_alive_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_LG_SCAN2DRAM, NULL, cmucal_vclk_ip_slh_axi_si_lg_scan2dram, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_ALIVE, NULL, cmucal_vclk_ip_slh_axi_mi_p_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_LP_AOC_ALIVE, NULL, cmucal_vclk_ip_slh_axi_mi_lp_aoc_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_ALIVE_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_axi_si_lp_alive_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_ALIVE_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_axi_mi_lp_alive_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LG_SCAN2DRAM_CD, NULL, cmucal_vclk_ip_lh_axi_si_lg_scan2dram_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LG_SCAN2DRAM_CD, NULL, cmucal_vclk_ip_lh_axi_mi_lg_scan2dram_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_AOC_ALIVE_CU, NULL, cmucal_vclk_ip_lh_axi_si_lp_aoc_alive_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_AOC_ALIVE_CU, NULL, cmucal_vclk_ip_lh_axi_mi_lp_aoc_alive_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_ALIVE_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_alive_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_ALIVE_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_alive_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_APM_CUSTOM, NULL, cmucal_vclk_ip_d_tzpc_apm_custom, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_APM_CUSTOM, NULL, cmucal_vclk_ip_gpc_apm_custom, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_DMA, NULL, cmucal_vclk_ip_apm_dma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AURMCUNS0, NULL, cmucal_vclk_ip_mailbox_ap_aurmcuns0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AOC_AURCORE0, NULL, cmucal_vclk_ip_mailbox_aoc_aurcore0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AOC_AURCORE1, NULL, cmucal_vclk_ip_mailbox_aoc_aurcore1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AOC_AURCORE2, NULL, cmucal_vclk_ip_mailbox_aoc_aurcore2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_ALIVE, NULL, cmucal_vclk_ip_sysmmu_s0_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_APM_CUSTOM, NULL, cmucal_vclk_ip_sysreg_apm_custom, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_ALIVE, NULL, cmucal_vclk_ip_xiu_d_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PMU, NULL, cmucal_vclk_ip_pmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_GPIO_CUSTOM_ALIVE, NULL, cmucal_vclk_ip_apbif_gpio_custom_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AURMCUNS1, NULL, cmucal_vclk_ip_mailbox_ap_aurmcuns1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AURMCUNS2, NULL, cmucal_vclk_ip_mailbox_ap_aurmcuns2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AURMCUNS3, NULL, cmucal_vclk_ip_mailbox_ap_aurmcuns3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AURMCUNS4, NULL, cmucal_vclk_ip_mailbox_ap_aurmcuns4, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AOC_AURMCU, NULL, cmucal_vclk_ip_mailbox_aoc_aurmcu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_AURMCU, NULL, cmucal_vclk_ip_mailbox_apm_aurmcu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_TPU_AURMCU, NULL, cmucal_vclk_ip_mailbox_tpu_aurmcu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AUR_CMU_AUR, NULL, cmucal_vclk_ip_aur_cmu_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AUR, NULL, cmucal_vclk_ip_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_SYSMMU_S1_NS_AUR, NULL, cmucal_vclk_ip_as_apb_sysmmu_s1_ns_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_AUR, NULL, cmucal_vclk_ip_d_tzpc_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_AUR, NULL, cmucal_vclk_ip_gpc_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D0_AUR, NULL, cmucal_vclk_ip_lh_acel_si_d0_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_AUR, NULL, cmucal_vclk_ip_ssmt_d0_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_AUR, NULL, cmucal_vclk_ip_ssmt_d1_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_AUR, NULL, cmucal_vclk_ip_ppmu_d0_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_AUR, NULL, cmucal_vclk_ip_ppmu_d1_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_AUR, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_AUR, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_AUR, NULL, cmucal_vclk_ip_sysreg_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_P0_AUR, NULL, cmucal_vclk_ip_uasc_p0_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D1_AUR, NULL, cmucal_vclk_ip_lh_acel_si_d1_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADM_DAP_G_AUR, NULL, cmucal_vclk_ip_adm_dap_g_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD_APBIF_AUR, NULL, cmucal_vclk_ip_add_apbif_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BAAW_AUR, NULL, cmucal_vclk_ip_baaw_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_AUR_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_si_lt_aur_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_AUR_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_atb_si_lt_aur_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_AUR_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_atb_mi_lt_aur_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_AUR_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_aur_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_AUR, NULL, cmucal_vclk_ip_slh_axi_mi_p_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_AUR_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_aur_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_P_AUR, NULL, cmucal_vclk_ip_ssmt_p_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_P1_AUR, NULL, cmucal_vclk_ip_uasc_p1_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_AUR, NULL, cmucal_vclk_ip_sysmmu_s0_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_AUR, NULL, cmucal_vclk_ip_xiu_d_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DAPAPBAP_AUR, NULL, cmucal_vclk_ip_dapapbap_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_AUR_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_aur_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD_AUR, NULL, cmucal_vclk_ip_add_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BW_CMU_BW, NULL, cmucal_vclk_ip_bw_cmu_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D_BW, NULL, cmucal_vclk_ip_lh_axi_si_d_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_BW, NULL, cmucal_vclk_ip_slh_axi_mi_p_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_BW, NULL, cmucal_vclk_ip_ppmu_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_BW, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_SYSMMU_S0_BW_S2, NULL, cmucal_vclk_ip_as_apb_sysmmu_s0_bw_s2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_BW, NULL, cmucal_vclk_ip_sysreg_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_BW, NULL, cmucal_vclk_ip_ssmt_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_BW, NULL, cmucal_vclk_ip_d_tzpc_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_BW, NULL, cmucal_vclk_ip_gpc_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_BW, NULL, cmucal_vclk_ip_uasc_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BW, NULL, cmucal_vclk_ip_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_BW, NULL, cmucal_vclk_ip_lh_axi_si_ip_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_BW, NULL, cmucal_vclk_ip_lh_axi_mi_ip_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_BW, NULL, cmucal_vclk_ip_xiu_d_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_BW, NULL, cmucal_vclk_ip_sysmmu_s0_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_BW, NULL, cmucal_vclk_ip_trex_d_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_BW_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_bw_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_CPUCL0, NULL, cmucal_vclk_ip_sysreg_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CSSYS, NULL, cmucal_vclk_ip_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_IT1_BOOKER, NULL, cmucal_vclk_ip_lh_atb_mi_it1_booker, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL0_CMU_CPUCL0, NULL, cmucal_vclk_ip_cpucl0_cmu_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_CPUCL0, NULL, cmucal_vclk_ip_d_tzpc_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IG_CSSYS, NULL, cmucal_vclk_ip_lh_axi_si_ig_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APB_ASYNC_P_CSSYS_0, NULL, cmucal_vclk_ip_apb_async_p_cssys_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_CPUCL0, NULL, cmucal_vclk_ip_gpc_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_DP_CSSYS, NULL, cmucal_vclk_ip_xiu_dp_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_CPUCL0, NULL, cmucal_vclk_ip_ssmt_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_S2MPU_S0_CPUCL0, NULL, cmucal_vclk_ip_s2mpu_s0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IG_HSI0, NULL, cmucal_vclk_ip_lh_axi_si_ig_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IG_STM, NULL, cmucal_vclk_ip_lh_axi_si_ig_stm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IG_STM, NULL, cmucal_vclk_ip_lh_axi_mi_ig_stm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_G_CSSYS_CD, NULL, cmucal_vclk_ip_lh_axi_si_g_cssys_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_G_CSSYS_CD, NULL, cmucal_vclk_ip_lh_axi_mi_g_cssys_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_ICC_CLUSTER0_GIC, NULL, cmucal_vclk_ip_lh_ast_si_l_icc_cluster0_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_ICC_CLUSTER0_GIC_CD, NULL, cmucal_vclk_ip_lh_ast_si_l_icc_cluster0_gic_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_ICC_CLUSTER0_GIC_CD, NULL, cmucal_vclk_ip_lh_ast_mi_l_icc_cluster0_gic_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_LG_ETR_HSI0, NULL, cmucal_vclk_ip_slh_axi_si_lg_etr_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LG_ETR_HSI0_CD, NULL, cmucal_vclk_ip_lh_axi_si_lg_etr_hsi0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LG_ETR_HSI0_CD, NULL, cmucal_vclk_ip_lh_axi_mi_lg_etr_hsi0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_IRI_GIC_CLUSTER0, NULL, cmucal_vclk_ip_lh_ast_mi_l_iri_gic_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_IRI_GIC_CLUSTER0_CU, NULL, cmucal_vclk_ip_lh_ast_si_l_iri_gic_cluster0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_IRI_GIC_CLUSTER0_CU, NULL, cmucal_vclk_ip_lh_ast_mi_l_iri_gic_cluster0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_AOC, NULL, cmucal_vclk_ip_lh_atb_mi_lt_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_AOC_CU, NULL, cmucal_vclk_ip_lh_atb_si_lt_aoc_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_AOC_CU, NULL, cmucal_vclk_ip_lh_atb_mi_lt_aoc_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_AUR_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_mi_lt_aur_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_AUR_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_atb_si_lt_aur_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_AUR_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_atb_mi_lt_aur_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_GSA_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_mi_lt_gsa_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_GSA_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_atb_si_lt_gsa_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_GSA_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_atb_mi_lt_gsa_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_CPUCL0, NULL, cmucal_vclk_ip_slh_axi_mi_p_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT0_TPU_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_mi_lt0_tpu_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT1_TPU_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_mi_lt1_tpu_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT0_TPU_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_atb_si_lt0_tpu_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT0_TPU_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_atb_mi_lt0_tpu_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT1_TPU_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_atb_si_lt1_tpu_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT1_TPU_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_atb_mi_lt1_tpu_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_T_BDU, NULL, cmucal_vclk_ip_lh_atb_mi_t_bdu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_T_SLC, NULL, cmucal_vclk_ip_lh_atb_mi_t_slc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_T_BDU_CU, NULL, cmucal_vclk_ip_lh_atb_si_t_bdu_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_T_BDU_CU, NULL, cmucal_vclk_ip_lh_atb_mi_t_bdu_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_T_SLC_CU, NULL, cmucal_vclk_ip_lh_atb_si_t_slc_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_T_SLC_CU, NULL, cmucal_vclk_ip_lh_atb_mi_t_slc_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL0_CON, NULL, cmucal_vclk_ip_cpucl0_con, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APB_ASYNC_P_CSSYS_1, NULL, cmucal_vclk_ip_apb_async_p_cssys_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_ID_PPU, NULL, cmucal_vclk_ip_lh_axi_si_id_ppu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_BOOKER, NULL, cmucal_vclk_ip_lh_axi_mi_ip_booker, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_ID_PPU, NULL, cmucal_vclk_ip_lh_axi_mi_id_ppu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CLUSTER0, NULL, cmucal_vclk_ip_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_LP_ALIVE_CPUCL0, NULL, cmucal_vclk_ip_slh_axi_mi_lp_alive_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_ALIVE_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_axi_si_lp_alive_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_ALIVE_CPUCL0_CU, NULL, cmucal_vclk_ip_lh_axi_mi_lp_alive_cpucl0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_CPUCL0_D0, NULL, cmucal_vclk_ip_ppmu_cpucl0_d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_CPUCL0_D1, NULL, cmucal_vclk_ip_ppmu_cpucl0_d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_CPUCL0_D2, NULL, cmucal_vclk_ip_ppmu_cpucl0_d2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_CPUCL0_D3, NULL, cmucal_vclk_ip_ppmu_cpucl0_d3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_G_CSSYS, NULL, cmucal_vclk_ip_slh_axi_si_g_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_CPUCL0_HSI1, NULL, cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_CPUCL0_HSI2, NULL, cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_CPUCL0_HSI2_CD, NULL, cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi2_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_CPUCL0_HSI1_CD, NULL, cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD0_CPUCL0, NULL, cmucal_vclk_ip_add0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD0_APBIF_CPUCL0, NULL, cmucal_vclk_ip_add0_apbif_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_CPUCL1, NULL, cmucal_vclk_ip_lh_axi_si_ip_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_CPUCL2, NULL, cmucal_vclk_ip_lh_axi_si_ip_cpucl2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IG_CSSYS, NULL, cmucal_vclk_ip_lh_axi_mi_ig_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_INSTRRUN_CLUSTER0_0, NULL, cmucal_vclk_ip_ppc_instrrun_cluster0_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_INSTRRUN_CLUSTER0_1, NULL, cmucal_vclk_ip_ppc_instrrun_cluster0_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_INSTRRET_CLUSTER0_0, NULL, cmucal_vclk_ip_ppc_instrret_cluster0_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_INSTRRET_CLUSTER0_1, NULL, cmucal_vclk_ip_ppc_instrret_cluster0_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_G3D_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_mi_lt_g3d_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_S2MPU_S0_PMMU0_CPUCL0, NULL, cmucal_vclk_ip_s2mpu_s0_pmmu0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IG_HSI0, NULL, cmucal_vclk_ip_lh_axi_mi_ig_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL0_QOS, NULL, cmucal_vclk_ip_cpucl0_qos, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AXI_DS_128TO32_P_PCIE_CLUSTER0, NULL, cmucal_vclk_ip_axi_ds_128to32_p_pcie_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_CPUCL0_NOCL0, NULL, cmucal_vclk_ip_lh_axi_si_p_cpucl0_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D0_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_si_d0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D1_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_si_d1_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D2_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_si_d2_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D3_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_si_d3_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_BOOKER, NULL, cmucal_vclk_ip_lh_axi_si_ip_booker, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_CPUCL0_HSI2_CD, NULL, cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi2_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_CPUCL0_HSI1_CD, NULL, cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IG_BOOKER, NULL, cmucal_vclk_ip_lh_axi_si_ig_booker, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IG_BOOKER, NULL, cmucal_vclk_ip_lh_axi_mi_ig_booker, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APB_ASYNC_P_SYSMMU, NULL, cmucal_vclk_ip_apb_async_p_sysmmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GRAY2BIN_CNTVALUEB, NULL, cmucal_vclk_ip_gray2bin_cntvalueb, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GRAY2BIN_TSVALUEB, NULL, cmucal_vclk_ip_gray2bin_tsvalueb, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APB_ASYNC_P_BOOKER_0, NULL, cmucal_vclk_ip_apb_async_p_booker_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APB_ASYNC_P_PCSM, NULL, cmucal_vclk_ip_apb_async_p_pcsm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADM_APB_G_CLUSTER0, NULL, cmucal_vclk_ip_adm_apb_g_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BPS_CPUCL0, NULL, cmucal_vclk_ip_bps_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P0_CPUCL0, NULL, cmucal_vclk_ip_xiu_p0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P2_CPUCL0, NULL, cmucal_vclk_ip_xiu_p2_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_IT1_BOOKER, NULL, cmucal_vclk_ip_lh_atb_si_it1_booker, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D0_NOCL0_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_mi_d0_nocl0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D1_NOCL0_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_mi_d1_nocl0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_LD_EH_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_mi_ld_eh_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL1_CMU_CPUCL1, NULL, cmucal_vclk_ip_cpucl1_cmu_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_CPUCL1, NULL, cmucal_vclk_ip_lh_axi_mi_ip_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD1_APBIF_CPUCL1, NULL, cmucal_vclk_ip_add1_apbif_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL2_CMU_CPUCL2, NULL, cmucal_vclk_ip_cpucl2_cmu_cpucl2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD2_APBIF_CPUCL2, NULL, cmucal_vclk_ip_add2_apbif_cpucl2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_CPUCL2, NULL, cmucal_vclk_ip_lh_axi_mi_ip_cpucl2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DPUB_CMU_DPUB, NULL, cmucal_vclk_ip_dpub_cmu_dpub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_DECON_MAIN, NULL, cmucal_vclk_ip_ad_apb_decon_main, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DPUB, NULL, cmucal_vclk_ip_dpub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_DPUB, NULL, cmucal_vclk_ip_slh_axi_mi_p_dpub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_DPUB, NULL, cmucal_vclk_ip_d_tzpc_dpub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_DPUB, NULL, cmucal_vclk_ip_gpc_dpub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_DPUB, NULL, cmucal_vclk_ip_sysreg_dpub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_DPUB_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_dpub_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DPUF0_CMU_DPUF0, NULL, cmucal_vclk_ip_dpuf0_cmu_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_DPUF0, NULL, cmucal_vclk_ip_sysreg_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_DPUF0, NULL, cmucal_vclk_ip_sysmmu_s0_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_DPUF0, NULL, cmucal_vclk_ip_slh_axi_mi_p_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D1_DPUF0, NULL, cmucal_vclk_ip_lh_axi_si_d1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_DPUF0, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_DPUF0, NULL, cmucal_vclk_ip_ppmu_d0_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_DPUF0, NULL, cmucal_vclk_ip_ppmu_d1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D0_DPUF0, NULL, cmucal_vclk_ip_lh_axi_si_d0_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DPUF0, NULL, cmucal_vclk_ip_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_DPUF0, NULL, cmucal_vclk_ip_d_tzpc_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_DPUF0_DMA, NULL, cmucal_vclk_ip_ad_apb_dpuf0_dma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_DPUF0, NULL, cmucal_vclk_ip_gpc_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_DPUF0, NULL, cmucal_vclk_ip_xiu_d0_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_DPUF0, NULL, cmucal_vclk_ip_xiu_d1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LD0_DPUF1_DPUF0, NULL, cmucal_vclk_ip_lh_axi_mi_ld0_dpuf1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LD1_DPUF1_DPUF0, NULL, cmucal_vclk_ip_lh_axi_mi_ld1_dpuf1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_DPUF0, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_DPUF0, NULL, cmucal_vclk_ip_ssmt_d0_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_DPUF0, NULL, cmucal_vclk_ip_ssmt_d1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_DPUF0_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_dpuf0_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_DPUF1_DMA, NULL, cmucal_vclk_ip_ad_apb_dpuf1_dma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_DPUF1, NULL, cmucal_vclk_ip_sysreg_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DPUF1_CMU_DPUF1, NULL, cmucal_vclk_ip_dpuf1_cmu_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_DPUF1, NULL, cmucal_vclk_ip_ppmu_d0_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_DPUF1, NULL, cmucal_vclk_ip_d_tzpc_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_DPUF1, NULL, cmucal_vclk_ip_gpc_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DPUF1, NULL, cmucal_vclk_ip_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_DPUF1, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LD0_DPUF1_DPUF0, NULL, cmucal_vclk_ip_lh_axi_si_ld0_dpuf1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LD1_DPUF1_DPUF0, NULL, cmucal_vclk_ip_lh_axi_si_ld1_dpuf1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_DPUF1, NULL, cmucal_vclk_ip_slh_axi_mi_p_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_DPUF1, NULL, cmucal_vclk_ip_ppmu_d1_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_DPUF1, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_DPUF1, NULL, cmucal_vclk_ip_sysmmu_s0_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_DPUF1, NULL, cmucal_vclk_ip_xiu_d_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_DPUF1, NULL, cmucal_vclk_ip_ssmt_d0_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_DPUF1, NULL, cmucal_vclk_ip_ssmt_d1_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_DPUF1_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_dpuf1_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_EH_CMU_EH, NULL, cmucal_vclk_ip_eh_cmu_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_P_SYSMMU_S1_NS_EH, NULL, cmucal_vclk_ip_as_p_sysmmu_s1_ns_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_EH, NULL, cmucal_vclk_ip_d_tzpc_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_EH, NULL, cmucal_vclk_ip_gpc_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_EH_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_eh_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_LD_EH_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_si_ld_eh_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_EH, NULL, cmucal_vclk_ip_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_EH, NULL, cmucal_vclk_ip_ssmt_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_EH, NULL, cmucal_vclk_ip_ppmu_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_EH, NULL, cmucal_vclk_ip_sysmmu_s0_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_EH, NULL, cmucal_vclk_ip_sysreg_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_EH, NULL, cmucal_vclk_ip_uasc_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_EH, NULL, cmucal_vclk_ip_qe_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_EH, NULL, cmucal_vclk_ip_slh_axi_mi_p_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_EH_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_eh_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_EH, NULL, cmucal_vclk_ip_lh_axi_si_ip_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_EH, NULL, cmucal_vclk_ip_lh_axi_mi_ip_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_EH, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_EH, NULL, cmucal_vclk_ip_xiu_d_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_EH_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_eh_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EH_EVENT, NULL, cmucal_vclk_ip_ppc_eh_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EH_CYCLE, NULL, cmucal_vclk_ip_ppc_eh_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_G2D_CMU_G2D, NULL, cmucal_vclk_ip_g2d_cmu_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_G2D, NULL, cmucal_vclk_ip_ppmu_d0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_G2D, NULL, cmucal_vclk_ip_ppmu_d1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_G2D, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_G2D, NULL, cmucal_vclk_ip_sysreg_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D0_G2D, NULL, cmucal_vclk_ip_lh_axi_si_d0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D1_G2D, NULL, cmucal_vclk_ip_lh_axi_si_d1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU2_G2D, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_G2D, NULL, cmucal_vclk_ip_ppmu_d2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D2_G2D, NULL, cmucal_vclk_ip_lh_acel_si_d2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_G2D, NULL, cmucal_vclk_ip_ssmt_d0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_G2D, NULL, cmucal_vclk_ip_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_G2D, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_JPEG, NULL, cmucal_vclk_ip_jpeg, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_G2D, NULL, cmucal_vclk_ip_d_tzpc_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_G2D, NULL, cmucal_vclk_ip_ssmt_d1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_G2D, NULL, cmucal_vclk_ip_ssmt_d2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_G2D, NULL, cmucal_vclk_ip_gpc_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_G2D, NULL, cmucal_vclk_ip_slh_axi_mi_p_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_G2D, NULL, cmucal_vclk_ip_as_apb_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_JPEG, NULL, cmucal_vclk_ip_as_apb_jpeg, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_G2D, NULL, cmucal_vclk_ip_sysmmu_s0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_G2D, NULL, cmucal_vclk_ip_xiu_d_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_ID_G2D0_JPEG, NULL, cmucal_vclk_ip_lh_ast_si_id_g2d0_jpeg, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_ID_G2D0_JPEG, NULL, cmucal_vclk_ip_lh_ast_mi_id_g2d0_jpeg, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_ID_G2D1_JPEG, NULL, cmucal_vclk_ip_lh_ast_si_id_g2d1_jpeg, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_ID_G2D1_JPEG, NULL, cmucal_vclk_ip_lh_ast_mi_id_g2d1_jpeg, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_ID_JPEG_G2D0, NULL, cmucal_vclk_ip_lh_ast_si_id_jpeg_g2d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_ID_JPEG_G2D0, NULL, cmucal_vclk_ip_lh_ast_mi_id_jpeg_g2d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_ID_JPEG_G2D1, NULL, cmucal_vclk_ip_lh_ast_si_id_jpeg_g2d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_ID_JPEG_G2D1, NULL, cmucal_vclk_ip_lh_ast_mi_id_jpeg_g2d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_G2D_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_g2d_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_G3D_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_g3d_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_G3D, NULL, cmucal_vclk_ip_sysreg_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_G3D_CMU_G3D, NULL, cmucal_vclk_ip_g3d_cmu_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_G3D, NULL, cmucal_vclk_ip_lh_axi_si_ip_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPU, NULL, cmucal_vclk_ip_gpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GRAY2BIN_G3D, NULL, cmucal_vclk_ip_gray2bin_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_G3D, NULL, cmucal_vclk_ip_d_tzpc_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_G3D, NULL, cmucal_vclk_ip_gpc_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_G3D, NULL, cmucal_vclk_ip_uasc_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD_APBIF_G3D, NULL, cmucal_vclk_ip_add_apbif_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD_G3D, NULL, cmucal_vclk_ip_add_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_G3D, NULL, cmucal_vclk_ip_slh_axi_mi_p_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_G3D_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_g3d_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_G3D_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_si_lt_g3d_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPCFW_G3D0, NULL, cmucal_vclk_ip_ppcfw_g3d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_G3D_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_g3d_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G3D0, NULL, cmucal_vclk_ip_ssmt_g3d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADM_DAP_G_GPU, NULL, cmucal_vclk_ip_adm_dap_g_gpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DAPAHBAP_GPU, NULL, cmucal_vclk_ip_dapahbap_gpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_SYSMMU_G3D, NULL, cmucal_vclk_ip_ad_apb_sysmmu_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPCFW_G3D1, NULL, cmucal_vclk_ip_ppcfw_g3d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_G3D_D0, NULL, cmucal_vclk_ip_ppmu_g3d_d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D0_G3D, NULL, cmucal_vclk_ip_lh_acel_si_d0_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D1_G3D, NULL, cmucal_vclk_ip_lh_acel_si_d1_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D2_G3D, NULL, cmucal_vclk_ip_lh_acel_si_d2_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D3_G3D, NULL, cmucal_vclk_ip_lh_acel_si_d3_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_G3D, NULL, cmucal_vclk_ip_lh_axi_mi_ip_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_G3D_D1, NULL, cmucal_vclk_ip_ppmu_g3d_d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_G3D_D2, NULL, cmucal_vclk_ip_ppmu_g3d_d2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_G3D_D3, NULL, cmucal_vclk_ip_ppmu_g3d_d3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_D_G3DMMU, NULL, cmucal_vclk_ip_slh_axi_si_d_g3dmmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G3D1, NULL, cmucal_vclk_ip_ssmt_g3d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G3D2, NULL, cmucal_vclk_ip_ssmt_g3d2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G3D3, NULL, cmucal_vclk_ip_ssmt_g3d3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_G3D, NULL, cmucal_vclk_ip_sysmmu_s0_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_G3D, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_G3D, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU2_G3D, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu2_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU3_G3D, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu3_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GDC_CMU_GDC, NULL, cmucal_vclk_ip_gdc_cmu_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_GDC1, NULL, cmucal_vclk_ip_ad_apb_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_GDC, NULL, cmucal_vclk_ip_d_tzpc_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GDC1, NULL, cmucal_vclk_ip_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_GDC, NULL, cmucal_vclk_ip_gpc_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_GDC0, NULL, cmucal_vclk_ip_ppmu_d0_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_GDC1, NULL, cmucal_vclk_ip_ppmu_d0_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D_LME, NULL, cmucal_vclk_ip_ssmt_d_lme, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_GDC1, NULL, cmucal_vclk_ip_ssmt_d0_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_GDC0, NULL, cmucal_vclk_ip_ssmt_d0_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_GDC, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_GDC, NULL, cmucal_vclk_ip_sysreg_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_ID_GDC0_GDC1, NULL, cmucal_vclk_ip_lh_ast_mi_id_gdc0_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_ID_GDC1_GDC0, NULL, cmucal_vclk_ip_lh_ast_mi_id_gdc1_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_ID_GDC1_LME, NULL, cmucal_vclk_ip_lh_ast_mi_id_gdc1_lme, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_ID_LME_GDC1, NULL, cmucal_vclk_ip_lh_ast_si_id_lme_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_ID_GDC1_LME, NULL, cmucal_vclk_ip_lh_ast_si_id_gdc1_lme, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU2_GDC, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu2_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_GDC, NULL, cmucal_vclk_ip_sysmmu_s0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D2_GDC0, NULL, cmucal_vclk_ip_qe_d2_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_GDC0, NULL, cmucal_vclk_ip_qe_d0_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_GDC, NULL, cmucal_vclk_ip_slh_axi_mi_p_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D1_GDC, NULL, cmucal_vclk_ip_lh_axi_si_d1_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D_LME, NULL, cmucal_vclk_ip_ppmu_d_lme, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_GDC, NULL, cmucal_vclk_ip_xiu_d1_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D4_GDC1, NULL, cmucal_vclk_ip_ppmu_d4_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_GDC1, NULL, cmucal_vclk_ip_ppmu_d2_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_GDC0, NULL, cmucal_vclk_ip_ppmu_d2_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D4_GDC0, NULL, cmucal_vclk_ip_ppmu_d4_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D4_GDC1, NULL, cmucal_vclk_ip_ssmt_d4_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_GDC1, NULL, cmucal_vclk_ip_ssmt_d2_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_GDC0, NULL, cmucal_vclk_ip_ssmt_d2_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D4_GDC0, NULL, cmucal_vclk_ip_ssmt_d4_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_GDC1, NULL, cmucal_vclk_ip_qe_d0_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D2_GDC1, NULL, cmucal_vclk_ip_qe_d2_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D4_GDC0, NULL, cmucal_vclk_ip_qe_d4_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D4_GDC1, NULL, cmucal_vclk_ip_qe_d4_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_ID_LME_GDC1, NULL, cmucal_vclk_ip_lh_ast_mi_id_lme_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_ID_GDC0_GDC1, NULL, cmucal_vclk_ip_lh_ast_si_id_gdc0_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_GDC0, NULL, cmucal_vclk_ip_ad_apb_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_LME, NULL, cmucal_vclk_ip_ad_apb_lme, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_GDC, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D0_GDC, NULL, cmucal_vclk_ip_lh_axi_si_d0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D2_GDC, NULL, cmucal_vclk_ip_lh_axi_si_d2_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GDC0, NULL, cmucal_vclk_ip_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_GDC, NULL, cmucal_vclk_ip_xiu_d0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D2_GDC, NULL, cmucal_vclk_ip_xiu_d2_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_ID_GDC1_GDC0, NULL, cmucal_vclk_ip_lh_ast_si_id_gdc1_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LD_RGBP_GDC, NULL, cmucal_vclk_ip_lh_axi_mi_ld_rgbp_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_GDC_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_gdc_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LME, NULL, cmucal_vclk_ip_lme, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GSACORE_CMU_GSACORE, NULL, cmucal_vclk_ip_gsacore_cmu_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CA32_GSACORE, NULL, cmucal_vclk_ip_ca32_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_GSACORE0, NULL, cmucal_vclk_ip_gpio_gsacore0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_KDN_GSACORE, NULL, cmucal_vclk_ip_kdn_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_OTP_CON_GSACORE, NULL, cmucal_vclk_ip_otp_con_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_GSACORE0, NULL, cmucal_vclk_ip_ppmu_gsacore0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_CA32_GSACORE, NULL, cmucal_vclk_ip_qe_ca32_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_DMA_GSACORE, NULL, cmucal_vclk_ip_qe_dma_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_SC_GSACORE, NULL, cmucal_vclk_ip_qe_sc_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_RESETMON_GSACORE, NULL, cmucal_vclk_ip_resetmon_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SPI_FPS_GSACORE, NULL, cmucal_vclk_ip_spi_fps_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SPI_GSC_GSACORE, NULL, cmucal_vclk_ip_spi_gsc_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SC_GSACORE, NULL, cmucal_vclk_ip_sc_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_GSACORE, NULL, cmucal_vclk_ip_sysreg_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UART_GSACORE, NULL, cmucal_vclk_ip_uart_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_WDT_GSACORE, NULL, cmucal_vclk_ip_wdt_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BAAW_GSACORE, NULL, cmucal_vclk_ip_baaw_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_INTMEM_GSACORE, NULL, cmucal_vclk_ip_intmem_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_GSA, NULL, cmucal_vclk_ip_lh_axi_si_ip_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DMA_GSACORE, NULL, cmucal_vclk_ip_dma_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_DMA_GSACORE_NS, NULL, cmucal_vclk_ip_ad_apb_dma_gsacore_ns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PUF_GSACORE, NULL, cmucal_vclk_ip_puf_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_DP0_GSA_ZM, NULL, cmucal_vclk_ip_xiu_dp0_gsa_zm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_I_DAP_GSA, NULL, cmucal_vclk_ip_lh_axi_mi_i_dap_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_I_CA32_GIC, NULL, cmucal_vclk_ip_lh_ast_mi_i_ca32_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_I_GIC_CA32, NULL, cmucal_vclk_ip_lh_ast_mi_i_gic_ca32, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GIC_GSACORE, NULL, cmucal_vclk_ip_gic_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_I_GIC_CA32, NULL, cmucal_vclk_ip_lh_ast_si_i_gic_ca32, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_I_CA32_GIC, NULL, cmucal_vclk_ip_lh_ast_si_i_ca32_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_GSA_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_atb_si_lt_gsa_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT_GSA_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_si_lt_gsa_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT_GSA_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_atb_mi_lt_gsa_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_AXI2APB1_GSACORE, NULL, cmucal_vclk_ip_lh_axi_si_ip_axi2apb1_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_AXI2APB1_GSACORE, NULL, cmucal_vclk_ip_lh_axi_mi_ip_axi2apb1_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_AXI2APB2_GSACORE, NULL, cmucal_vclk_ip_lh_axi_si_ip_axi2apb2_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_AXI2APB2_GSACORE, NULL, cmucal_vclk_ip_lh_axi_mi_ip_axi2apb2_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_INTMEM_GSACORE, NULL, cmucal_vclk_ip_ad_apb_intmem_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_GSACORE1, NULL, cmucal_vclk_ip_gpio_gsacore1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_GSACORE2, NULL, cmucal_vclk_ip_gpio_gsacore2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_GSACORE3, NULL, cmucal_vclk_ip_gpio_gsacore3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_ID_GME_GSA, NULL, cmucal_vclk_ip_lh_axi_si_id_gme_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UGME, NULL, cmucal_vclk_ip_ugme, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_ID_SC_GSACORE, NULL, cmucal_vclk_ip_lh_axi_si_id_sc_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_ID_SC_GSACORE, NULL, cmucal_vclk_ip_lh_axi_mi_id_sc_gsacore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_GSACORE1, NULL, cmucal_vclk_ip_ppmu_gsacore1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_GSA_ZM, NULL, cmucal_vclk_ip_xiu_d0_gsa_zm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GSACTRL_CMU_GSACTRL, NULL, cmucal_vclk_ip_gsactrl_cmu_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_GSACTRL, NULL, cmucal_vclk_ip_gpc_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_GSA2AOC, NULL, cmucal_vclk_ip_mailbox_gsa2aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_GSA2NONTZ, NULL, cmucal_vclk_ip_mailbox_gsa2nontz, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_GSA2TPU, NULL, cmucal_vclk_ip_mailbox_gsa2tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_GSA2AUR, NULL, cmucal_vclk_ip_mailbox_gsa2aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_GSACTRL, NULL, cmucal_vclk_ip_sysreg_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TZPC_GSACTRL, NULL, cmucal_vclk_ip_tzpc_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_INTMEM_GSACTRL, NULL, cmucal_vclk_ip_intmem_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_GSA, NULL, cmucal_vclk_ip_lh_axi_mi_ip_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_GSA2TZ, NULL, cmucal_vclk_ip_mailbox_gsa2tz, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PMU_GSA, NULL, cmucal_vclk_ip_pmu_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_GPIO_GSACTRL, NULL, cmucal_vclk_ip_apbif_gpio_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TIMER_GSACTRL, NULL, cmucal_vclk_ip_timer_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DAP_GSACTRL, NULL, cmucal_vclk_ip_dap_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_GSA_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_gsa_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_GSACTRLEXT, NULL, cmucal_vclk_ip_sysreg_gsactrlext, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SECJTAG_GSACTRL, NULL, cmucal_vclk_ip_secjtag_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_I_DAP_GSA, NULL, cmucal_vclk_ip_lh_axi_si_i_dap_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_INTMEM_GSACTRL, NULL, cmucal_vclk_ip_ad_apb_intmem_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_GSA, NULL, cmucal_vclk_ip_slh_axi_mi_p_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_GSA_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_gsa_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_AXI2APB0_GSACTRL, NULL, cmucal_vclk_ip_lh_axi_si_ip_axi2apb0_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_AXI2APB0_GSACTRL, NULL, cmucal_vclk_ip_lh_axi_mi_ip_axi2apb0_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_DP1_GSA_ZM, NULL, cmucal_vclk_ip_xiu_dp1_gsa_zm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_GSA_ZM, NULL, cmucal_vclk_ip_xiu_d1_gsa_zm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_ID_GME_GSA, NULL, cmucal_vclk_ip_lh_axi_mi_id_gme_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D_GSA, NULL, cmucal_vclk_ip_lh_axi_si_d_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_SYSMMU_GSA_S1_NS, NULL, cmucal_vclk_ip_ad_apb_sysmmu_gsa_s1_ns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_GSACTRL, NULL, cmucal_vclk_ip_ssmt_gsactrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_GSA_ZM, NULL, cmucal_vclk_ip_sysmmu_s0_gsa_zm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_GSA_ZM, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_gsa_zm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GSE_CMU_GSE, NULL, cmucal_vclk_ip_gse_cmu_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_GSE, NULL, cmucal_vclk_ip_d_tzpc_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_GSE, NULL, cmucal_vclk_ip_slh_axi_mi_p_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_GSE, NULL, cmucal_vclk_ip_sysreg_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_GSE, NULL, cmucal_vclk_ip_ad_apb_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D_GSE, NULL, cmucal_vclk_ip_lh_axi_si_d_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_GSE, NULL, cmucal_vclk_ip_ppmu_d0_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GSE, NULL, cmucal_vclk_ip_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_GSE, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_GSE, NULL, cmucal_vclk_ip_gpc_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_GSE, NULL, cmucal_vclk_ip_sysmmu_s0_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_GSE, NULL, cmucal_vclk_ip_ppmu_d1_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_GSE, NULL, cmucal_vclk_ip_qe_d0_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D1_GSE, NULL, cmucal_vclk_ip_qe_d1_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_GSE, NULL, cmucal_vclk_ip_ssmt_d0_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_GSE, NULL, cmucal_vclk_ip_ssmt_d1_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_GSE, NULL, cmucal_vclk_ip_xiu_d1_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_OTF_YUVP_GSE, NULL, cmucal_vclk_ip_lh_ast_mi_l_otf_yuvp_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_OTF_TNR_GSE, NULL, cmucal_vclk_ip_lh_ast_mi_l_otf_tnr_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_GSE, NULL, cmucal_vclk_ip_xiu_d0_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_GSE, NULL, cmucal_vclk_ip_ssmt_d2_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D2_GSE, NULL, cmucal_vclk_ip_qe_d2_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_GSE, NULL, cmucal_vclk_ip_ppmu_d2_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_GSE_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_gse_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AXI_US_128TO256_QE_D2_GSE, NULL, cmucal_vclk_ip_axi_us_128to256_qe_d2_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HSI0_CMU_HSI0, NULL, cmucal_vclk_ip_hsi0_cmu_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USB32DRD, NULL, cmucal_vclk_ip_usb32drd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DP_LINK, NULL, cmucal_vclk_ip_dp_link, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_HSI0, NULL, cmucal_vclk_ip_xiu_d0_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ETR_MIU, NULL, cmucal_vclk_ip_etr_miu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_HSI0, NULL, cmucal_vclk_ip_ppmu_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LD_HSI0_AOC, NULL, cmucal_vclk_ip_lh_axi_si_ld_hsi0_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D_HSI0, NULL, cmucal_vclk_ip_lh_acel_si_d_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_HSI0, NULL, cmucal_vclk_ip_gpc_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_HSI0, NULL, cmucal_vclk_ip_d_tzpc_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_HSI0, NULL, cmucal_vclk_ip_ssmt_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_HSI0, NULL, cmucal_vclk_ip_sysreg_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P_HSI0, NULL, cmucal_vclk_ip_xiu_p_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D2_HSI0, NULL, cmucal_vclk_ip_xiu_d2_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_HSI0_LINK, NULL, cmucal_vclk_ip_uasc_hsi0_link, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_HSI0, NULL, cmucal_vclk_ip_sysmmu_s0_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_LG_ETR_HSI0, NULL, cmucal_vclk_ip_slh_axi_mi_lg_etr_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_LP_AOC_HSI0, NULL, cmucal_vclk_ip_slh_axi_mi_lp_aoc_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_HSI0, NULL, cmucal_vclk_ip_slh_axi_mi_p_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LG_ETR_HSI0_CU, NULL, cmucal_vclk_ip_lh_axi_si_lg_etr_hsi0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LG_ETR_HSI0_CU, NULL, cmucal_vclk_ip_lh_axi_mi_lg_etr_hsi0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_AOC_HSI0_CU, NULL, cmucal_vclk_ip_lh_axi_si_lp_aoc_hsi0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_AOC_HSI0_CU, NULL, cmucal_vclk_ip_lh_axi_mi_lp_aoc_hsi0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_HSI0_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_hsi0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_HSI0_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_hsi0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_HSI0, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI0_HSI0, NULL, cmucal_vclk_ip_usi0_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI1_HSI0, NULL, cmucal_vclk_ip_usi1_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI2_HSI0, NULL, cmucal_vclk_ip_usi2_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI3_HSI0, NULL, cmucal_vclk_ip_usi3_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_EUSBPHY_HSI0, NULL, cmucal_vclk_ip_ad_apb_eusbphy_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_HSI0_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_hsi0_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI4_HSI0, NULL, cmucal_vclk_ip_usi4_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_I3C2_HSI0, NULL, cmucal_vclk_ip_i3c2_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_I3C3_HSI0, NULL, cmucal_vclk_ip_i3c3_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HSI1_CMU_HSI1, NULL, cmucal_vclk_ip_hsi1_cmu_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D_HSI1, NULL, cmucal_vclk_ip_lh_acel_si_d_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_HSI1_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_hsi1_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_HSI1, NULL, cmucal_vclk_ip_sysreg_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_HSI1, NULL, cmucal_vclk_ip_ppmu_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P_HSI1, NULL, cmucal_vclk_ip_xiu_p_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_GEN3_0, NULL, cmucal_vclk_ip_pcie_gen3_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_IA_GEN3A_0, NULL, cmucal_vclk_ip_pcie_ia_gen3a_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_HSI1, NULL, cmucal_vclk_ip_d_tzpc_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_HSI1, NULL, cmucal_vclk_ip_gpc_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_HSI1, NULL, cmucal_vclk_ip_ssmt_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_HSI1, NULL, cmucal_vclk_ip_gpio_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN3A_DBI_0, NULL, cmucal_vclk_ip_uasc_pcie_gen3a_dbi_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN3A_SLV_0, NULL, cmucal_vclk_ip_uasc_pcie_gen3a_slv_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PCIE_IA_GEN3A_0, NULL, cmucal_vclk_ip_ssmt_pcie_ia_gen3a_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_PCIEPHY_HSI1, NULL, cmucal_vclk_ip_as_apb_pciephy_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_HSI1, NULL, cmucal_vclk_ip_sysmmu_s0_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_HSI1, NULL, cmucal_vclk_ip_slh_axi_mi_p_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_HSI1_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_hsi1_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_CPUCL0_HSI1, NULL, cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_LP_AOC_HSI1, NULL, cmucal_vclk_ip_slh_axi_mi_lp_aoc_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_CPUCL0_HSI1_CU, NULL, cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi1_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_AOC_HSI1_CU, NULL, cmucal_vclk_ip_lh_axi_si_lp_aoc_hsi1_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_CPUCL0_HSI1_CU, NULL, cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi1_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_AOC_HSI1_CU, NULL, cmucal_vclk_ip_lh_axi_mi_lp_aoc_hsi1_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LD_HSI1_AOC, NULL, cmucal_vclk_ip_lh_axi_si_ld_hsi1_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_HSI1, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_HSI1, NULL, cmucal_vclk_ip_xiu_d1_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_HSI1_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_hsi1_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HSI2_CMU_HSI2, NULL, cmucal_vclk_ip_hsi2_cmu_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_HSI2, NULL, cmucal_vclk_ip_sysreg_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_HSI2, NULL, cmucal_vclk_ip_gpio_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D_HSI2, NULL, cmucal_vclk_ip_lh_acel_si_d_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_HSI2_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_hsi2_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_HSI2, NULL, cmucal_vclk_ip_xiu_d0_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P_HSI2, NULL, cmucal_vclk_ip_xiu_p_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_HSI2, NULL, cmucal_vclk_ip_ppmu_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_GEN3A_1, NULL, cmucal_vclk_ip_pcie_gen3a_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_HSI2, NULL, cmucal_vclk_ip_ssmt_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_IA_GEN3A_1, NULL, cmucal_vclk_ip_pcie_ia_gen3a_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_HSI2, NULL, cmucal_vclk_ip_d_tzpc_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UFS_EMBD, NULL, cmucal_vclk_ip_ufs_embd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_IA_GEN3B_1, NULL, cmucal_vclk_ip_pcie_ia_gen3b_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_HSI2, NULL, cmucal_vclk_ip_gpc_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MMC_CARD, NULL, cmucal_vclk_ip_mmc_card, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PCIE_GEN3A_HSI2, NULL, cmucal_vclk_ip_qe_pcie_gen3a_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PCIE_GEN3B_HSI2, NULL, cmucal_vclk_ip_qe_pcie_gen3b_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_UFS_EMBD_HSI2, NULL, cmucal_vclk_ip_qe_ufs_embd_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN3A_DBI_1, NULL, cmucal_vclk_ip_uasc_pcie_gen3a_dbi_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN3A_SLV_1, NULL, cmucal_vclk_ip_uasc_pcie_gen3a_slv_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN3B_DBI_1, NULL, cmucal_vclk_ip_uasc_pcie_gen3b_dbi_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN3B_SLV_1, NULL, cmucal_vclk_ip_uasc_pcie_gen3b_slv_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_MMC_CARD_HSI2, NULL, cmucal_vclk_ip_qe_mmc_card_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PCIE_IA_GEN3A_1, NULL, cmucal_vclk_ip_ssmt_pcie_ia_gen3a_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PCIE_IA_GEN3B_1, NULL, cmucal_vclk_ip_ssmt_pcie_ia_gen3b_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_HSI2UFS, NULL, cmucal_vclk_ip_gpio_hsi2ufs, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_HSI2, NULL, cmucal_vclk_ip_sysmmu_s0_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_HSI2, NULL, cmucal_vclk_ip_slh_axi_mi_p_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_HSI2_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_hsi2_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_HSI2, NULL, cmucal_vclk_ip_xiu_d1_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_CPUCL0_HSI2, NULL, cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LP_CPUCL0_HSI2_CU, NULL, cmucal_vclk_ip_lh_axi_si_lp_cpucl0_hsi2_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_HSI2, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_GEN3B_1, NULL, cmucal_vclk_ip_pcie_gen3b_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LP_CPUCL0_HSI2_CU, NULL, cmucal_vclk_ip_lh_axi_mi_lp_cpucl0_hsi2_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_PCIEPHY_0_HSI2, NULL, cmucal_vclk_ip_as_apb_pciephy_0_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_HSI2_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_hsi2_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D0_ISPFE, NULL, cmucal_vclk_ip_lh_axi_si_d0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_ISPFE, NULL, cmucal_vclk_ip_slh_axi_mi_p_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_ISPFE, NULL, cmucal_vclk_ip_sysreg_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ISPFE_CMU_ISPFE, NULL, cmucal_vclk_ip_ispfe_cmu_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MIPI_PHY_LINK_WRAP, NULL, cmucal_vclk_ip_mipi_phy_link_wrap, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_ISPFE, NULL, cmucal_vclk_ip_d_tzpc_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_ISPFE, NULL, cmucal_vclk_ip_ppmu_d0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D3_ISPFE, NULL, cmucal_vclk_ip_lh_axi_si_d3_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_ISPFE, NULL, cmucal_vclk_ip_gpc_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_SYSMMU_S0_ISPFE_S1_NS, NULL, cmucal_vclk_ip_ad_apb_sysmmu_s0_ispfe_s1_ns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_ISPFE, NULL, cmucal_vclk_ip_ppmu_d1_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_PMMU0_ISPFE, NULL, cmucal_vclk_ip_sysmmu_s1_pmmu0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_ISPFE, NULL, cmucal_vclk_ip_ssmt_d1_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_ISPFE, NULL, cmucal_vclk_ip_ssmt_d0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_ISPFE, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_ISPFE, NULL, cmucal_vclk_ip_sysmmu_s0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S2_PMMU0_ISPFE, NULL, cmucal_vclk_ip_sysmmu_s2_pmmu0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_ISPFE, NULL, cmucal_vclk_ip_xiu_d0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D1_ISPFE, NULL, cmucal_vclk_ip_lh_axi_si_d1_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_ISPFE, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D2_ISPFE, NULL, cmucal_vclk_ip_lh_axi_si_d2_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ISPFE, NULL, cmucal_vclk_ip_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_ISPFE, NULL, cmucal_vclk_ip_qe_d0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D1_ISPFE, NULL, cmucal_vclk_ip_qe_d1_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D3_ISPFE, NULL, cmucal_vclk_ip_qe_d3_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D2_ISPFE, NULL, cmucal_vclk_ip_qe_d2_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_ISPFE, NULL, cmucal_vclk_ip_ssmt_d2_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D3_ISPFE, NULL, cmucal_vclk_ip_ssmt_d3_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_ISPFE, NULL, cmucal_vclk_ip_ppmu_d2_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D3_ISPFE, NULL, cmucal_vclk_ip_ppmu_d3_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_ISPFE, NULL, cmucal_vclk_ip_uasc_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_ISPFE, NULL, cmucal_vclk_ip_xiu_d1_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D2_ISPFE, NULL, cmucal_vclk_ip_xiu_d2_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_ISPFE, NULL, cmucal_vclk_ip_sysmmu_s1_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S2_ISPFE, NULL, cmucal_vclk_ip_sysmmu_s2_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_IP_ISPFE, NULL, cmucal_vclk_ip_lh_axi_si_ip_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_IP_ISPFE, NULL, cmucal_vclk_ip_lh_axi_mi_ip_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_MIPI_PHY, NULL, cmucal_vclk_ip_ad_apb_mipi_phy, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BAAW_ISPFE, NULL, cmucal_vclk_ip_baaw_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_ISPFE_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_ispfe_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_MCSC, NULL, cmucal_vclk_ip_slh_axi_mi_p_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D0_MCSC, NULL, cmucal_vclk_ip_lh_axi_si_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_MCSC, NULL, cmucal_vclk_ip_sysreg_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MCSC_CMU_MCSC, NULL, cmucal_vclk_ip_mcsc_cmu_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_OTF_YUVP_MCSC, NULL, cmucal_vclk_ip_lh_ast_mi_l_otf_yuvp_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_MCSC, NULL, cmucal_vclk_ip_d_tzpc_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_MCSC, NULL, cmucal_vclk_ip_gpc_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_MCSC, NULL, cmucal_vclk_ip_ssmt_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_MCSC, NULL, cmucal_vclk_ip_sysmmu_s0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D3_MCSC, NULL, cmucal_vclk_ip_ppmu_d3_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_MCSC, NULL, cmucal_vclk_ip_ssmt_d2_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_MCSC, NULL, cmucal_vclk_ip_ppmu_d2_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_MCSC, NULL, cmucal_vclk_ip_ppmu_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_OTF_TNR_MCSC, NULL, cmucal_vclk_ip_lh_ast_mi_l_otf_tnr_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D3_MCSC, NULL, cmucal_vclk_ip_ssmt_d3_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_MCSC, NULL, cmucal_vclk_ip_ppmu_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_MCSC, NULL, cmucal_vclk_ip_ssmt_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D6_MCSC, NULL, cmucal_vclk_ip_qe_d6_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_MCSC, NULL, cmucal_vclk_ip_qe_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D1_MCSC, NULL, cmucal_vclk_ip_qe_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D2_MCSC, NULL, cmucal_vclk_ip_qe_d2_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D4_MCSC, NULL, cmucal_vclk_ip_qe_d4_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D3_MCSC, NULL, cmucal_vclk_ip_qe_d3_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D5_MCSC, NULL, cmucal_vclk_ip_qe_d5_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_MCSC, NULL, cmucal_vclk_ip_ad_apb_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D4_MCSC, NULL, cmucal_vclk_ip_ppmu_d4_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D5_MCSC, NULL, cmucal_vclk_ip_ppmu_d5_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D6_MCSC, NULL, cmucal_vclk_ip_ppmu_d6_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_MCSC, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D4_MCSC, NULL, cmucal_vclk_ip_ssmt_d4_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D5_MCSC, NULL, cmucal_vclk_ip_ssmt_d5_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D6_MCSC, NULL, cmucal_vclk_ip_ssmt_d6_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MCSC, NULL, cmucal_vclk_ip_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D1_MCSC, NULL, cmucal_vclk_ip_lh_axi_si_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_MCSC, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_MCSC, NULL, cmucal_vclk_ip_xiu_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D2_MCSC, NULL, cmucal_vclk_ip_xiu_d2_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_MCSC_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_mcsc_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_MCSC, NULL, cmucal_vclk_ip_xiu_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MFC_CMU_MFC, NULL, cmucal_vclk_ip_mfc_cmu_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_MFC, NULL, cmucal_vclk_ip_as_apb_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_MFC, NULL, cmucal_vclk_ip_sysreg_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D0_MFC, NULL, cmucal_vclk_ip_lh_axi_si_d0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D1_MFC, NULL, cmucal_vclk_ip_lh_axi_si_d1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_MFC, NULL, cmucal_vclk_ip_slh_axi_mi_p_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_MFC, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_MFC, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_MFC, NULL, cmucal_vclk_ip_ppmu_d0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_MFC, NULL, cmucal_vclk_ip_ppmu_d1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_MFC, NULL, cmucal_vclk_ip_ssmt_d0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MFC, NULL, cmucal_vclk_ip_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_MFC, NULL, cmucal_vclk_ip_d_tzpc_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_MFC, NULL, cmucal_vclk_ip_ssmt_d1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_MFC, NULL, cmucal_vclk_ip_gpc_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_MFC, NULL, cmucal_vclk_ip_sysmmu_s0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_MFC, NULL, cmucal_vclk_ip_xiu_d_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_MFC_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_mfc_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MIF_CMU_MIF, NULL, cmucal_vclk_ip_mif_cmu_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_MIF, NULL, cmucal_vclk_ip_sysreg_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_MIF_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_mif_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AXI2APB_P_MIF, NULL, cmucal_vclk_ip_axi2apb_p_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QCH_ADAPTER_PPC_DEBUG, NULL, cmucal_vclk_ip_qch_adapter_ppc_debug, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_MIF, NULL, cmucal_vclk_ip_gpc_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_MIF, NULL, cmucal_vclk_ip_d_tzpc_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_DEBUG, NULL, cmucal_vclk_ip_ppc_debug, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GEN_WREN_SECURE, NULL, cmucal_vclk_ip_gen_wren_secure, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_MIF, NULL, cmucal_vclk_ip_slh_axi_mi_p_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_MIF_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_mif_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QCH_ADAPTER_DDRPHY, NULL, cmucal_vclk_ip_qch_adapter_ddrphy, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QCH_ADAPTER_SMC, NULL, cmucal_vclk_ip_qch_adapter_smc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_MIF_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_mif_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_MISC, NULL, cmucal_vclk_ip_sysreg_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_WDT_CLUSTER1, NULL, cmucal_vclk_ip_wdt_cluster1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_WDT_CLUSTER0, NULL, cmucal_vclk_ip_wdt_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_OTP_CON_BIRA, NULL, cmucal_vclk_ip_otp_con_bira, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GIC, NULL, cmucal_vclk_ip_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MCT, NULL, cmucal_vclk_ip_mct, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_OTP_CON_TOP, NULL, cmucal_vclk_ip_otp_con_top, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_MISC, NULL, cmucal_vclk_ip_d_tzpc_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TMU_SUB, NULL, cmucal_vclk_ip_tmu_sub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TMU_TOP, NULL, cmucal_vclk_ip_tmu_top, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DIT, NULL, cmucal_vclk_ip_dit, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_MISC_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_misc_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D_MISC, NULL, cmucal_vclk_ip_lh_acel_si_d_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PDMA0, NULL, cmucal_vclk_ip_pdma0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_MISC, NULL, cmucal_vclk_ip_ppmu_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_DIT, NULL, cmucal_vclk_ip_qe_dit, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PDMA0, NULL, cmucal_vclk_ip_qe_pdma0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MISC_CMU_MISC, NULL, cmucal_vclk_ip_misc_cmu_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_RTIC, NULL, cmucal_vclk_ip_qe_rtic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_SPDMA0, NULL, cmucal_vclk_ip_qe_spdma0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_SC, NULL, cmucal_vclk_ip_qe_sc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_RTIC, NULL, cmucal_vclk_ip_rtic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SPDMA0, NULL, cmucal_vclk_ip_spdma0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SC, NULL, cmucal_vclk_ip_sc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_SC, NULL, cmucal_vclk_ip_ssmt_sc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_MISC, NULL, cmucal_vclk_ip_gpc_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_DIT, NULL, cmucal_vclk_ip_ad_apb_dit, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_PUF, NULL, cmucal_vclk_ip_ad_apb_puf, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_ICC_CLUSTER0_GIC_CU, NULL, cmucal_vclk_ip_lh_ast_mi_l_icc_cluster0_gic_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_ID_SC, NULL, cmucal_vclk_ip_lh_axi_mi_id_sc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_IRI_GIC_CLUSTER0_CD, NULL, cmucal_vclk_ip_lh_ast_si_l_iri_gic_cluster0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_ID_SC, NULL, cmucal_vclk_ip_lh_axi_si_id_sc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PUF, NULL, cmucal_vclk_ip_puf, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_MISC, NULL, cmucal_vclk_ip_xiu_d0_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_MISC, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_MISC_GIC_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_misc_gic_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_RTIC, NULL, cmucal_vclk_ip_ssmt_rtic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_SPDMA0, NULL, cmucal_vclk_ip_ssmt_spdma0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PDMA0, NULL, cmucal_vclk_ip_ssmt_pdma0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_DIT, NULL, cmucal_vclk_ip_ssmt_dit, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_IRI_GIC_CLUSTER0_CD, NULL, cmucal_vclk_ip_lh_ast_mi_l_iri_gic_cluster0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_IRI_GIC_CLUSTER0, NULL, cmucal_vclk_ip_lh_ast_si_l_iri_gic_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_ICC_CLUSTER0_GIC, NULL, cmucal_vclk_ip_lh_ast_mi_l_icc_cluster0_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_ICC_CLUSTER0_GIC_CU, NULL, cmucal_vclk_ip_lh_ast_si_l_icc_cluster0_gic_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_MISC, NULL, cmucal_vclk_ip_slh_axi_mi_p_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_MISC_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_misc_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SPDMA1, NULL, cmucal_vclk_ip_spdma1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PDMA1, NULL, cmucal_vclk_ip_qe_pdma1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_SPDMA1, NULL, cmucal_vclk_ip_qe_spdma1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PDMA1, NULL, cmucal_vclk_ip_ssmt_pdma1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_SPDMA1, NULL, cmucal_vclk_ip_ssmt_spdma1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PDMA1, NULL, cmucal_vclk_ip_pdma1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_MISC_GIC, NULL, cmucal_vclk_ip_slh_axi_mi_p_misc_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_MISC_GIC_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_misc_gic_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_MISC, NULL, cmucal_vclk_ip_xiu_d1_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_MISC, NULL, cmucal_vclk_ip_sysmmu_s0_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_OTP_CON_BISR, NULL, cmucal_vclk_ip_otp_con_bisr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_MISC_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_misc_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MCT_SUB, NULL, cmucal_vclk_ip_mct_sub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MCT_V41, NULL, cmucal_vclk_ip_mct_v41, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_NOCL0, NULL, cmucal_vclk_ip_sysreg_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_P_NOCL0, NULL, cmucal_vclk_ip_trex_p_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D1_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_mi_d1_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_NOCL0, NULL, cmucal_vclk_ip_trex_d_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_NOCL0, NULL, cmucal_vclk_ip_d_tzpc_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BDU, NULL, cmucal_vclk_ip_bdu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_NOCL0, NULL, cmucal_vclk_ip_gpc_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL0_ALIVE_P, NULL, cmucal_vclk_ip_ppmu_nocl0_alive_p, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL0_CPUCL0_P, NULL, cmucal_vclk_ip_ppmu_nocl0_cpucl0_p, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SFR_APBIF_CMU_TOPC, NULL, cmucal_vclk_ip_sfr_apbif_cmu_topc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL1A_M0_EVENT, NULL, cmucal_vclk_ip_ppc_nocl1a_m0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL1A_M1_EVENT, NULL, cmucal_vclk_ip_ppc_nocl1a_m1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL1A_M2_EVENT, NULL, cmucal_vclk_ip_ppc_nocl1a_m2_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL1A_M3_EVENT, NULL, cmucal_vclk_ip_ppc_nocl1a_m3_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL1B_M0_EVENT, NULL, cmucal_vclk_ip_ppc_nocl1b_m0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CPUCL0_D0_CYCLE, NULL, cmucal_vclk_ip_ppc_cpucl0_d0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CB_TOP, NULL, cmucal_vclk_ip_slc_cb_top, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CPUCL0_D0_EVENT, NULL, cmucal_vclk_ip_ppc_cpucl0_d0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CPUCL0_D2_EVENT, NULL, cmucal_vclk_ip_ppc_cpucl0_d2_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CPUCL0_D3_EVENT, NULL, cmucal_vclk_ip_ppc_cpucl0_d3_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL1A_M0_CYCLE, NULL, cmucal_vclk_ip_ppc_nocl1a_m0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_DBG_CC, NULL, cmucal_vclk_ip_ppc_dbg_cc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MPACE_ASB_D0_MIF, NULL, cmucal_vclk_ip_mpace_asb_d0_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MPACE_ASB_D1_MIF, NULL, cmucal_vclk_ip_mpace_asb_d1_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MPACE_ASB_D2_MIF, NULL, cmucal_vclk_ip_mpace_asb_d2_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MPACE_ASB_D3_MIF, NULL, cmucal_vclk_ip_mpace_asb_d3_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CPUCL0_D1_EVENT, NULL, cmucal_vclk_ip_ppc_cpucl0_d1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CH_TOP, NULL, cmucal_vclk_ip_slc_ch_top, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GRAY2BIN_ATB_TSVALUE, NULL, cmucal_vclk_ip_gray2bin_atb_tsvalue, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_G_NOCL0, NULL, cmucal_vclk_ip_slh_axi_mi_g_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL1A_CU, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl1a_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL1B_CU, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl1b_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL2AA_CU, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl2aa_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_ALIVE, NULL, cmucal_vclk_ip_slh_axi_si_p_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_CPUCL0, NULL, cmucal_vclk_ip_slh_axi_si_p_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_EH, NULL, cmucal_vclk_ip_slh_axi_si_p_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_MISC_GIC, NULL, cmucal_vclk_ip_slh_axi_si_p_misc_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_MIF0, NULL, cmucal_vclk_ip_slh_axi_si_p_mif0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_MIF1, NULL, cmucal_vclk_ip_slh_axi_si_p_mif1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_MIF2, NULL, cmucal_vclk_ip_slh_axi_si_p_mif2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_MIF3, NULL, cmucal_vclk_ip_slh_axi_si_p_mif3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_MISC, NULL, cmucal_vclk_ip_slh_axi_si_p_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_PERIC0, NULL, cmucal_vclk_ip_slh_axi_si_p_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_PERIC1, NULL, cmucal_vclk_ip_slh_axi_si_p_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_T_BDU, NULL, cmucal_vclk_ip_lh_atb_si_t_bdu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_T_SLC, NULL, cmucal_vclk_ip_lh_atb_si_t_slc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_ALIVE_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_alive_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_MISC_GIC_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_misc_gic_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_MIF0_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_mif0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_MIF1_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_mif1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_MIF2_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_mif2_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_MIF3_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_mif3_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_MISC_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_misc_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_PERIC0_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_peric0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_PERIC1_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_peric1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_T_BDU_CD, NULL, cmucal_vclk_ip_lh_atb_si_t_bdu_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_T_SLC_CD, NULL, cmucal_vclk_ip_lh_atb_si_t_slc_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_ALIVE_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_alive_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_MISC_GIC_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_misc_gic_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_MIF0_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_mif0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_MIF1_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_mif1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_MIF2_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_mif2_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_MIF3_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_mif3_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_MISC_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_misc_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_PERIC0_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_peric0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_PERIC1_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_peric1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_T_BDU_CD, NULL, cmucal_vclk_ip_lh_atb_mi_t_bdu_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_T_SLC_CD, NULL, cmucal_vclk_ip_lh_atb_mi_t_slc_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL1A, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL1B, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL2AA, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL1A_CU, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl1a_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL1B_CU, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl1b_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL2AA_CU, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl2aa_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D2_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_mi_d2_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D3_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_mi_d3_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL0_IOC0, NULL, cmucal_vclk_ip_ppmu_nocl0_ioc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL0_IOC1, NULL, cmucal_vclk_ip_ppmu_nocl0_ioc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL0_S0, NULL, cmucal_vclk_ip_ppmu_nocl0_s0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL0_S1, NULL, cmucal_vclk_ip_ppmu_nocl0_s1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL0_S2, NULL, cmucal_vclk_ip_ppmu_nocl0_s2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL0_S3, NULL, cmucal_vclk_ip_ppmu_nocl0_s3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL2AB, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_NOCL0_CMU_NOCL0, NULL, cmucal_vclk_ip_nocl0_cmu_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL2AB_CU, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl2ab_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL2AB_CU, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl2ab_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D0_NOCL0_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_si_d0_nocl0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D1_NOCL0_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_si_d1_nocl0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_D0_NOCL1A_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_mi_d0_nocl1a_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_P_NOCL0_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_si_p_nocl0_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_D_NOCL1B_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_mi_d_nocl1b_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_D1_NOCL1A_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_mi_d1_nocl1a_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_D2_NOCL1A_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_mi_d2_nocl1a_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_D3_NOCL1A_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_mi_d3_nocl1a_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_P_NOCL0_NOCL1B, NULL, cmucal_vclk_ip_lh_taxi_si_p_nocl0_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_P_NOCL0_NOCL2AA, NULL, cmucal_vclk_ip_lh_taxi_si_p_nocl0_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_P_NOCL0_NOCL2AB, NULL, cmucal_vclk_ip_lh_taxi_si_p_nocl0_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL1B_M0_CYCLE, NULL, cmucal_vclk_ip_ppc_nocl1b_m0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL0_DP, NULL, cmucal_vclk_ip_ppmu_nocl0_dp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_EH_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_eh_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_EH_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_eh_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_CPUCL0_NOCL0, NULL, cmucal_vclk_ip_lh_axi_mi_p_cpucl0_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CH1, NULL, cmucal_vclk_ip_slc_ch1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CH2, NULL, cmucal_vclk_ip_slc_ch2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CH3, NULL, cmucal_vclk_ip_slc_ch3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_NOCL0_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_nocl0_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LD_SLC_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_ld_slc_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LD_SLC1_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_ld_slc1_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LD_SLC2_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_ld_slc2_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LD_SLC3_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_ld_slc3_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL0_IO0_CYCLE, NULL, cmucal_vclk_ip_ppc_nocl0_io0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL0_IO0_EVENT, NULL, cmucal_vclk_ip_ppc_nocl0_io0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL0_IO1_EVENT, NULL, cmucal_vclk_ip_ppc_nocl0_io1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D0_CPUCL0, NULL, cmucal_vclk_ip_lh_acel_mi_d0_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_NOCL1A, NULL, cmucal_vclk_ip_trex_d_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_NOCL1A, NULL, cmucal_vclk_ip_sysreg_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D0_G3D, NULL, cmucal_vclk_ip_lh_acel_mi_d0_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_NOCL1A, NULL, cmucal_vclk_ip_d_tzpc_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D1_G3D, NULL, cmucal_vclk_ip_lh_acel_mi_d1_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D2_G3D, NULL, cmucal_vclk_ip_lh_acel_mi_d2_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D3_G3D, NULL, cmucal_vclk_ip_lh_acel_mi_d3_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_G3D_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_g3d_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_NOCL1A, NULL, cmucal_vclk_ip_gpc_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_P_NOCL1A, NULL, cmucal_vclk_ip_trex_p_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL1A_CD, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl1a_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL2AA_S0_EVENT, NULL, cmucal_vclk_ip_ppc_nocl2aa_s0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL2AA_S1_EVENT, NULL, cmucal_vclk_ip_ppc_nocl2aa_s1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL2AB_S0_EVENT, NULL, cmucal_vclk_ip_ppc_nocl2ab_s0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL2AB_S1_EVENT, NULL, cmucal_vclk_ip_ppc_nocl2ab_s1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_G3D_D0_EVENT, NULL, cmucal_vclk_ip_ppc_g3d_d0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_G3D_D1_EVENT, NULL, cmucal_vclk_ip_ppc_g3d_d1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_G3D_D2_EVENT, NULL, cmucal_vclk_ip_ppc_g3d_d2_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_G3D_D3_EVENT, NULL, cmucal_vclk_ip_ppc_g3d_d3_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_TPU_D0_EVENT, NULL, cmucal_vclk_ip_ppc_tpu_d0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL2AA_S0_CYCLE, NULL, cmucal_vclk_ip_ppc_nocl2aa_s0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_G3D_D0_CYCLE, NULL, cmucal_vclk_ip_ppc_g3d_d0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_TPU_D0_CYCLE, NULL, cmucal_vclk_ip_ppc_tpu_d0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_AUR_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_aur_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_AUR_D0_EVENT, NULL, cmucal_vclk_ip_ppc_aur_d0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_AUR_D1_EVENT, NULL, cmucal_vclk_ip_ppc_aur_d1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_AUR_D0_CYCLE, NULL, cmucal_vclk_ip_ppc_aur_d0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL1A_CD, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl1a_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL1A, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_AUR_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_aur_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_AUR, NULL, cmucal_vclk_ip_slh_axi_si_p_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_G3D_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_g3d_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_G3D, NULL, cmucal_vclk_ip_slh_axi_si_p_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_TPU_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_tpu_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_TPU_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_tpu_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_TPU, NULL, cmucal_vclk_ip_slh_axi_si_p_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D0_AUR, NULL, cmucal_vclk_ip_lh_acel_mi_d0_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D0_TPU, NULL, cmucal_vclk_ip_lh_acel_mi_d0_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D1_AUR, NULL, cmucal_vclk_ip_lh_acel_mi_d1_aur, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D1_TPU, NULL, cmucal_vclk_ip_lh_acel_mi_d1_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_D_G3DMMU, NULL, cmucal_vclk_ip_slh_axi_mi_d_g3dmmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_D0_NOCL2AA_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_mi_d0_nocl2aa_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_D0_NOCL2AB_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_mi_d0_nocl2ab_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_D1_NOCL2AA_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_mi_d1_nocl2aa_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_D1_NOCL2AB_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_mi_d1_nocl2ab_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_D0_NOCL1A_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_si_d0_nocl1a_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_D1_NOCL1A_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_si_d1_nocl1a_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_D2_NOCL1A_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_si_d2_nocl1a_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_D3_NOCL1A_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_si_d3_nocl1a_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_NOCL2AB_S0_CYCLE, NULL, cmucal_vclk_ip_ppc_nocl2ab_s0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_TPU_D1_EVENT, NULL, cmucal_vclk_ip_ppc_tpu_d1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL1A_M0, NULL, cmucal_vclk_ip_ppmu_nocl1a_m0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL1A_M1, NULL, cmucal_vclk_ip_ppmu_nocl1a_m1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL1A_M2, NULL, cmucal_vclk_ip_ppmu_nocl1a_m2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL1A_M3, NULL, cmucal_vclk_ip_ppmu_nocl1a_m3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_NOCL1A_CMU_NOCL1A, NULL, cmucal_vclk_ip_nocl1a_cmu_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_G3DMMU_D_EVENT, NULL, cmucal_vclk_ip_ppc_g3dmmu_d_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_BW_D_EVENT, NULL, cmucal_vclk_ip_ppc_bw_d_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_BW_D_CYCLE, NULL, cmucal_vclk_ip_ppc_bw_d_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_P_NOCL0_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_mi_p_nocl0_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D_BW, NULL, cmucal_vclk_ip_lh_axi_mi_d_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_BW, NULL, cmucal_vclk_ip_slh_axi_si_p_bw, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_NOCL1B_CMU_NOCL1B, NULL, cmucal_vclk_ip_nocl1b_cmu_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_NOCL1B, NULL, cmucal_vclk_ip_trex_d_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_NOCL1B, NULL, cmucal_vclk_ip_d_tzpc_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D_HSI0, NULL, cmucal_vclk_ip_lh_acel_mi_d_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D_HSI1, NULL, cmucal_vclk_ip_lh_acel_mi_d_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D_AOC, NULL, cmucal_vclk_ip_lh_axi_mi_d_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D_ALIVE, NULL, cmucal_vclk_ip_lh_axi_mi_d_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D_GSA, NULL, cmucal_vclk_ip_lh_axi_mi_d_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_AOC_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_aoc_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_GSA_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_gsa_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_HSI0_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_hsi0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_HSI1_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_hsi1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_NOCL1B, NULL, cmucal_vclk_ip_sysreg_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_P_NOCL1B, NULL, cmucal_vclk_ip_trex_p_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_NOCL1B, NULL, cmucal_vclk_ip_gpc_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_G_CSSYS_CU, NULL, cmucal_vclk_ip_lh_axi_mi_g_cssys_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL1B_CD, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl1b_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_AOC_EVENT, NULL, cmucal_vclk_ip_ppc_aoc_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_AOC_CYCLE, NULL, cmucal_vclk_ip_ppc_aoc_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL1B_CD, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl1b_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL1B, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_AOC_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_aoc_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_AOC, NULL, cmucal_vclk_ip_slh_axi_si_p_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_GSA_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_gsa_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_GSA, NULL, cmucal_vclk_ip_slh_axi_si_p_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_HSI0_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_hsi0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_HSI0, NULL, cmucal_vclk_ip_slh_axi_si_p_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_HSI1_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_hsi1_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_HSI1, NULL, cmucal_vclk_ip_slh_axi_si_p_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_G_CSSYS, NULL, cmucal_vclk_ip_slh_axi_mi_g_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_G_CSSYS_CU, NULL, cmucal_vclk_ip_lh_axi_si_g_cssys_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL1B_M0, NULL, cmucal_vclk_ip_ppmu_nocl1b_m0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_D_NOCL1B_NOCL0, NULL, cmucal_vclk_ip_lh_taxi_si_d_nocl1b_nocl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_P_NOCL0_NOCL1B, NULL, cmucal_vclk_ip_lh_taxi_mi_p_nocl0_nocl1b, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_NOCL1B_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_nocl1b_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_NOCL2AA_CMU_NOCL2AA, NULL, cmucal_vclk_ip_nocl2aa_cmu_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_NOCL2AA, NULL, cmucal_vclk_ip_sysreg_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D_HSI2, NULL, cmucal_vclk_ip_lh_acel_mi_d_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D1_ISPFE, NULL, cmucal_vclk_ip_lh_axi_mi_d1_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D1_DPUF0, NULL, cmucal_vclk_ip_lh_axi_mi_d1_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D0_DPUF0, NULL, cmucal_vclk_ip_lh_axi_mi_d0_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D0_MFC, NULL, cmucal_vclk_ip_lh_axi_mi_d0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D1_MFC, NULL, cmucal_vclk_ip_lh_axi_mi_d1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_HSI2_CD, NULL, cmucal_vclk_ip_lh_axi_si_p_hsi2_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D0_ISPFE, NULL, cmucal_vclk_ip_lh_axi_mi_d0_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D3_ISPFE, NULL, cmucal_vclk_ip_lh_axi_mi_d3_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_NOCL2AA, NULL, cmucal_vclk_ip_d_tzpc_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_NOCL2AA, NULL, cmucal_vclk_ip_trex_d_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_NOCL2AA, NULL, cmucal_vclk_ip_gpc_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D6_RGBP, NULL, cmucal_vclk_ip_lh_axi_mi_d6_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D5_RGBP, NULL, cmucal_vclk_ip_lh_axi_mi_d5_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D0_RGBP, NULL, cmucal_vclk_ip_lh_axi_mi_d0_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D1_RGBP, NULL, cmucal_vclk_ip_lh_axi_mi_d1_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_P_NOCL2AA, NULL, cmucal_vclk_ip_trex_p_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D2_ISPFE, NULL, cmucal_vclk_ip_lh_axi_mi_d2_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D2_RGBP, NULL, cmucal_vclk_ip_lh_axi_mi_d2_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D3_RGBP, NULL, cmucal_vclk_ip_lh_axi_mi_d3_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL2AA_CD, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl2aa_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D4_RGBP, NULL, cmucal_vclk_ip_lh_axi_mi_d4_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL2AA_CD, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl2aa_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL2AA, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_HSI2_CD, NULL, cmucal_vclk_ip_lh_axi_mi_p_hsi2_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_HSI2, NULL, cmucal_vclk_ip_slh_axi_si_p_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_DPUF0, NULL, cmucal_vclk_ip_slh_axi_si_p_dpuf0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_ISPFE, NULL, cmucal_vclk_ip_slh_axi_si_p_ispfe, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_RGBP, NULL, cmucal_vclk_ip_slh_axi_si_p_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_MFC, NULL, cmucal_vclk_ip_slh_axi_si_p_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_DPUF1, NULL, cmucal_vclk_ip_slh_axi_si_p_dpuf1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_DPUB, NULL, cmucal_vclk_ip_slh_axi_si_p_dpub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL2AA_M0, NULL, cmucal_vclk_ip_ppmu_nocl2aa_m0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL2AA_M1, NULL, cmucal_vclk_ip_ppmu_nocl2aa_m1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_P_NOCL0_NOCL2AA, NULL, cmucal_vclk_ip_lh_taxi_mi_p_nocl0_nocl2aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_D0_NOCL2AA_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_si_d0_nocl2aa_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_D1_NOCL2AA_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_si_d1_nocl2aa_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_NOCL2AA_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_nocl2aa_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_NOCL2AB_CMU_NOCL2AB, NULL, cmucal_vclk_ip_nocl2ab_cmu_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_NOCL2AB, NULL, cmucal_vclk_ip_d_tzpc_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_NOCL2AB, NULL, cmucal_vclk_ip_gpc_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_NOCL2AB, NULL, cmucal_vclk_ip_sysreg_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_NOCL2AB, NULL, cmucal_vclk_ip_trex_d_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_P_NOCL2AB, NULL, cmucal_vclk_ip_trex_p_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL2AB_M0, NULL, cmucal_vclk_ip_ppmu_nocl2ab_m0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_NOCL2AB_M1, NULL, cmucal_vclk_ip_ppmu_nocl2ab_m1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D0_GDC, NULL, cmucal_vclk_ip_lh_axi_mi_d0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D1_GDC, NULL, cmucal_vclk_ip_lh_axi_mi_d1_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D_GSE, NULL, cmucal_vclk_ip_lh_axi_mi_d_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D0_G2D, NULL, cmucal_vclk_ip_lh_axi_mi_d0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D1_G2D, NULL, cmucal_vclk_ip_lh_axi_mi_d1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D0_MCSC, NULL, cmucal_vclk_ip_lh_axi_mi_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D1_MCSC, NULL, cmucal_vclk_ip_lh_axi_mi_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D0_TNR, NULL, cmucal_vclk_ip_lh_axi_mi_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D1_TNR, NULL, cmucal_vclk_ip_lh_axi_mi_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D2_TNR, NULL, cmucal_vclk_ip_lh_axi_mi_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D3_TNR, NULL, cmucal_vclk_ip_lh_axi_mi_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D_YUVP, NULL, cmucal_vclk_ip_lh_axi_mi_d_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_MI_P_NOCL0_NOCL2AB, NULL, cmucal_vclk_ip_lh_taxi_mi_p_nocl0_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_D0_NOCL2AB_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_si_d0_nocl2ab_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_TAXI_SI_D1_NOCL2AB_NOCL1A, NULL, cmucal_vclk_ip_lh_taxi_si_d1_nocl2ab_nocl1a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL2AB_CD, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl2ab_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_G_NOCL2AB_CD, NULL, cmucal_vclk_ip_lh_ast_mi_g_nocl2ab_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_G_NOCL2AB, NULL, cmucal_vclk_ip_lh_ast_si_g_nocl2ab, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_GDC, NULL, cmucal_vclk_ip_slh_axi_si_p_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_GSE, NULL, cmucal_vclk_ip_slh_axi_si_p_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_MCSC, NULL, cmucal_vclk_ip_slh_axi_si_p_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_G2D, NULL, cmucal_vclk_ip_slh_axi_si_p_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_TNR, NULL, cmucal_vclk_ip_slh_axi_si_p_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_SI_P_YUVP, NULL, cmucal_vclk_ip_slh_axi_si_p_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D_MISC, NULL, cmucal_vclk_ip_lh_acel_mi_d_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D5_TNR, NULL, cmucal_vclk_ip_lh_axi_mi_d5_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D4_TNR, NULL, cmucal_vclk_ip_lh_axi_mi_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_MI_D2_G2D, NULL, cmucal_vclk_ip_lh_acel_mi_d2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_D2_GDC, NULL, cmucal_vclk_ip_lh_axi_mi_d2_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_NOCL2AB_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_nocl2ab_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_PERIC0, NULL, cmucal_vclk_ip_gpio_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_PERIC0, NULL, cmucal_vclk_ip_sysreg_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PERIC0_CMU_PERIC0, NULL, cmucal_vclk_ip_peric0_cmu_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_PERIC0_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_peric0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_PERIC0, NULL, cmucal_vclk_ip_d_tzpc_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_PERIC0, NULL, cmucal_vclk_ip_gpc_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI1_USI, NULL, cmucal_vclk_ip_usi1_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI2_USI, NULL, cmucal_vclk_ip_usi2_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI3_USI, NULL, cmucal_vclk_ip_usi3_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI4_USI, NULL, cmucal_vclk_ip_usi4_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI5_USI, NULL, cmucal_vclk_ip_usi5_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI6_USI, NULL, cmucal_vclk_ip_usi6_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_I3C1, NULL, cmucal_vclk_ip_i3c1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_I3C2, NULL, cmucal_vclk_ip_i3c2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_I3C3, NULL, cmucal_vclk_ip_i3c3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_I3C4, NULL, cmucal_vclk_ip_i3c4, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_I3C5, NULL, cmucal_vclk_ip_i3c5, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_I3C6, NULL, cmucal_vclk_ip_i3c6, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI0_UART, NULL, cmucal_vclk_ip_usi0_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI14_USI, NULL, cmucal_vclk_ip_usi14_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_PERIC0, NULL, cmucal_vclk_ip_slh_axi_mi_p_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_PERIC0_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_peric0_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_PERIC1, NULL, cmucal_vclk_ip_gpio_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_PERIC1, NULL, cmucal_vclk_ip_sysreg_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PERIC1_CMU_PERIC1, NULL, cmucal_vclk_ip_peric1_cmu_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_PERIC1_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_peric1_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_PERIC1, NULL, cmucal_vclk_ip_d_tzpc_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_PERIC1, NULL, cmucal_vclk_ip_gpc_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI0_USI, NULL, cmucal_vclk_ip_usi0_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI9_USI, NULL, cmucal_vclk_ip_usi9_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI10_USI, NULL, cmucal_vclk_ip_usi10_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI11_USI, NULL, cmucal_vclk_ip_usi11_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI12_USI, NULL, cmucal_vclk_ip_usi12_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI13_USI, NULL, cmucal_vclk_ip_usi13_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_I3C0, NULL, cmucal_vclk_ip_i3c0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PWM, NULL, cmucal_vclk_ip_pwm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_PERIC1, NULL, cmucal_vclk_ip_slh_axi_mi_p_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_PERIC1_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_peric1_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USI15_USI, NULL, cmucal_vclk_ip_usi15_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_RGBP_CMU_RGBP, NULL, cmucal_vclk_ip_rgbp_cmu_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_RGBP, NULL, cmucal_vclk_ip_ad_apb_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_RGBP, NULL, cmucal_vclk_ip_d_tzpc_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_RGBP, NULL, cmucal_vclk_ip_gpc_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_RGBP, NULL, cmucal_vclk_ip_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_RGBP, NULL, cmucal_vclk_ip_slh_axi_mi_p_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_RGBP, NULL, cmucal_vclk_ip_sysreg_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D0_RGBP, NULL, cmucal_vclk_ip_lh_axi_si_d0_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D1_RGBP, NULL, cmucal_vclk_ip_lh_axi_si_d1_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_RGBP, NULL, cmucal_vclk_ip_ssmt_d0_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_RGBP, NULL, cmucal_vclk_ip_qe_d0_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_RGBP, NULL, cmucal_vclk_ip_ppmu_d0_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_MCFP, NULL, cmucal_vclk_ip_ad_apb_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_RGBP, NULL, cmucal_vclk_ip_ppmu_d1_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_RGBP, NULL, cmucal_vclk_ip_ppmu_d2_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_MCFP, NULL, cmucal_vclk_ip_ppmu_d0_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_MCFP, NULL, cmucal_vclk_ip_ppmu_d2_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D3_MCFP, NULL, cmucal_vclk_ip_ppmu_d3_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D4_MCFP, NULL, cmucal_vclk_ip_ppmu_d4_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D5_MCFP, NULL, cmucal_vclk_ip_ppmu_d5_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_RGBP, NULL, cmucal_vclk_ip_sysmmu_s0_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_RGBP, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_RGBP, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_RGBP, NULL, cmucal_vclk_ip_sysmmu_s1_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_PMMU0_RGBP, NULL, cmucal_vclk_ip_sysmmu_s1_pmmu0_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_PMMU1_RGBP, NULL, cmucal_vclk_ip_sysmmu_s1_pmmu1_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_PMMU2_RGBP, NULL, cmucal_vclk_ip_sysmmu_s1_pmmu2_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_PMMU3_RGBP, NULL, cmucal_vclk_ip_sysmmu_s1_pmmu3_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_PMMU4_RGBP, NULL, cmucal_vclk_ip_sysmmu_s1_pmmu4_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D2_RGBP, NULL, cmucal_vclk_ip_lh_axi_si_d2_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D3_RGBP, NULL, cmucal_vclk_ip_lh_axi_si_d3_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D4_RGBP, NULL, cmucal_vclk_ip_lh_axi_si_d4_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D5_RGBP, NULL, cmucal_vclk_ip_lh_axi_si_d5_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D6_RGBP, NULL, cmucal_vclk_ip_lh_axi_si_d6_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LD_RGBP_GDC, NULL, cmucal_vclk_ip_lh_axi_si_ld_rgbp_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MCFP, NULL, cmucal_vclk_ip_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_RGBP, NULL, cmucal_vclk_ip_ssmt_d1_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_RGBP, NULL, cmucal_vclk_ip_ssmt_d2_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_MCFP, NULL, cmucal_vclk_ip_ssmt_d0_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_MCFP, NULL, cmucal_vclk_ip_ssmt_d2_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D3_MCFP, NULL, cmucal_vclk_ip_ssmt_d3_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D4_MCFP, NULL, cmucal_vclk_ip_ssmt_d4_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D5_MCFP, NULL, cmucal_vclk_ip_ssmt_d5_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D1_RGBP, NULL, cmucal_vclk_ip_qe_d1_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D2_RGBP, NULL, cmucal_vclk_ip_qe_d2_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D3_RGBP, NULL, cmucal_vclk_ip_qe_d3_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D4_RGBP, NULL, cmucal_vclk_ip_qe_d4_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D5_RGBP, NULL, cmucal_vclk_ip_qe_d5_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D6_RGBP, NULL, cmucal_vclk_ip_qe_d6_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D4_MCFP, NULL, cmucal_vclk_ip_qe_d4_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D5_MCFP, NULL, cmucal_vclk_ip_qe_d5_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D6_MCFP, NULL, cmucal_vclk_ip_qe_d6_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D7_MCFP, NULL, cmucal_vclk_ip_qe_d7_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D8_MCFP, NULL, cmucal_vclk_ip_qe_d8_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D9_MCFP, NULL, cmucal_vclk_ip_qe_d9_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D11_MCFP, NULL, cmucal_vclk_ip_qe_d11_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_I_RGBP_MCFP, NULL, cmucal_vclk_ip_lh_ast_si_i_rgbp_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_I_RGBP_MCFP, NULL, cmucal_vclk_ip_lh_ast_mi_i_rgbp_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_OTF_RGBP_YUVP, NULL, cmucal_vclk_ip_lh_ast_si_l_otf_rgbp_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D10_MCFP, NULL, cmucal_vclk_ip_qe_d10_mcfp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_RGBP_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_rgbp_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_RGBP, NULL, cmucal_vclk_ip_xiu_d1_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D4_RGBP, NULL, cmucal_vclk_ip_xiu_d4_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D5_RGBP, NULL, cmucal_vclk_ip_xiu_d5_rgbp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_S2D_CMU_S2D, NULL, cmucal_vclk_ip_s2d_cmu_s2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BIS_S2D, NULL, cmucal_vclk_ip_bis_s2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_LG_SCAN2DRAM_CU, NULL, cmucal_vclk_ip_lh_axi_mi_lg_scan2dram_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_LG_SCAN2DRAM, NULL, cmucal_vclk_ip_slh_axi_mi_lg_scan2dram, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_LG_SCAN2DRAM_CU, NULL, cmucal_vclk_ip_lh_axi_si_lg_scan2dram_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_GTNR_MERGE, NULL, cmucal_vclk_ip_ad_apb_gtnr_merge, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_TNR, NULL, cmucal_vclk_ip_d_tzpc_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_TNR, NULL, cmucal_vclk_ip_slh_axi_mi_p_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_OTF_TNR_MCSC, NULL, cmucal_vclk_ip_lh_ast_si_l_otf_tnr_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D0_TNR, NULL, cmucal_vclk_ip_lh_axi_si_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D1_TNR, NULL, cmucal_vclk_ip_lh_axi_si_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_TNR, NULL, cmucal_vclk_ip_ppmu_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_TNR, NULL, cmucal_vclk_ip_ppmu_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_TNR, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_TNR, NULL, cmucal_vclk_ip_sysreg_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_OTF_YUVP_TNR, NULL, cmucal_vclk_ip_lh_ast_mi_l_otf_yuvp_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D2_TNR, NULL, cmucal_vclk_ip_lh_axi_si_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D3_TNR, NULL, cmucal_vclk_ip_lh_axi_si_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_TNR, NULL, cmucal_vclk_ip_ppmu_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D3_TNR, NULL, cmucal_vclk_ip_ppmu_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_PMMU2_TNR, NULL, cmucal_vclk_ip_sysmmu_s1_pmmu2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_PMMU1_TNR, NULL, cmucal_vclk_ip_sysmmu_s1_pmmu1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D10_TNRA, NULL, cmucal_vclk_ip_qe_d10_tnra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_TNR, NULL, cmucal_vclk_ip_xiu_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_TNR, NULL, cmucal_vclk_ip_xiu_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_TNR, NULL, cmucal_vclk_ip_qe_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D1_TNR, NULL, cmucal_vclk_ip_qe_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D5_TNR, NULL, cmucal_vclk_ip_qe_d5_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D6_TNR, NULL, cmucal_vclk_ip_qe_d6_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D7_TNR, NULL, cmucal_vclk_ip_qe_d7_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_TNR, NULL, cmucal_vclk_ip_ssmt_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_TNR, NULL, cmucal_vclk_ip_ssmt_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_TNR, NULL, cmucal_vclk_ip_ssmt_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D3_TNR, NULL, cmucal_vclk_ip_ssmt_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_PMMU0_TNR, NULL, cmucal_vclk_ip_sysmmu_s1_pmmu0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_TNR, NULL, cmucal_vclk_ip_gpc_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_OTF_TNR_GSE, NULL, cmucal_vclk_ip_lh_ast_si_l_otf_tnr_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D8_TNR, NULL, cmucal_vclk_ip_qe_d8_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D9_TNR, NULL, cmucal_vclk_ip_qe_d9_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_TNR, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D4_TNR, NULL, cmucal_vclk_ip_xiu_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D11_TNRA, NULL, cmucal_vclk_ip_qe_d11_tnra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D2_TNR, NULL, cmucal_vclk_ip_xiu_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D3_TNR, NULL, cmucal_vclk_ip_xiu_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GTNR_MERGE, NULL, cmucal_vclk_ip_gtnr_merge, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D4_TNR, NULL, cmucal_vclk_ip_lh_axi_si_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D10_TNRA, NULL, cmucal_vclk_ip_ppmu_d10_tnra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D4_TNR, NULL, cmucal_vclk_ip_ppmu_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D5_TNR, NULL, cmucal_vclk_ip_ppmu_d5_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D6_TNR, NULL, cmucal_vclk_ip_ppmu_d6_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D7_TNR, NULL, cmucal_vclk_ip_ppmu_d7_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D8_TNR, NULL, cmucal_vclk_ip_ppmu_d8_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D9_TNR, NULL, cmucal_vclk_ip_ppmu_d9_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D10_TNRA, NULL, cmucal_vclk_ip_ssmt_d10_tnra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D4_TNR, NULL, cmucal_vclk_ip_ssmt_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D5_TNR, NULL, cmucal_vclk_ip_ssmt_d5_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D6_TNR, NULL, cmucal_vclk_ip_ssmt_d6_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D7_TNR, NULL, cmucal_vclk_ip_ssmt_d7_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D8_TNR, NULL, cmucal_vclk_ip_ssmt_d8_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D9_TNR, NULL, cmucal_vclk_ip_ssmt_d9_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_TNR, NULL, cmucal_vclk_ip_sysmmu_s0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S1_TNR, NULL, cmucal_vclk_ip_sysmmu_s1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D6_TNR, NULL, cmucal_vclk_ip_xiu_d6_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TNR_CMU_TNR, NULL, cmucal_vclk_ip_tnr_cmu_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D11_TNRA, NULL, cmucal_vclk_ip_ssmt_d11_tnra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D11_TNRA, NULL, cmucal_vclk_ip_ppmu_d11_tnra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_GTNR_ALIGN, NULL, cmucal_vclk_ip_ad_apb_gtnr_align, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GTNR_ALIGN, NULL, cmucal_vclk_ip_gtnr_align, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D10_TNR, NULL, cmucal_vclk_ip_xiu_d10_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D2_TNR, NULL, cmucal_vclk_ip_qe_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D3_TNR, NULL, cmucal_vclk_ip_qe_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D4_TNR, NULL, cmucal_vclk_ip_qe_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S2_PMMU0_TNR, NULL, cmucal_vclk_ip_sysmmu_s2_pmmu0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S2_TNR, NULL, cmucal_vclk_ip_sysmmu_s2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D7_TNR, NULL, cmucal_vclk_ip_xiu_d7_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D11_TNR, NULL, cmucal_vclk_ip_xiu_d11_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D5_TNR, NULL, cmucal_vclk_ip_lh_axi_si_d5_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_TNR_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_tnr_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TPU_CMU_TPU, NULL, cmucal_vclk_ip_tpu_cmu_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_MI_P_TPU_CU, NULL, cmucal_vclk_ip_lh_axi_mi_p_tpu_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_TPU, NULL, cmucal_vclk_ip_d_tzpc_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D0_TPU, NULL, cmucal_vclk_ip_lh_acel_si_d0_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_TPU, NULL, cmucal_vclk_ip_sysreg_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_TPU, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_TPU, NULL, cmucal_vclk_ip_ppmu_d0_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_TPU, NULL, cmucal_vclk_ip_ssmt_d0_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_TPU, NULL, cmucal_vclk_ip_gpc_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_SYSMMU_S1_NS_TPU, NULL, cmucal_vclk_ip_as_apb_sysmmu_s1_ns_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TPU, NULL, cmucal_vclk_ip_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT0_TPU_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_si_lt0_tpu_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT1_TPU_CPUCL0, NULL, cmucal_vclk_ip_lh_atb_si_lt1_tpu_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADM_DAP_G_TPU, NULL, cmucal_vclk_ip_adm_dap_g_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ASYNC_APB_INT_TPU, NULL, cmucal_vclk_ip_async_apb_int_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT0_TPU_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_atb_mi_lt0_tpu_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_MI_LT1_TPU_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_atb_mi_lt1_tpu_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT0_TPU_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_atb_si_lt0_tpu_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ATB_SI_LT1_TPU_CPUCL0_CD, NULL, cmucal_vclk_ip_lh_atb_si_lt1_tpu_cpucl0_cd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_TPU, NULL, cmucal_vclk_ip_slh_axi_mi_p_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_P_TPU_CU, NULL, cmucal_vclk_ip_lh_axi_si_p_tpu_cu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_TPU, NULL, cmucal_vclk_ip_xiu_d_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_TPU, NULL, cmucal_vclk_ip_ssmt_d1_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_TPU, NULL, cmucal_vclk_ip_ppmu_d1_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU1_TPU, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu1_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_TPU, NULL, cmucal_vclk_ip_sysmmu_s0_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_ACEL_SI_D1_TPU, NULL, cmucal_vclk_ip_lh_acel_si_d1_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD_APBIF_TPU, NULL, cmucal_vclk_ip_add_apbif_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD_TPU, NULL, cmucal_vclk_ip_add_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DAPAPBAP_TPU, NULL, cmucal_vclk_ip_dapapbap_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_TPU_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_tpu_frc_otp_deserial, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_YUVP, NULL, cmucal_vclk_ip_ad_apb_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_YUVP, NULL, cmucal_vclk_ip_d_tzpc_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_YUVP, NULL, cmucal_vclk_ip_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_YUVP, NULL, cmucal_vclk_ip_gpc_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLH_AXI_MI_P_YUVP, NULL, cmucal_vclk_ip_slh_axi_mi_p_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AXI_SI_D_YUVP, NULL, cmucal_vclk_ip_lh_axi_si_d_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_YUVP, NULL, cmucal_vclk_ip_ppmu_d0_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_YUVP, NULL, cmucal_vclk_ip_ssmt_d0_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_PMMU0_YUVP, NULL, cmucal_vclk_ip_sysmmu_s0_pmmu0_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_YUVP, NULL, cmucal_vclk_ip_sysreg_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_OTF_YUVP_MCSC, NULL, cmucal_vclk_ip_lh_ast_si_l_otf_yuvp_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_MI_L_OTF_RGBP_YUVP, NULL, cmucal_vclk_ip_lh_ast_mi_l_otf_rgbp_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_OTF_YUVP_GSE, NULL, cmucal_vclk_ip_lh_ast_si_l_otf_yuvp_gse, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LH_AST_SI_L_OTF_YUVP_TNR, NULL, cmucal_vclk_ip_lh_ast_si_l_otf_yuvp_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_YUVP, NULL, cmucal_vclk_ip_qe_d0_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_YUVP_CMU_YUVP, NULL, cmucal_vclk_ip_yuvp_cmu_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_YUVP, NULL, cmucal_vclk_ip_ssmt_d1_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_YUVP, NULL, cmucal_vclk_ip_ppmu_d1_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D1_YUVP, NULL, cmucal_vclk_ip_qe_d1_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D4_YUVP, NULL, cmucal_vclk_ip_ppmu_d4_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D4_YUVP, NULL, cmucal_vclk_ip_ssmt_d4_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D4_YUVP, NULL, cmucal_vclk_ip_qe_d4_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S0_YUVP, NULL, cmucal_vclk_ip_sysmmu_s0_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_YUVP, NULL, cmucal_vclk_ip_xiu_d1_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_YUVP, NULL, cmucal_vclk_ip_xiu_d0_yuvp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BLK_YUVP_FRC_OTP_DESERIAL, NULL, cmucal_vclk_ip_blk_yuvp_frc_otp_deserial, NULL, NULL),
};
unsigned int cmucal_vclk_size = ARRAY_SIZE(cmucal_vclk_list);
