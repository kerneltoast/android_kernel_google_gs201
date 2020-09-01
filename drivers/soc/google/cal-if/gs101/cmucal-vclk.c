#include "../cmucal.h"
#include "cmucal-node.h"
#include "cmucal-vclk.h"
#include "cmucal-vclklut.h"

/* DVFS VCLK -> Clock Node List */
enum clk_id cmucal_vclk_vdd_int[] = {
	CLKCMU_MFC_MFC,
	MUX_CLKCMU_MFC_MFC,
	CLKCMU_HSI0_USB31DRD,
	CLKCMU_G2D_G2D,
	MUX_CLKCMU_G2D_G2D,
	CLKCMU_CSIS_BUS,
	MUX_CLKCMU_CSIS_BUS,
	MUX_CLKCMU_MIF_SWITCH,
	CLKCMU_ITP_BUS,
	MUX_CLKCMU_ITP_BUS,
	CLKCMU_G3AA_G3AA,
	MUX_CLKCMU_G3AA_G3AA,
	CLKCMU_MCSC_ITSC,
	MUX_CLKCMU_MCSC_ITSC,
	CLKCMU_G2D_MSCL,
	MUX_CLKCMU_G2D_MSCL,
	CLKCMU_BO_BUS,
	MUX_CLKCMU_BO_BUS,
	DIV_CLK_MISC_BUSP,
	DIV_CLK_MISC_GIC,
	CLKCMU_MISC_BUS,
	MUX_CLKCMU_MISC_BUS,
	CLKCMU_MIF_BUSP,
	MUX_CLKCMU_MIF_BUSP,
	CLKCMU_PDP_VRA,
	MUX_CLKCMU_PDP_VRA,
	CLKCMU_DPU_BUS,
	MUX_CLKCMU_DPU_BUS,
	CLKCMU_CPUCL1_SWITCH,
	MUX_CLKCMU_CPUCL1_SWITCH,
	MUX_CLKCMU_HSI0_BUS,
	CLKCMU_IPP_BUS,
	MUX_CLKCMU_IPP_BUS,
	CLKCMU_TNR_BUS,
	MUX_CLKCMU_TNR_BUS,
	CLKCMU_BUS1_BUS,
	MUX_CLKCMU_BUS1_BUS,
	CLKCMU_BUS0_BUS,
	MUX_CLKCMU_BUS0_BUS,
	CLKCMU_DNS_BUS,
	MUX_CLKCMU_DNS_BUS,
	CLKCMU_GDC_GDC0,
	MUX_CLKCMU_GDC_GDC0,
	CLKCMU_GDC_GDC1,
	MUX_CLKCMU_GDC_GDC1,
	CLKCMU_MCSC_MCSC,
	MUX_CLKCMU_MCSC_MCSC,
	CLKCMU_GDC_SCSC,
	MUX_CLKCMU_GDC_SCSC,
	CLKCMU_MISC_SSS,
	MUX_CLKCMU_MISC_SSS,
	CLKCMU_DISP_BUS,
	MUX_CLKCMU_DISP_BUS,
	CLKCMU_EH_BUS,
	MUX_CLKCMU_EH_BUS,
	CLKCMU_PDP_BUS,
	MUX_CLKCMU_PDP_BUS,
	CLKCMU_G3D_BUSD,
	MUX_CLKCMU_G3D_BUSD,
	CLKCMU_CPUCL0_SWITCH,
	MUX_CLKCMU_CPUCL0_SWITCH,
	CLKCMU_CPUCL0_DBG,
	MUX_CLKCMU_CPUCL0_DBG,
	CLKCMU_HSI1_BUS,
	MUX_CLKCMU_HSI1_BUS,
	CLKCMU_HSI2_BUS,
	MUX_CLKCMU_HSI2_BUS,
	CLKCMU_TPU_BUS,
	MUX_CLKCMU_TPU_BUS,
	DIV_CLK_CMU_CMUREF,
	MUX_CLKCMU_TOP_CMUREF,
	DIV_CLK_BUS2_BUSP,
	CLKCMU_BUS2_BUS,
	MUX_CLKCMU_BUS2_BUS,
	CLKCMU_CPUCL2_SWITCH,
	MUX_CLKCMU_CPUCL2_SWITCH,
	CLKCMU_HPM,
	CLKCMU_HSI2_PCIE,
	CLKCMU_CIS_CLK0,
	CLKCMU_CIS_CLK1,
	CLKCMU_CIS_CLK2,
	CLKCMU_CIS_CLK3,
	CLKCMU_HSI2_UFS_EMBD,
	CLKCMU_PERIC0_IP,
	CLKCMU_PERIC1_IP,
	CLKCMU_HSI1_PCIE,
	CLKCMU_CIS_CLK4,
	MUX_BUS2_CMUREF,
	MUX_CLKCMU_CMU_BOOST_OPTION1,
	DIV_CLKCMU_CMU_BOOST,
	CLKCMU_CIS_CLK5,
	CLKCMU_CIS_CLK6,
	CLKCMU_CIS_CLK7,
};
enum clk_id cmucal_vclk_vdd_mif[] = {
	MUX_CLK_EH_BUS,
	PLL_CORE,
	PLL_MIF,
	PLL_MIF_EXT,
	MUX_CLK_S2D_CORE,
};
enum clk_id cmucal_vclk_vdd_g3d[] = {
	PLL_G3D,
	PLL_G3D_L2,
};
enum clk_id cmucal_vclk_vdd_cpucl0[] = {
	PLL_CPUCL0,
	MUX_CPUCL0_CMUREF,
};
enum clk_id cmucal_vclk_vdd_cpucl1[] = {
	PLL_CPUCL1,
};
enum clk_id cmucal_vclk_vdd_tpu[] = {
	PLL_TPU,
};
enum clk_id cmucal_vclk_vdd_cpucl2[] = {
	PLL_CPUCL2,
};
/* SPECIAL VCLK -> Clock Node List */
enum clk_id cmucal_vclk_mux_bus0_cmuref[] = {
	MUX_BUS0_CMUREF,
};
enum clk_id cmucal_vclk_mux_bus1_cmuref[] = {
	MUX_BUS1_CMUREF,
};
enum clk_id cmucal_vclk_mux_cmu_cmuref[] = {
	MUX_CMU_CMUREF,
	MUX_CLKCMU_TOP_BOOST_OPTION1,
};
enum clk_id cmucal_vclk_mux_core_cmuref[] = {
	MUX_CORE_CMUREF,
};
enum clk_id cmucal_vclk_mux_cpucl1_cmuref[] = {
	MUX_CPUCL1_CMUREF,
};
enum clk_id cmucal_vclk_mux_cpucl2_cmuref[] = {
	MUX_CPUCL2_CMUREF,
};
enum clk_id cmucal_vclk_mux_clk_hsi0_usb20_ref[] = {
	MUX_CLK_HSI0_USB20_REF,
};
enum clk_id cmucal_vclk_mux_clkcmu_hsi0_usbdpdbg[] = {
	MUX_CLKCMU_HSI0_USBDPDBG,
};
enum clk_id cmucal_vclk_mux_mif_cmuref[] = {
	MUX_MIF_CMUREF,
};
enum clk_id cmucal_vclk_clkcmu_hsi0_dpgtc[] = {
	CLKCMU_HSI0_DPGTC,
	MUX_CLKCMU_HSI0_DPGTC,
};
enum clk_id cmucal_vclk_mux_clkcmu_hsi1_pcie[] = {
	MUX_CLKCMU_HSI1_PCIE,
};
enum clk_id cmucal_vclk_mux_clkcmu_hsi2_pcie[] = {
	MUX_CLKCMU_HSI2_PCIE,
};
enum clk_id cmucal_vclk_clkcmu_tpu_uart[] = {
	CLKCMU_TPU_UART,
	MUX_CLKCMU_TPU_UART,
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
enum clk_id cmucal_vclk_mux_clkcmu_hpm[] = {
	MUX_CLKCMU_HPM,
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
enum clk_id cmucal_vclk_div_clk_cpucl0_cmuref[] = {
	DIV_CLK_CPUCL0_CMUREF,
};
enum clk_id cmucal_vclk_div_clk_cluster0_periphclk[] = {
	DIV_CLK_CLUSTER0_PERIPHCLK,
};
enum clk_id cmucal_vclk_div_clk_cpucl1_cmuref[] = {
	DIV_CLK_CPUCL1_CMUREF,
};
enum clk_id cmucal_vclk_div_clk_cpucl2_cmuref[] = {
	DIV_CLK_CPUCL2_CMUREF,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi1_usi[] = {
	DIV_CLK_PERIC0_USI1_USI,
	MUX_CLKCMU_PERIC0_USI1_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi2_usi[] = {
	DIV_CLK_PERIC0_USI2_USI,
	MUX_CLKCMU_PERIC0_USI2_USI_USER,
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
enum clk_id cmucal_vclk_div_clk_peric0_usi6_usi[] = {
	DIV_CLK_PERIC0_USI6_USI,
	MUX_CLKCMU_PERIC0_USI6_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi7_usi[] = {
	DIV_CLK_PERIC0_USI7_USI,
	MUX_CLKCMU_PERIC0_USI7_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi8_usi[] = {
	DIV_CLK_PERIC0_USI8_USI,
	MUX_CLKCMU_PERIC0_USI8_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_i3c[] = {
	DIV_CLK_PERIC0_I3C,
	MUX_CLKCMU_PERIC0_I3C_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi0_uart[] = {
	DIV_CLK_PERIC0_USI0_UART,
	MUX_CLKCMU_PERIC0_USI0_UART_USER,
};
enum clk_id cmucal_vclk_div_clk_peric0_usi14_usi[] = {
	DIV_CLK_PERIC0_USI14_USI,
	MUX_CLKCMU_PERIC0_USI14_USI_USER,
};
enum clk_id cmucal_vclk_mux_clkcmu_peric0_ip[] = {
	MUX_CLKCMU_PERIC0_IP,
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
enum clk_id cmucal_vclk_div_clk_peric1_usi11_usi[] = {
	DIV_CLK_PERIC1_USI11_USI,
	MUX_CLKCMU_PERIC1_USI11_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric1_usi12_usi[] = {
	DIV_CLK_PERIC1_USI12_USI,
	MUX_CLKCMU_PERIC1_USI12_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric1_usi13_usi[] = {
	DIV_CLK_PERIC1_USI13_USI,
	MUX_CLKCMU_PERIC1_USI13_USI_USER,
};
enum clk_id cmucal_vclk_div_clk_peric1_i3c[] = {
	DIV_CLK_PERIC1_I3C,
	MUX_CLKCMU_PERIC1_I3C_USER,
};

enum clk_id cmucal_vclk_mux_clkcmu_peric1_ip[] = {
	MUX_CLKCMU_PERIC1_IP,
};

enum clk_id cmucal_vclk_div_clk_top_hsi0_bus[] = {
	CLKCMU_HSI0_BUS,
	MUX_CLKCMU_HSI0_BUS,
};

/* COMMON VCLK -> Clock Node List */
enum clk_id cmucal_vclk_blk_cmu[] = {
	PLL_SHARED0_DIV3,
	CLKCMU_PERIC0_BUS,
	MUX_CLKCMU_PERIC0_BUS,
	MUX_CLKCMU_HSI2_UFS_EMBD,
	CLKCMU_HSI0_BUS,
	MUX_CLKCMU_CMU_BOOST,
	CLKCMU_HSI2_MMC_CARD,
	MUX_CLKCMU_HSI2_MMC_CARD,
	CLKCMU_PERIC1_BUS,
	MUX_CLKCMU_PERIC1_BUS,
	PLL_SHARED0_DIV4,
	PLL_SHARED0_DIV2,
	PLL_SHARED0_DIV5,
	PLL_SHARED0,
	PLL_SHARED1_DIV3,
	PLL_SHARED1_DIV4,
	PLL_SHARED1_DIV2,
	PLL_SHARED1,
	MUX_CLKCMU_HSI0_USB31DRD,
	PLL_SHARED2_DIV2,
	PLL_SHARED2,
	PLL_SHARED3_DIV2,
	PLL_SHARED3,
	PLL_SPARE,
};
enum clk_id cmucal_vclk_blk_hsi0[] = {
	MUX_CLK_HSI0_USB31DRD,
	PLL_USB,
	MUX_CLK_HSI0_BUS,
	DIV_CLK_HSI0_USB31DRD,
};
enum clk_id cmucal_vclk_blk_s2d[] = {
	PLL_MIF_S2D,
};
enum clk_id cmucal_vclk_blk_apm[] = {
	DIV_CLK_APM_BOOST,
	MUX_CLKCMU_APM_FUNC,
	MUX_CLKCMU_APM_FUNCSRC,
};
enum clk_id cmucal_vclk_blk_bus0[] = {
	DIV_CLK_BUS0_BUSP,
	MUX_CLK_BUS0_BUS_OPTION1,
};
enum clk_id cmucal_vclk_blk_core[] = {
	DIV_CLK_CORE_BUSP,
	MUX_CLK_CORE_BUS_OPTION1,
};
enum clk_id cmucal_vclk_blk_cpucl0[] = {
	DIV_CLK_CLUSTER0_ACLK,
	DIV_CLK_CLUSTER0_ATCLK,
	DIV_CLK_CLUSTER0_PCLKDBG,
	DIV_CLK_CPUCL0_PCLK,
	MUX_CLK_CPUCL0_PLL,
	DIV_CLK_CPUCL0_DBG_PCLKDBG,
	DIV_CLK_CPUCL0_DBG_BUS,
};
enum clk_id cmucal_vclk_blk_cpucl1[] = {
	MUX_CLK_CPUCL1_PLL,
};
enum clk_id cmucal_vclk_blk_cpucl2[] = {
	MUX_CLK_CPUCL2_PLL,
};
enum clk_id cmucal_vclk_blk_g3d[] = {
	DIV_CLK_G3D_TOP,
	MUX_CLK_G3D_TOP,
	DIV_CLK_G3D_BUSP,
};
enum clk_id cmucal_vclk_blk_bo[] = {
	DIV_CLK_BO_BUSP,
};
enum clk_id cmucal_vclk_blk_bus1[] = {
	DIV_CLK_BUS1_BUSP,
};
enum clk_id cmucal_vclk_blk_csis[] = {
	DIV_CLK_CSIS_BUSP,
};
enum clk_id cmucal_vclk_blk_disp[] = {
	DIV_CLK_DISP_BUSP,
};
enum clk_id cmucal_vclk_blk_dns[] = {
	DIV_CLK_DNS_BUSP,
};
enum clk_id cmucal_vclk_blk_dpu[] = {
	DIV_CLK_DPU_BUSP,
};
enum clk_id cmucal_vclk_blk_eh[] = {
	DIV_CLK_EH_BUSP,
};
enum clk_id cmucal_vclk_blk_g2d[] = {
	DIV_CLK_G2D_BUSP,
};
enum clk_id cmucal_vclk_blk_g3aa[] = {
	DIV_CLK_G3AA_BUSP,
};
enum clk_id cmucal_vclk_blk_gdc[] = {
	DIV_CLK_GDC_BUSP,
};
enum clk_id cmucal_vclk_blk_ipp[] = {
	DIV_CLK_IPP_BUSP,
};
enum clk_id cmucal_vclk_blk_itp[] = {
	DIV_CLK_ITP_BUSP,
};
enum clk_id cmucal_vclk_blk_mcsc[] = {
	DIV_CLK_MCSC_BUSP,
};
enum clk_id cmucal_vclk_blk_mfc[] = {
	DIV_CLK_MFC_BUSP,
};
enum clk_id cmucal_vclk_blk_pdp[] = {
	DIV_CLK_PDP_BUSP,
};
enum clk_id cmucal_vclk_blk_tnr[] = {
	DIV_CLK_TNR_BUSP,
};
enum clk_id cmucal_vclk_blk_tpu[] = {
	DIV_CLK_TPU_BUSP,
	DIV_CLK_TPU_TPUCTL_DBG,
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
enum clk_id cmucal_vclk_ip_lhm_axi_d_hsi0aoc[] = {
	GOUT_BLK_AOC_UID_LHM_AXI_D_HSI0AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_aoc[] = {
	GOUT_BLK_AOC_UID_LHM_AXI_P_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_aoc[] = {
	GOUT_BLK_AOC_UID_LHS_AXI_D_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_aocapm[] = {
	GOUT_BLK_AOC_UID_LHS_AXI_P_AOCAPM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_aochsi0[] = {
	GOUT_BLK_AOC_UID_LHS_AXI_P_AOCHSI0_IPCLKPORT_I_CLK,
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
enum clk_id cmucal_vclk_ip_sysmmu_aoc[] = {
	GOUT_BLK_AOC_UID_SYSMMU_AOC_IPCLKPORT_CLK_S1,
	GOUT_BLK_AOC_UID_SYSMMU_AOC_IPCLKPORT_CLK_S2,
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
enum clk_id cmucal_vclk_ip_lhs_atb_t_aoc[] = {
	GOUT_BLK_AOC_UID_LHS_ATB_T_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_aoc[] = {
	GOUT_BLK_AOC_UID_LHM_AXI_G_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_aoc_sysctrl_apb[] = {
	GOUT_BLK_AOC_UID_AOC_SYSCTRL_APB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_apm[] = {
	GOUT_BLK_APM_UID_LHS_AXI_D_APM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_apm[] = {
	GOUT_BLK_APM_UID_LHM_AXI_P_APM_IPCLKPORT_I_CLK,
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
enum clk_id cmucal_vclk_ip_lhs_axi_g_scan2dram[] = {
	GOUT_BLK_APM_UID_LHS_AXI_G_SCAN2DRAM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_pmu_intr_gen[] = {
	GOUT_BLK_APM_UID_PMU_INTR_GEN_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_speedy_apm[] = {
	GOUT_BLK_APM_UID_SPEEDY_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_dp_apm[] = {
	GOUT_BLK_APM_UID_XIU_DP_APM_IPCLKPORT_ACLK,
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
enum clk_id cmucal_vclk_ip_apbif_trtc[] = {
	GOUT_BLK_APM_UID_APBIF_TRTC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_apm[] = {
	GOUT_BLK_APM_UID_D_TZPC_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_aocapm[] = {
	GOUT_BLK_APM_UID_LHM_AXI_P_AOCAPM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mailbox_apm_aoc[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_dbgcore[] = {
	GOUT_BLK_APM_UID_MAILBOX_AP_DBGCORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_g_dbgcore[] = {
	GOUT_BLK_APM_UID_LHS_AXI_G_DBGCORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_apbif_rtc[] = {
	GOUT_BLK_APM_UID_APBIF_RTC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_speedy_sub_apm[] = {
	GOUT_BLK_APM_UID_SPEEDY_SUB_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_apm_gsa[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM_GSA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mailbox_ap_aoc[] = {
	GOUT_BLK_APM_UID_MAILBOX_AP_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d_apm[] = {
	GOUT_BLK_APM_UID_SSMT_D_APM_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_SSMT_D_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_g_dbgcore[] = {
	GOUT_BLK_APM_UID_SSMT_G_DBGCORE_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_SSMT_G_DBGCORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d_apm[] = {
	GOUT_BLK_APM_UID_SYSMMU_D_APM_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_gpc_apm[] = {
	GOUT_BLK_APM_UID_GPC_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_apm[] = {
	GOUT_BLK_APM_UID_UASC_APM_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_UASC_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_dbgcore[] = {
	GOUT_BLK_APM_UID_UASC_DBGCORE_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_UASC_DBGCORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_p_apm[] = {
	GOUT_BLK_APM_UID_UASC_P_APM_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_UASC_P_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_p_aocapm[] = {
	GOUT_BLK_APM_UID_UASC_P_AOCAPM_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_UASC_P_AOCAPM_IPCLKPORT_PCLK,
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
enum clk_id cmucal_vclk_ip_lhm_axi_g_swd[] = {
	GOUT_BLK_APM_UID_LHM_AXI_G_SWD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_uasc_g_swd[] = {
	GOUT_BLK_APM_UID_UASC_G_SWD_IPCLKPORT_ACLK,
	GOUT_BLK_APM_UID_UASC_G_SWD_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_apm_usi0_uart[] = {
	GOUT_BLK_APM_UID_APM_USI0_UART_IPCLKPORT_PCLK,
	GOUT_BLK_APM_UID_APM_USI0_UART_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_apm_usi1_uart[] = {
	GOUT_BLK_APM_UID_APM_USI1_UART_IPCLKPORT_PCLK,
	GOUT_BLK_APM_UID_APM_USI1_UART_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_apm_usi0_usi[] = {
	GOUT_BLK_APM_UID_APM_USI0_USI_IPCLKPORT_PCLK,
	GOUT_BLK_APM_UID_APM_USI0_USI_IPCLKPORT_IPCLK,
};
enum clk_id cmucal_vclk_ip_bo_cmu_bo[] = {
	CLK_BLK_BO_UID_BO_CMU_BO_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_bo[] = {
	GOUT_BLK_BO_UID_LHS_AXI_D_BO_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_bo[] = {
	GOUT_BLK_BO_UID_LHM_AXI_P_BO_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_bo[] = {
	GOUT_BLK_BO_UID_PPMU_BO_IPCLKPORT_ACLK,
	GOUT_BLK_BO_UID_PPMU_BO_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_bo[] = {
	GOUT_BLK_BO_UID_SYSMMU_BO_IPCLKPORT_CLK_S1,
	GOUT_BLK_BO_UID_SYSMMU_BO_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_as_apb_sysmmu_s1_ns_bo[] = {
	GOUT_BLK_BO_UID_AS_APB_SYSMMU_S1_NS_BO_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_sysreg_bo[] = {
	GOUT_BLK_BO_UID_SYSREG_BO_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_bo[] = {
	GOUT_BLK_BO_UID_SSMT_BO_IPCLKPORT_PCLK,
	GOUT_BLK_BO_UID_SSMT_BO_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_bo[] = {
	GOUT_BLK_BO_UID_D_TZPC_BO_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_bo[] = {
	GOUT_BLK_BO_UID_GPC_BO_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_bo[] = {
	GOUT_BLK_BO_UID_UASC_BO_IPCLKPORT_ACLK,
	GOUT_BLK_BO_UID_UASC_BO_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_as_axi_p_bo[] = {
	GOUT_BLK_BO_UID_AS_AXI_P_BO_IPCLKPORT_ACLKM,
};
enum clk_id cmucal_vclk_ip_bo[] = {
	CLK_BLK_BO_UID_BO_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_bus0_cmu_bus0[] = {
	CLK_BLK_BUS0_UID_BUS0_CMU_BUS0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_d_bus0[] = {
	GOUT_BLK_BUS0_UID_TREX_D_BUS0_IPCLKPORT_ACLK,
	GOUT_BLK_BUS0_UID_TREX_D_BUS0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_bus0[] = {
	GOUT_BLK_BUS0_UID_D_TZPC_BUS0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d_hsi0[] = {
	GOUT_BLK_BUS0_UID_LHM_ACEL_D_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d_hsi1[] = {
	GOUT_BLK_BUS0_UID_LHM_ACEL_D_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_aoc[] = {
	GOUT_BLK_BUS0_UID_LHM_AXI_D_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_apm[] = {
	GOUT_BLK_BUS0_UID_LHM_AXI_D_APM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_gsa[] = {
	GOUT_BLK_BUS0_UID_LHM_AXI_D_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_aoc[] = {
	GOUT_BLK_BUS0_UID_LHS_AXI_P_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_gsa[] = {
	GOUT_BLK_BUS0_UID_LHS_AXI_P_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_hsi0[] = {
	GOUT_BLK_BUS0_UID_LHS_AXI_P_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_hsi1[] = {
	GOUT_BLK_BUS0_UID_LHS_AXI_P_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_bus0[] = {
	GOUT_BLK_BUS0_UID_SYSREG_BUS0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_p_bus0[] = {
	GOUT_BLK_BUS0_UID_TREX_P_BUS0_IPCLKPORT_ACLK_P_BUS0,
	GOUT_BLK_BUS0_UID_TREX_P_BUS0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_bus0[] = {
	GOUT_BLK_BUS0_UID_GPC_BUS0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_cssys[] = {
	GOUT_BLK_BUS0_UID_LHM_AXI_G_CSSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_dbg_g_bus0[] = {
	GOUT_BLK_BUS0_UID_LHS_DBG_G_BUS0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_aoc[] = {
	GOUT_BLK_BUS0_UID_PPC_EVENT_AOC_IPCLKPORT_CLK,
	GOUT_BLK_BUS0_UID_PPC_EVENT_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cycle_aoc[] = {
	GOUT_BLK_BUS0_UID_PPC_CYCLE_AOC_IPCLKPORT_CLK,
	GOUT_BLK_BUS0_UID_PPC_CYCLE_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_bus1_cmu_bus1[] = {
	CLK_BLK_BUS1_UID_BUS1_CMU_BUS1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_bus1[] = {
	GOUT_BLK_BUS1_UID_SYSREG_BUS1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d0_g2d[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D0_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d1_g2d[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D1_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d2_g2d[] = {
	GOUT_BLK_BUS1_UID_LHM_ACEL_D2_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d0_csis[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D0_CSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d_misc[] = {
	GOUT_BLK_BUS1_UID_LHM_ACEL_D_MISC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d0_dpu[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D0_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d0_mfc[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D0_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d1_dpu[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D1_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d1_mfc[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D1_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d2_dpu[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D2_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_dpu[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_tnr[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_hsi2[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_g2d[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_dns[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_DNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_itp[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_ITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_mfc[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d1_csis[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D1_CSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_mcsc[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d_hsi2[] = {
	GOUT_BLK_BUS1_UID_LHM_ACEL_D_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_bo[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D_BO_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_bus1[] = {
	GOUT_BLK_BUS1_UID_D_TZPC_BUS1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_d_bus1[] = {
	GOUT_BLK_BUS1_UID_TREX_D_BUS1_IPCLKPORT_ACLK,
	GOUT_BLK_BUS1_UID_TREX_D_BUS1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_bo[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_BO_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_csis[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_CSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_g3aa[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_G3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_ipp[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_IPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_bus1[] = {
	GOUT_BLK_BUS1_UID_GPC_BUS1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_g3aa[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D_G3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_dns[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D_DNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_ipp[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D_IPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d0_mcsc[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D0_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d0_tnr[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D0_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d1_mcsc[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D1_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d1_tnr[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D1_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_trex_p_bus1[] = {
	GOUT_BLK_BUS1_UID_TREX_P_BUS1_IPCLKPORT_PCLK,
	GOUT_BLK_BUS1_UID_TREX_P_BUS1_IPCLKPORT_ACLK_P_BUS1,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d0_gdc[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D0_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d1_gdc[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D1_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d2_gdc[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D2_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d2_tnr[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D2_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d3_tnr[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D3_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_dbg_g_bus1[] = {
	GOUT_BLK_BUS1_UID_LHS_DBG_G_BUS1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d2_mcsc[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D2_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d4_tnr[] = {
	GOUT_BLK_BUS1_UID_LHM_AXI_D4_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_disp[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_DISP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_pdp[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_PDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_gdc[] = {
	GOUT_BLK_BUS1_UID_LHS_AXI_P_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_bus2_cmu_bus2[] = {
	CLK_BLK_BUS2_UID_BUS2_CMU_BUS2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_d_bus2[] = {
	GOUT_BLK_BUS2_UID_TREX_D_BUS2_IPCLKPORT_ACLK,
	GOUT_BLK_BUS2_UID_TREX_D_BUS2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_bus2[] = {
	GOUT_BLK_BUS2_UID_SYSREG_BUS2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d0_g3d[] = {
	GOUT_BLK_BUS2_UID_LHM_ACEL_D0_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_bus2[] = {
	GOUT_BLK_BUS2_UID_D_TZPC_BUS2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d1_g3d[] = {
	GOUT_BLK_BUS2_UID_LHM_ACEL_D1_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d2_g3d[] = {
	GOUT_BLK_BUS2_UID_LHM_ACEL_D2_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d3_g3d[] = {
	GOUT_BLK_BUS2_UID_LHM_ACEL_D3_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_g3d0[] = {
	GOUT_BLK_BUS2_UID_SSMT_G3D0_IPCLKPORT_ACLK,
	GOUT_BLK_BUS2_UID_SSMT_G3D0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_tpu[] = {
	GOUT_BLK_BUS2_UID_LHM_AXI_D_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_g3d0[] = {
	GOUT_BLK_BUS2_UID_SYSMMU_G3D0_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_g3d[] = {
	GOUT_BLK_BUS2_UID_LHS_AXI_P_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_tpu[] = {
	GOUT_BLK_BUS2_UID_LHS_AXI_P_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_bus2[] = {
	GOUT_BLK_BUS2_UID_GPC_BUS2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_g3d1[] = {
	GOUT_BLK_BUS2_UID_SSMT_G3D1_IPCLKPORT_ACLK,
	GOUT_BLK_BUS2_UID_SSMT_G3D1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_g3d2[] = {
	GOUT_BLK_BUS2_UID_SSMT_G3D2_IPCLKPORT_ACLK,
	GOUT_BLK_BUS2_UID_SSMT_G3D2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_g3d3[] = {
	GOUT_BLK_BUS2_UID_SSMT_G3D3_IPCLKPORT_ACLK,
	GOUT_BLK_BUS2_UID_SSMT_G3D3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_g3d1[] = {
	GOUT_BLK_BUS2_UID_SYSMMU_G3D1_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ppcfw_g3d0[] = {
	GOUT_BLK_BUS2_UID_PPCFW_G3D0_IPCLKPORT_ACLK,
	GOUT_BLK_BUS2_UID_PPCFW_G3D0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_g3d2[] = {
	GOUT_BLK_BUS2_UID_SYSMMU_G3D2_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysmmu_g3d3[] = {
	GOUT_BLK_BUS2_UID_SYSMMU_G3D3_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ad_apb_sysmmu_g3d0[] = {
	GOUT_BLK_BUS2_UID_AD_APB_SYSMMU_G3D0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_trex_p_bus2[] = {
	GOUT_BLK_BUS2_UID_TREX_P_BUS2_IPCLKPORT_PCLK,
	GOUT_BLK_BUS2_UID_TREX_P_BUS2_IPCLKPORT_ACLK_P_BUS2,
};
enum clk_id cmucal_vclk_ip_lhs_dbg_g_bus2[] = {
	GOUT_BLK_BUS2_UID_LHS_DBG_G_BUS2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_bus2_s0[] = {
	GOUT_BLK_BUS2_UID_PPC_EVENT_BUS2_S0_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_EVENT_BUS2_S0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_bus2_s1[] = {
	GOUT_BLK_BUS2_UID_PPC_EVENT_BUS2_S1_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_EVENT_BUS2_S1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_bus2_s2[] = {
	GOUT_BLK_BUS2_UID_PPC_EVENT_BUS2_S2_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_EVENT_BUS2_S2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_bus2_s3[] = {
	GOUT_BLK_BUS2_UID_PPC_EVENT_BUS2_S3_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_EVENT_BUS2_S3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_g3d0[] = {
	GOUT_BLK_BUS2_UID_PPC_EVENT_G3D0_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_EVENT_G3D0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_g3d1[] = {
	GOUT_BLK_BUS2_UID_PPC_EVENT_G3D1_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_EVENT_G3D1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_g3d2[] = {
	GOUT_BLK_BUS2_UID_PPC_EVENT_G3D2_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_EVENT_G3D2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_g3d3[] = {
	GOUT_BLK_BUS2_UID_PPC_EVENT_G3D3_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_EVENT_G3D3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_event_tpu[] = {
	GOUT_BLK_BUS2_UID_PPC_EVENT_TPU_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_EVENT_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cycle_bus2_s0[] = {
	GOUT_BLK_BUS2_UID_PPC_CYCLE_BUS2_S0_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_CYCLE_BUS2_S0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cycle_g3d0[] = {
	GOUT_BLK_BUS2_UID_PPC_CYCLE_G3D0_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_CYCLE_G3D0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cycle_tpu[] = {
	GOUT_BLK_BUS2_UID_PPC_CYCLE_TPU_IPCLKPORT_CLK,
	GOUT_BLK_BUS2_UID_PPC_CYCLE_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppcfw_g3d1[] = {
	CLK_BLK_BUS2_UID_PPCFW_G3D1_IPCLKPORT_ACLK,
	GOUT_BLK_BUS2_UID_PPCFW_G3D1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_core_cmu_core[] = {
	CLK_BLK_CORE_UID_CORE_CMU_CORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_core[] = {
	GOUT_BLK_CORE_UID_SYSREG_CORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_trex_p_core[] = {
	GOUT_BLK_CORE_UID_TREX_P_CORE_IPCLKPORT_PCLK,
	GOUT_BLK_CORE_UID_TREX_P_CORE_IPCLKPORT_ACLK_D_CORE,
	GOUT_BLK_CORE_UID_TREX_P_CORE_IPCLKPORT_ACLK_P_CORE,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_mif1[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_MIF1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_peric0[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_PERIC0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_mif0[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_MIF0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ace_d0_cluster0[] = {
	GOUT_BLK_CORE_UID_LHM_ACE_D0_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ace_d1_cluster0[] = {
	GOUT_BLK_CORE_UID_LHM_ACE_D1_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_trex_d_core[] = {
	GOUT_BLK_CORE_UID_TREX_D_CORE_IPCLKPORT_PCLK,
	GOUT_BLK_CORE_UID_TREX_D_CORE_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_peric1[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_PERIC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_cci[] = {
	GOUT_BLK_CORE_UID_AD_APB_CCI_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_apm[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_APM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_core[] = {
	GOUT_BLK_CORE_UID_D_TZPC_CORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_misc[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_MISC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_mif2[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_MIF2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_mif3[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_MIF3_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_bdu[] = {
	GOUT_BLK_CORE_UID_BDU_IPCLKPORT_I_CLK,
	GOUT_BLK_CORE_UID_BDU_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_core[] = {
	GOUT_BLK_CORE_UID_GPC_CORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t_bdu[] = {
	GOUT_BLK_CORE_UID_LHS_ATB_T_BDU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t_slc[] = {
	GOUT_BLK_CORE_UID_LHS_ATB_T_SLC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_ace_cpucl0[] = {
	GOUT_BLK_CORE_UID_PPMU_ACE_CPUCL0_IPCLKPORT_ACLK,
	GOUT_BLK_CORE_UID_PPMU_ACE_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_ace_cpucl1[] = {
	GOUT_BLK_CORE_UID_PPMU_ACE_CPUCL1_IPCLKPORT_ACLK,
	GOUT_BLK_CORE_UID_PPMU_ACE_CPUCL1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sfr_apbif_cmu_topc[] = {
	GOUT_BLK_CORE_UID_SFR_APBIF_CMU_TOPC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_bus2_m0_event[] = {
	GOUT_BLK_CORE_UID_PPC_BUS2_M0_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_BUS2_M0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_bus2_m1_event[] = {
	GOUT_BLK_CORE_UID_PPC_BUS2_M1_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_BUS2_M1_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_bus2_m2_event[] = {
	GOUT_BLK_CORE_UID_PPC_BUS2_M2_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_BUS2_M2_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_bus2_m3_event[] = {
	GOUT_BLK_CORE_UID_PPC_BUS2_M3_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_BUS2_M3_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_bus0_m0_event[] = {
	GOUT_BLK_CORE_UID_PPC_BUS0_M0_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_BUS0_M0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cpucl0_d0_cycle[] = {
	GOUT_BLK_CORE_UID_PPC_CPUCL0_D0_CYCLE_IPCLKPORT_PCLK,
	CLK_BLK_CORE_UID_PPC_CPUCL0_D0_CYCLE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_slc_cb[] = {
	GOUT_BLK_CORE_UID_SLC_CB_IPCLKPORT_I_ACLK_CB,
};
enum clk_id cmucal_vclk_ip_cci[] = {
	GOUT_BLK_CORE_UID_CCI_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lhm_acel_d_eh[] = {
	GOUT_BLK_CORE_UID_LHM_ACEL_D_EH_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_cpucl0[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_eh[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_EH_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_eh_cycle[] = {
	GOUT_BLK_CORE_UID_PPC_EH_CYCLE_IPCLKPORT_PCLK,
	CLK_BLK_CORE_UID_PPC_EH_CYCLE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_gic[] = {
	GOUT_BLK_CORE_UID_LHS_AXI_P_GIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_io_event[] = {
	GOUT_BLK_CORE_UID_PPC_IO_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_IO_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_eh_event[] = {
	GOUT_BLK_CORE_UID_PPC_EH_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_EH_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cpucl0_d0_event[] = {
	GOUT_BLK_CORE_UID_PPC_CPUCL0_D0_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_CPUCL0_D0_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cci_m1_event[] = {
	GOUT_BLK_CORE_UID_PPC_CCI_M1_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_CCI_M1_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cci_m2_event[] = {
	GOUT_BLK_CORE_UID_PPC_CCI_M2_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_CCI_M2_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cci_m3_event[] = {
	GOUT_BLK_CORE_UID_PPC_CCI_M3_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_CCI_M3_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_cci_m4_event[] = {
	GOUT_BLK_CORE_UID_PPC_CCI_M4_EVENT_IPCLKPORT_CLK,
	GOUT_BLK_CORE_UID_PPC_CCI_M4_EVENT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppc_io_cycle[] = {
	GOUT_BLK_CORE_UID_PPC_IO_CYCLE_IPCLKPORT_PCLK,
	CLK_BLK_CORE_UID_PPC_IO_CYCLE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_cci_m1_cycle[] = {
	GOUT_BLK_CORE_UID_PPC_CCI_M1_CYCLE_IPCLKPORT_PCLK,
	CLK_BLK_CORE_UID_PPC_CCI_M1_CYCLE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_bus2_m0_cycle[] = {
	GOUT_BLK_CORE_UID_PPC_BUS2_M0_CYCLE_IPCLKPORT_PCLK,
	CLK_BLK_CORE_UID_PPC_BUS2_M0_CYCLE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_bus0_m0_cycle[] = {
	GOUT_BLK_CORE_UID_PPC_BUS0_M0_CYCLE_IPCLKPORT_PCLK,
	CLK_BLK_CORE_UID_PPC_BUS0_M0_CYCLE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_dbg_cc[] = {
	GOUT_BLK_CORE_UID_PPC_DBG_CC_IPCLKPORT_PCLK,
	GOUT_BLK_CORE_UID_PPC_DBG_CC_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_mpace_asb_d0_mif[] = {
	GOUT_BLK_CORE_UID_MPACE_ASB_D0_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mpace_asb_d1_mif[] = {
	GOUT_BLK_CORE_UID_MPACE_ASB_D1_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mpace_asb_d2_mif[] = {
	GOUT_BLK_CORE_UID_MPACE_ASB_D2_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mpace_asb_d3_mif[] = {
	GOUT_BLK_CORE_UID_MPACE_ASB_D3_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppc_cpucl0_d1_event[] = {
	GOUT_BLK_CORE_UID_PPC_CPUCL0_D1_EVENT_IPCLKPORT_PCLK,
	GOUT_BLK_CORE_UID_PPC_CPUCL0_D1_EVENT_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_slc_ch[] = {
	GOUT_BLK_CORE_UID_SLC_CH_IPCLKPORT_I_ACLK,
	GOUT_BLK_CORE_UID_SLC_CH_IPCLKPORT_I_DCLK,
};
enum clk_id cmucal_vclk_ip_slc_ch1[] = {
	GOUT_BLK_CORE_UID_SLC_CH1_IPCLKPORT_I_ACLK,
	GOUT_BLK_CORE_UID_SLC_CH1_IPCLKPORT_I_DCLK,
};
enum clk_id cmucal_vclk_ip_slc_ch2[] = {
	GOUT_BLK_CORE_UID_SLC_CH2_IPCLKPORT_I_ACLK,
	GOUT_BLK_CORE_UID_SLC_CH2_IPCLKPORT_I_DCLK,
};
enum clk_id cmucal_vclk_ip_slc_ch3[] = {
	GOUT_BLK_CORE_UID_SLC_CH3_IPCLKPORT_I_ACLK,
	GOUT_BLK_CORE_UID_SLC_CH3_IPCLKPORT_I_DCLK,
};
enum clk_id cmucal_vclk_ip_cpe425[] = {
	GOUT_BLK_CORE_UID_CPE425_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_gray2bin_atb_tsvalue[] = {
	GOUT_BLK_CORE_UID_GRAY2BIN_ATB_TSVALUE_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_core[] = {
	GOUT_BLK_CORE_UID_LHM_AXI_G_CORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_SYSREG_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_hpm_apbif_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_HPM_APBIF_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_cssys[] = {
	GOUT_BLK_CPUCL0_UID_CSSYS_IPCLKPORT_PCLKDBG,
	GOUT_BLK_CPUCL0_UID_CSSYS_IPCLKPORT_ATCLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t_aoc[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T_AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t_bdu[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T_BDU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t0_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T0_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t6_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T6_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t1_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T1_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t7_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T7_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t2_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T2_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t3_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T3_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_AXI_P_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ace_d0_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ACE_D0_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t0_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ATB_T0_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t1_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ATB_T1_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t2_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ATB_T2_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t3_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ATB_T3_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_adm_apb_g_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_ADM_APB_G_CLUSTER0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_cpucl0_cmu_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_CPUCL0_CMU_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_cluster0[] = {
	CLK_BLK_CPUCL0_UID_CLUSTER0_IPCLKPORT_PERIPHCLK,
	CLK_BLK_CPUCL0_UID_CLUSTER0_IPCLKPORT_SCLK,
	CLK_BLK_CPUCL0_UID_CLUSTER0_IPCLKPORT_PCLK,
	CLK_BLK_CPUCL0_UID_CLUSTER0_IPCLKPORT_ATCLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t4_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T4_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t5_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T5_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ace_d1_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ACE_D1_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t4_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ATB_T4_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t5_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ATB_T5_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_D_TZPC_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_g_int_cssys[] = {
	GOUT_BLK_CPUCL0_UID_LHS_AXI_G_INT_CSSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_int_cssys[] = {
	GOUT_BLK_CPUCL0_UID_LHM_AXI_G_INT_CSSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_p_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_XIU_P_CPUCL0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_hpm_cpucl0_1[] = {
	CLK_BLK_CPUCL0_UID_HPM_CPUCL0_1_IPCLKPORT_HPM_TARGETCLK_C,
};
enum clk_id cmucal_vclk_ip_hpm_cpucl0_0[] = {
	CLK_BLK_CPUCL0_UID_HPM_CPUCL0_0_IPCLKPORT_HPM_TARGETCLK_C,
};
enum clk_id cmucal_vclk_ip_apb_async_p_cssys_0[] = {
	GOUT_BLK_CPUCL0_UID_APB_ASYNC_P_CSSYS_0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_bps_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_BPS_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t6_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ATB_T6_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t_gsa[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T_GSA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t_slc[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T_SLC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_g_cssys[] = {
	GOUT_BLK_CPUCL0_UID_LHS_AXI_G_CSSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_GPC_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_dbgcore[] = {
	GOUT_BLK_CPUCL0_UID_LHM_AXI_G_DBGCORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_g_int_dbgcore[] = {
	GOUT_BLK_CPUCL0_UID_LHS_AXI_G_INT_DBGCORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_dp_cssys[] = {
	GOUT_BLK_CPUCL0_UID_XIU_DP_CSSYS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_int_dbgcore[] = {
	GOUT_BLK_CPUCL0_UID_LHM_AXI_G_INT_DBGCORE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_iri_giccpu[] = {
	GOUT_BLK_CPUCL0_UID_LHM_AST_IRI_GICCPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_icc_cpugic[] = {
	GOUT_BLK_CPUCL0_UID_LHS_AST_ICC_CPUGIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_SSMT_CPUCL0_IPCLKPORT_PCLK,
	GOUT_BLK_CPUCL0_UID_SSMT_CPUCL0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_s2_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_SYSMMU_S2_CPUCL0_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_lhs_axi_g_etr_hsi0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_AXI_G_ETR_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_int_hsi0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_AXI_G_INT_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_apb_async_p_cssys_4[] = {
	GOUT_BLK_CPUCL0_UID_APB_ASYNC_P_CSSYS_4_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t0_tpucpucl0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T0_TPUCPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_t1_tpucpucl0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_ATB_T1_TPUCPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_g_int_hsi0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_AXI_G_INT_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_trex_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_TREX_CPUCL0_IPCLKPORT_CLK,
	GOUT_BLK_CPUCL0_UID_TREX_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_g_int_stm[] = {
	GOUT_BLK_CPUCL0_UID_LHS_AXI_G_INT_STM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t7_cluster0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ATB_T7_CLUSTER0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_int_stm[] = {
	GOUT_BLK_CPUCL0_UID_LHM_AXI_G_INT_STM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_cpucl1[] = {
	CLK_BLK_CPUCL1_UID_CPUCL1_IPCLKPORT_CK_IN_DD_CTRL_ENYO_1,
	CLK_BLK_CPUCL1_UID_CPUCL1_IPCLKPORT_CK_IN_DD_CTRL_ENYO_0,
	CLK_BLK_CPUCL1_UID_CPUCL1_IPCLKPORT_CORECLK,
	GOUT_BLK_CPUCL1_UID_CPUCL1_IPCLKPORT_DDD_CLK_IN_OCC,
};
enum clk_id cmucal_vclk_ip_cpucl1_cmu_cpucl1[] = {
	CLK_BLK_CPUCL1_UID_CPUCL1_CMU_CPUCL1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_cpucl2_cmu_cpucl2[] = {
	CLK_BLK_CPUCL2_UID_CPUCL2_CMU_CPUCL2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_cmu_cpucl2_shortstop[] = {
	GOUT_BLK_CPUCL2_UID_CMU_CPUCL2_SHORTSTOP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_cpucl2[] = {
	CLK_BLK_CPUCL2_UID_CPUCL2_IPCLKPORT_CK_IN_DD_CTRL_HERA_0,
	CLK_BLK_CPUCL2_UID_CPUCL2_IPCLKPORT_CK_IN_DD_CTRL_HERA_1,
	CLK_BLK_CPUCL2_UID_CPUCL2_IPCLKPORT_CORECLK,
	GOUT_BLK_CPUCL2_UID_CPUCL2_IPCLKPORT_DDD_CLK_IN_OCC,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d0_csis[] = {
	GOUT_BLK_CSIS_UID_LHS_AXI_D0_CSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_csis[] = {
	GOUT_BLK_CSIS_UID_LHM_AXI_P_CSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_csis[] = {
	GOUT_BLK_CSIS_UID_SYSREG_CSIS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_csis_cmu_csis[] = {
	CLK_BLK_CSIS_UID_CSIS_CMU_CSIS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_zotf2_ippcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_ZOTF2_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_mipi_phy_link_wrap[] = {
	GOUT_BLK_CSIS_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_ACLK_CSIS1,
	GOUT_BLK_CSIS_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_ACLK_CSIS2,
	GOUT_BLK_CSIS_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_ACLK_CSIS3,
	GOUT_BLK_CSIS_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_ACLK_CSIS5,
	GOUT_BLK_CSIS_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_ACLK_CSIS4,
	GOUT_BLK_CSIS_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_ACLK_CSIS6,
	GOUT_BLK_CSIS_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_ACLK_CSIS7,
	GOUT_BLK_CSIS_UID_MIPI_PHY_LINK_WRAP_IPCLKPORT_ACLK_CSIS0,
};
enum clk_id cmucal_vclk_ip_d_tzpc_csis[] = {
	GOUT_BLK_CSIS_UID_D_TZPC_CSIS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_zotf1_ippcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_ZOTF1_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0[] = {
	GOUT_BLK_CSIS_UID_PPMU_D0_IPCLKPORT_PCLK,
	GOUT_BLK_CSIS_UID_PPMU_D0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_zotf0_ippcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_ZOTF0_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_sotf0_ippcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_SOTF0_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_sotf1_ippcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_SOTF1_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_sotf2_ippcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_SOTF2_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf0_csispdp[] = {
	GOUT_BLK_CSIS_UID_LHS_AST_OTF0_CSISPDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf1_csispdp[] = {
	GOUT_BLK_CSIS_UID_LHS_AST_OTF1_CSISPDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf2_csispdp[] = {
	GOUT_BLK_CSIS_UID_LHS_AST_OTF2_CSISPDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_csis[] = {
	GOUT_BLK_CSIS_UID_GPC_CSIS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_csis0[] = {
	GOUT_BLK_CSIS_UID_AD_APB_CSIS0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ppmu_d1[] = {
	GOUT_BLK_CSIS_UID_PPMU_D1_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_PPMU_D1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d0_csis[] = {
	GOUT_BLK_CSIS_UID_SYSMMU_D0_CSIS_IPCLKPORT_CLK_S2,
	GOUT_BLK_CSIS_UID_SYSMMU_D0_CSIS_IPCLKPORT_CLK_S1,
};
enum clk_id cmucal_vclk_ip_sysmmu_d1_csis[] = {
	GOUT_BLK_CSIS_UID_SYSMMU_D1_CSIS_IPCLKPORT_CLK_S1,
	GOUT_BLK_CSIS_UID_SYSMMU_D1_CSIS_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ssmt_d1[] = {
	GOUT_BLK_CSIS_UID_SSMT_D1_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_SSMT_D1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0[] = {
	GOUT_BLK_CSIS_UID_SSMT_D0_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_SSMT_D0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_zsl1[] = {
	GOUT_BLK_CSIS_UID_QE_ZSL1_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_ZSL1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_zsl2[] = {
	GOUT_BLK_CSIS_UID_QE_ZSL2_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_ZSL2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_zsl0[] = {
	GOUT_BLK_CSIS_UID_QE_ZSL0_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_ZSL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_strp0[] = {
	GOUT_BLK_CSIS_UID_QE_STRP0_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_STRP0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_csis[] = {
	GOUT_BLK_CSIS_UID_XIU_D0_CSIS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_csis[] = {
	GOUT_BLK_CSIS_UID_XIU_D1_CSIS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_vo_mcsccsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_VO_MCSCCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d1_csis[] = {
	GOUT_BLK_CSIS_UID_LHS_AXI_D1_CSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf0_pdpcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_OTF0_PDPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf1_pdpcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_OTF1_PDPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf2_pdpcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AST_OTF2_PDPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_vo_csispdp[] = {
	GOUT_BLK_CSIS_UID_LHS_AST_VO_CSISPDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_pdpcsis[] = {
	GOUT_BLK_CSIS_UID_LHM_AXI_D_PDPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_qe_strp2[] = {
	GOUT_BLK_CSIS_UID_QE_STRP2_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_STRP2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_strp1[] = {
	GOUT_BLK_CSIS_UID_QE_STRP1_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_STRP1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d2_csis[] = {
	GOUT_BLK_CSIS_UID_XIU_D2_CSIS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_csisx8[] = {
	GOUT_BLK_CSIS_UID_CSISX8_IPCLKPORT_ACLK_EBUF,
	GOUT_BLK_CSIS_UID_CSISX8_IPCLKPORT_ACLK_CSIS_DMA,
	GOUT_BLK_CSIS_UID_CSISX8_IPCLKPORT_ACLK_C2_CSIS,
};
enum clk_id cmucal_vclk_ip_qe_csis_dma0[] = {
	GOUT_BLK_CSIS_UID_QE_CSIS_DMA0_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_CSIS_DMA0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_csis_dma1[] = {
	GOUT_BLK_CSIS_UID_QE_CSIS_DMA1_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_CSIS_DMA1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_csis_dma2[] = {
	GOUT_BLK_CSIS_UID_QE_CSIS_DMA2_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_CSIS_DMA2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_csis_dma3[] = {
	GOUT_BLK_CSIS_UID_QE_CSIS_DMA3_IPCLKPORT_ACLK,
	GOUT_BLK_CSIS_UID_QE_CSIS_DMA3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_disp_cmu_disp[] = {
	CLK_BLK_DISP_UID_DISP_CMU_DISP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_decon_main[] = {
	GOUT_BLK_DISP_UID_AD_APB_DECON_MAIN_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_dpub[] = {
	GOUT_BLK_DISP_UID_DPUB_IPCLKPORT_ACLK_DECON,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_disp[] = {
	CLK_BLK_DISP_UID_LHM_AXI_P_DISP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_disp[] = {
	GOUT_BLK_DISP_UID_D_TZPC_DISP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_disp[] = {
	GOUT_BLK_DISP_UID_GPC_DISP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_disp[] = {
	GOUT_BLK_DISP_UID_SYSREG_DISP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_dns[] = {
	GOUT_BLK_DNS_UID_AD_APB_DNS_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_dns[] = {
	GOUT_BLK_DNS_UID_D_TZPC_DNS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_dns[] = {
	GOUT_BLK_DNS_UID_DNS_IPCLKPORT_I_CLK,
	GOUT_BLK_DNS_UID_DNS_IPCLKPORT_I_CLK_C2COM,
	GOUT_BLK_DNS_UID_DNS_IPCLKPORT_I_CLK_C2COM,
};
enum clk_id cmucal_vclk_ip_gpc_dns[] = {
	GOUT_BLK_DNS_UID_GPC_DNS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_dns[] = {
	GOUT_BLK_DNS_UID_LHM_AXI_P_DNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_dns[] = {
	GOUT_BLK_DNS_UID_LHS_AXI_D_DNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_dns[] = {
	GOUT_BLK_DNS_UID_PPMU_DNS_IPCLKPORT_ACLK,
	GOUT_BLK_DNS_UID_PPMU_DNS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_dns[] = {
	GOUT_BLK_DNS_UID_SSMT_DNS_IPCLKPORT_ACLK,
	GOUT_BLK_DNS_UID_SSMT_DNS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_dns[] = {
	GOUT_BLK_DNS_UID_SYSMMU_DNS_IPCLKPORT_CLK_S1,
	GOUT_BLK_DNS_UID_SYSMMU_DNS_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysreg_dns[] = {
	GOUT_BLK_DNS_UID_SYSREG_DNS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf0_dnsitp[] = {
	GOUT_BLK_DNS_UID_LHS_AST_OTF0_DNSITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf1_dnsitp[] = {
	GOUT_BLK_DNS_UID_LHS_AST_OTF1_DNSITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf2_dnsitp[] = {
	GOUT_BLK_DNS_UID_LHS_AST_OTF2_DNSITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf0_dnsmcsc[] = {
	GOUT_BLK_DNS_UID_LHS_AST_OTF0_DNSMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf1_dnsmcsc[] = {
	GOUT_BLK_DNS_UID_LHS_AST_OTF1_DNSMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf2_dnsmcsc[] = {
	GOUT_BLK_DNS_UID_LHS_AST_OTF2_DNSMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf_itpdns[] = {
	GOUT_BLK_DNS_UID_LHM_AST_OTF_ITPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf_dnsgdc[] = {
	GOUT_BLK_DNS_UID_LHS_AST_OTF_DNSGDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_ctl_itpdns[] = {
	GOUT_BLK_DNS_UID_LHM_AST_CTL_ITPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_ctl_dnsitp[] = {
	GOUT_BLK_DNS_UID_LHS_AST_CTL_DNSITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_vo_ippdns[] = {
	GOUT_BLK_DNS_UID_LHM_AST_VO_IPPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_vo_dnstnr[] = {
	GOUT_BLK_DNS_UID_LHS_AST_VO_DNSTNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_pdpdns[] = {
	GOUT_BLK_DNS_UID_LHM_AXI_D_PDPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_dns[] = {
	GOUT_BLK_DNS_UID_XIU_D_DNS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_ippdns[] = {
	GOUT_BLK_DNS_UID_LHM_AXI_D_IPPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_mcscdns[] = {
	GOUT_BLK_DNS_UID_LHM_AXI_D_MCSCDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_qe_dns[] = {
	GOUT_BLK_DNS_UID_QE_DNS_IPCLKPORT_ACLK,
	GOUT_BLK_DNS_UID_QE_DNS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf_ippdns[] = {
	GOUT_BLK_DNS_UID_LHM_AST_OTF_IPPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_dns_cmu_dns[] = {
	CLK_BLK_DNS_UID_DNS_CMU_DNS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_dpu_cmu_dpu[] = {
	CLK_BLK_DPU_UID_DPU_CMU_DPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_dpu[] = {
	GOUT_BLK_DPU_UID_SYSREG_DPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_dpud0[] = {
	GOUT_BLK_DPU_UID_SYSMMU_DPUD0_IPCLKPORT_CLK_S1,
	GOUT_BLK_DPU_UID_SYSMMU_DPUD0_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_dpu[] = {
	GOUT_BLK_DPU_UID_LHM_AXI_P_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d1_dpu[] = {
	GOUT_BLK_DPU_UID_LHS_AXI_D1_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d2_dpu[] = {
	GOUT_BLK_DPU_UID_LHS_AXI_D2_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_dpud2[] = {
	GOUT_BLK_DPU_UID_SYSMMU_DPUD2_IPCLKPORT_CLK_S1,
	GOUT_BLK_DPU_UID_SYSMMU_DPUD2_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysmmu_dpud1[] = {
	GOUT_BLK_DPU_UID_SYSMMU_DPUD1_IPCLKPORT_CLK_S1,
	GOUT_BLK_DPU_UID_SYSMMU_DPUD1_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ppmu_dpud0[] = {
	GOUT_BLK_DPU_UID_PPMU_DPUD0_IPCLKPORT_ACLK,
	GOUT_BLK_DPU_UID_PPMU_DPUD0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_dpud1[] = {
	GOUT_BLK_DPU_UID_PPMU_DPUD1_IPCLKPORT_ACLK,
	GOUT_BLK_DPU_UID_PPMU_DPUD1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_dpud2[] = {
	GOUT_BLK_DPU_UID_PPMU_DPUD2_IPCLKPORT_ACLK,
	GOUT_BLK_DPU_UID_PPMU_DPUD2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d0_dpu[] = {
	GOUT_BLK_DPU_UID_LHS_AXI_D0_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_dpuf[] = {
	GOUT_BLK_DPU_UID_DPUF_IPCLKPORT_ACLK_DMA,
	GOUT_BLK_DPU_UID_DPUF_IPCLKPORT_ACLK_DPP,
};
enum clk_id cmucal_vclk_ip_d_tzpc_dpu[] = {
	GOUT_BLK_DPU_UID_D_TZPC_DPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_dpu_dma[] = {
	GOUT_BLK_DPU_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ssmt_dpu0[] = {
	GOUT_BLK_DPU_UID_SSMT_DPU0_IPCLKPORT_ACLK,
	GOUT_BLK_DPU_UID_SSMT_DPU0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_dpu1[] = {
	GOUT_BLK_DPU_UID_SSMT_DPU1_IPCLKPORT_ACLK,
	GOUT_BLK_DPU_UID_SSMT_DPU1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_dpu2[] = {
	GOUT_BLK_DPU_UID_SSMT_DPU2_IPCLKPORT_ACLK,
	GOUT_BLK_DPU_UID_SSMT_DPU2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_dpu[] = {
	GOUT_BLK_DPU_UID_GPC_DPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_eh_cmu_eh[] = {
	CLK_BLK_EH_UID_EH_CMU_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_as_p_sysmmu_s2_eh[] = {
	GOUT_BLK_EH_UID_AS_P_SYSMMU_S2_EH_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_eh[] = {
	GOUT_BLK_EH_UID_D_TZPC_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_eh[] = {
	GOUT_BLK_EH_UID_GPC_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_eh[] = {
	GOUT_BLK_EH_UID_LHM_AXI_P_EH_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_acel_d_eh[] = {
	GOUT_BLK_EH_UID_LHS_ACEL_D_EH_IPCLKPORT_I_CLK,
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
enum clk_id cmucal_vclk_ip_sysmmu_eh[] = {
	GOUT_BLK_EH_UID_SYSMMU_EH_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysreg_eh[] = {
	GOUT_BLK_EH_UID_SYSREG_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_eh[] = {
	GOUT_BLK_EH_UID_UASC_EH_IPCLKPORT_ACLK,
	GOUT_BLK_EH_UID_UASC_EH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_axi_p_eh[] = {
	CLK_BLK_EH_UID_AD_AXI_P_EH_IPCLKPORT_ACLKM,
};
enum clk_id cmucal_vclk_ip_qe_eh[] = {
	CLK_BLK_EH_UID_QE_EH_IPCLKPORT_ACLK,
	CLK_BLK_EH_UID_QE_EH_IPCLKPORT_PCLK,
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
enum clk_id cmucal_vclk_ip_sysmmu_d0_g2d[] = {
	GOUT_BLK_G2D_UID_SYSMMU_D0_G2D_IPCLKPORT_CLK_S1,
	GOUT_BLK_G2D_UID_SYSMMU_D0_G2D_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysreg_g2d[] = {
	GOUT_BLK_G2D_UID_SYSREG_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d0_g2d[] = {
	GOUT_BLK_G2D_UID_LHS_AXI_D0_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d1_g2d[] = {
	GOUT_BLK_G2D_UID_LHS_AXI_D1_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d2_g2d[] = {
	GOUT_BLK_G2D_UID_SYSMMU_D2_G2D_IPCLKPORT_CLK_S1,
	GOUT_BLK_G2D_UID_SYSMMU_D2_G2D_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_g2d[] = {
	GOUT_BLK_G2D_UID_PPMU_D2_G2D_IPCLKPORT_ACLK,
	GOUT_BLK_G2D_UID_PPMU_D2_G2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_acel_d2_g2d[] = {
	GOUT_BLK_G2D_UID_LHS_ACEL_D2_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_g2d[] = {
	GOUT_BLK_G2D_UID_SSMT_D0_G2D_IPCLKPORT_PCLK,
	GOUT_BLK_G2D_UID_SSMT_D0_G2D_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_g2d[] = {
	GOUT_BLK_G2D_UID_G2D_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d1_g2d[] = {
	GOUT_BLK_G2D_UID_SYSMMU_D1_G2D_IPCLKPORT_CLK_S1,
	GOUT_BLK_G2D_UID_SYSMMU_D1_G2D_IPCLKPORT_CLK_S2,
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
enum clk_id cmucal_vclk_ip_lhm_axi_p_g2d[] = {
	GOUT_BLK_G2D_UID_LHM_AXI_P_G2D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_as_apb_g2d[] = {
	GOUT_BLK_G2D_UID_AS_APB_G2D_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_as_apb_jpeg[] = {
	GOUT_BLK_G2D_UID_AS_APB_JPEG_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_g3aa[] = {
	GOUT_BLK_G3AA_UID_LHM_AXI_P_G3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_g3aa[] = {
	GOUT_BLK_G3AA_UID_LHS_AXI_D_G3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_apb_async_top_g3aa[] = {
	GOUT_BLK_G3AA_UID_APB_ASYNC_TOP_G3AA_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_sysreg_g3aa[] = {
	GOUT_BLK_G3AA_UID_SYSREG_G3AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_g3aa_cmu_g3aa[] = {
	CLK_BLK_G3AA_UID_G3AA_CMU_G3AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_g3aa[] = {
	GOUT_BLK_G3AA_UID_PPMU_G3AA_IPCLKPORT_ACLK,
	GOUT_BLK_G3AA_UID_PPMU_G3AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_g3aa[] = {
	GOUT_BLK_G3AA_UID_D_TZPC_G3AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_g3aa[] = {
	GOUT_BLK_G3AA_UID_GPC_G3AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_g3aa[] = {
	GOUT_BLK_G3AA_UID_G3AA_IPCLKPORT_ACLK_AXIM,
};
enum clk_id cmucal_vclk_ip_ssmt_g3aa[] = {
	GOUT_BLK_G3AA_UID_SSMT_G3AA_IPCLKPORT_ACLK,
	GOUT_BLK_G3AA_UID_SSMT_G3AA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_g3aa[] = {
	GOUT_BLK_G3AA_UID_SYSMMU_G3AA_IPCLKPORT_CLK_S1,
	GOUT_BLK_G3AA_UID_SYSMMU_G3AA_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf0_pdpg3aa[] = {
	GOUT_BLK_G3AA_UID_LHM_AST_OTF0_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_yotf0_pdpg3aa[] = {
	GOUT_BLK_G3AA_UID_LHM_AST_YOTF0_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf1_pdpg3aa[] = {
	GOUT_BLK_G3AA_UID_LHM_AST_OTF1_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf2_pdpg3aa[] = {
	GOUT_BLK_G3AA_UID_LHM_AST_OTF2_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_yotf1_pdpg3aa[] = {
	GOUT_BLK_G3AA_UID_LHM_AST_YOTF1_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_g3d[] = {
	GOUT_BLK_G3D_UID_LHM_AXI_P_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_busif_hpmg3d[] = {
	GOUT_BLK_G3D_UID_BUSIF_HPMG3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_hpm_g3d[] = {
	CLK_BLK_G3D_UID_HPM_G3D_IPCLKPORT_HPM_TARGETCLK_C,
};
enum clk_id cmucal_vclk_ip_sysreg_g3d[] = {
	GOUT_BLK_G3D_UID_SYSREG_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_g3d_cmu_g3d[] = {
	CLK_BLK_G3D_UID_G3D_CMU_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_p_int_g3d[] = {
	GOUT_BLK_G3D_UID_LHS_AXI_P_INT_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpu[] = {
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_STACKS,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_STACKS,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_COREGROUP,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK_COREGROUP,
	CLK_BLK_G3D_UID_GPU_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_int_g3d[] = {
	GOUT_BLK_G3D_UID_LHM_AXI_P_INT_G3D_IPCLKPORT_I_CLK,
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
enum clk_id cmucal_vclk_ip_asb_g3d[] = {
	CLK_BLK_G3D_UID_ASB_G3D_IPCLKPORT_CLK_LH,
};
enum clk_id cmucal_vclk_ip_gdc_cmu_gdc[] = {
	CLK_BLK_GDC_UID_GDC_CMU_GDC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_gdc0[] = {
	GOUT_BLK_GDC_UID_AD_APB_GDC0_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ad_apb_gdc1[] = {
	GOUT_BLK_GDC_UID_AD_APB_GDC1_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ad_apb_scsc[] = {
	GOUT_BLK_GDC_UID_AD_APB_SCSC_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_gdc[] = {
	GOUT_BLK_GDC_UID_D_TZPC_GDC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gdc0[] = {
	GOUT_BLK_GDC_UID_GDC0_IPCLKPORT_CLK,
	GOUT_BLK_GDC_UID_GDC0_IPCLKPORT_C2CLK,
};
enum clk_id cmucal_vclk_ip_gdc1[] = {
	GOUT_BLK_GDC_UID_GDC1_IPCLKPORT_CLK,
	GOUT_BLK_GDC_UID_GDC1_IPCLKPORT_C2CLK,
};
enum clk_id cmucal_vclk_ip_gpc_gdc[] = {
	GOUT_BLK_GDC_UID_GPC_GDC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d2_gdc[] = {
	GOUT_BLK_GDC_UID_LHS_AXI_D2_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_gdc[] = {
	GOUT_BLK_GDC_UID_PPMU_D0_GDC_IPCLKPORT_PCLK,
	GOUT_BLK_GDC_UID_PPMU_D0_GDC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_gdc[] = {
	GOUT_BLK_GDC_UID_PPMU_D1_GDC_IPCLKPORT_PCLK,
	GOUT_BLK_GDC_UID_PPMU_D1_GDC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_scsc[] = {
	GOUT_BLK_GDC_UID_SCSC_IPCLKPORT_CLK,
	GOUT_BLK_GDC_UID_SCSC_IPCLKPORT_C2CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_gdc[] = {
	GOUT_BLK_GDC_UID_SSMT_D0_GDC_IPCLKPORT_PCLK,
	GOUT_BLK_GDC_UID_SSMT_D0_GDC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_gdc[] = {
	GOUT_BLK_GDC_UID_SSMT_D1_GDC_IPCLKPORT_PCLK,
	GOUT_BLK_GDC_UID_SSMT_D1_GDC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d_scsc[] = {
	GOUT_BLK_GDC_UID_SSMT_D_SCSC_IPCLKPORT_ACLK,
	GOUT_BLK_GDC_UID_SSMT_D_SCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d2_gdc[] = {
	GOUT_BLK_GDC_UID_SYSMMU_D2_GDC_IPCLKPORT_CLK_S1,
	GOUT_BLK_GDC_UID_SYSMMU_D2_GDC_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysreg_gdc[] = {
	GOUT_BLK_GDC_UID_SYSREG_GDC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_int_gdc0gdc1[] = {
	GOUT_BLK_GDC_UID_LHM_AST_INT_GDC0GDC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_int_gdc1scsc[] = {
	GOUT_BLK_GDC_UID_LHM_AST_INT_GDC1SCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf_dnsgdc[] = {
	GOUT_BLK_GDC_UID_LHM_AST_OTF_DNSGDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf_tnrgdc[] = {
	GOUT_BLK_GDC_UID_LHM_AST_OTF_TNRGDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_vo_tnrgdc[] = {
	GOUT_BLK_GDC_UID_LHM_AST_VO_TNRGDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_int_gdc0gdc1[] = {
	GOUT_BLK_GDC_UID_LHS_AST_INT_GDC0GDC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_int_gdc1scsc[] = {
	GOUT_BLK_GDC_UID_LHS_AST_INT_GDC1SCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_vo_gdcmcsc[] = {
	GOUT_BLK_GDC_UID_LHS_AST_VO_GDCMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d0_gdc[] = {
	GOUT_BLK_GDC_UID_SYSMMU_D0_GDC_IPCLKPORT_CLK_S1,
	GOUT_BLK_GDC_UID_SYSMMU_D0_GDC_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysmmu_d1_gdc[] = {
	GOUT_BLK_GDC_UID_SYSMMU_D1_GDC_IPCLKPORT_CLK_S1,
	GOUT_BLK_GDC_UID_SYSMMU_D1_GDC_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d0_gdc[] = {
	GOUT_BLK_GDC_UID_LHS_AXI_D0_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d_scsc[] = {
	GOUT_BLK_GDC_UID_PPMU_D_SCSC_IPCLKPORT_ACLK,
	GOUT_BLK_GDC_UID_PPMU_D_SCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_gdc[] = {
	GOUT_BLK_GDC_UID_XIU_D_GDC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_d1_scsc[] = {
	GOUT_BLK_GDC_UID_QE_D1_SCSC_IPCLKPORT_ACLK,
	GOUT_BLK_GDC_UID_QE_D1_SCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d0_scsc[] = {
	GOUT_BLK_GDC_UID_QE_D0_SCSC_IPCLKPORT_ACLK,
	GOUT_BLK_GDC_UID_QE_D0_SCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_gdc[] = {
	GOUT_BLK_GDC_UID_LHM_AXI_P_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d1_gdc[] = {
	GOUT_BLK_GDC_UID_LHS_AXI_D1_GDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_hsi0_cmu_hsi0[] = {
	CLK_BLK_HSI0_UID_HSI0_CMU_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_usb31drd[] = {
	GOUT_BLK_HSI0_UID_USB31DRD_IPCLKPORT_I_USB31DRD_REF_CLK_40,
	GOUT_BLK_HSI0_UID_USB31DRD_IPCLKPORT_I_USBDPPHY_REF_SOC_PLL,
	GOUT_BLK_HSI0_UID_USB31DRD_IPCLKPORT_ACLK_PHYCTRL,
	GOUT_BLK_HSI0_UID_USB31DRD_IPCLKPORT_I_USBDPPHY_SCL_APB_PCLK,
	GOUT_BLK_HSI0_UID_USB31DRD_IPCLKPORT_I_USBPCS_APB_CLK,
	GOUT_BLK_HSI0_UID_USB31DRD_IPCLKPORT_BUS_CLK_EARLY,
	GOUT_BLK_HSI0_UID_USB31DRD_IPCLKPORT_I_USB20_PHY_REFCLK_26,
	GOUT_BLK_HSI0_UID_USB31DRD_IPCLKPORT_USBDPPHY_UDBG_I_APB_PCLK,
	GOUT_BLK_HSI0_UID_USB31DRD_IPCLKPORT_USBDPPHY_I_ACLK,
	CLK_BLK_HSI0_UID_USB31DRD_IPCLKPORT_I_USB31DRD_SUSPEND_CLK_26,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_etr_hsi0[] = {
	GOUT_BLK_HSI0_UID_LHM_AXI_G_ETR_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_dp_link[] = {
	GOUT_BLK_HSI0_UID_DP_LINK_IPCLKPORT_I_DP_GTC_CLK,
	GOUT_BLK_HSI0_UID_DP_LINK_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_hsi0[] = {
	GOUT_BLK_HSI0_UID_XIU_D0_HSI0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_etr_miu[] = {
	GOUT_BLK_HSI0_UID_ETR_MIU_IPCLKPORT_I_ACLK,
	GOUT_BLK_HSI0_UID_ETR_MIU_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_hsi0_bus0[] = {
	GOUT_BLK_HSI0_UID_PPMU_HSI0_BUS0_IPCLKPORT_ACLK,
	GOUT_BLK_HSI0_UID_PPMU_HSI0_BUS0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_hsi0_aoc[] = {
	GOUT_BLK_HSI0_UID_PPMU_HSI0_AOC_IPCLKPORT_ACLK,
	GOUT_BLK_HSI0_UID_PPMU_HSI0_AOC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_hsi0aoc[] = {
	GOUT_BLK_HSI0_UID_LHS_AXI_D_HSI0AOC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_acel_d_hsi0[] = {
	GOUT_BLK_HSI0_UID_LHS_ACEL_D_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_aochsi0[] = {
	GOUT_BLK_HSI0_UID_LHM_AXI_P_AOCHSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_hsi0[] = {
	GOUT_BLK_HSI0_UID_LHM_AXI_P_HSI0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_hsi0[] = {
	GOUT_BLK_HSI0_UID_GPC_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_hsi0[] = {
	GOUT_BLK_HSI0_UID_D_TZPC_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_usb[] = {
	GOUT_BLK_HSI0_UID_SSMT_USB_IPCLKPORT_PCLK,
	GOUT_BLK_HSI0_UID_SSMT_USB_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_usb[] = {
	GOUT_BLK_HSI0_UID_SYSMMU_USB_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysreg_hsi0[] = {
	GOUT_BLK_HSI0_UID_SYSREG_HSI0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_p_hsi0[] = {
	GOUT_BLK_HSI0_UID_XIU_P_HSI0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_hsi0[] = {
	GOUT_BLK_HSI0_UID_XIU_D1_HSI0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_uasc_hsi0_ctrl[] = {
	GOUT_BLK_HSI0_UID_UASC_HSI0_CTRL_IPCLKPORT_ACLK,
	GOUT_BLK_HSI0_UID_UASC_HSI0_CTRL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_hsi0_link[] = {
	GOUT_BLK_HSI0_UID_UASC_HSI0_LINK_IPCLKPORT_ACLK,
	GOUT_BLK_HSI0_UID_UASC_HSI0_LINK_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_hsi1_cmu_hsi1[] = {
	CLK_BLK_HSI1_UID_HSI1_CMU_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_acel_d_hsi1[] = {
	GOUT_BLK_HSI1_UID_LHS_ACEL_D_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_hsi1[] = {
	GOUT_BLK_HSI1_UID_LHM_AXI_P_HSI1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_hsi1[] = {
	GOUT_BLK_HSI1_UID_SYSREG_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_hsi1[] = {
	GOUT_BLK_HSI1_UID_XIU_D_HSI1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_hsi1[] = {
	GOUT_BLK_HSI1_UID_PPMU_HSI1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_PPMU_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_hsi1[] = {
	GOUT_BLK_HSI1_UID_SYSMMU_HSI1_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_xiu_p_hsi1[] = {
	GOUT_BLK_HSI1_UID_XIU_P_HSI1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_pcie_gen4_0[] = {
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCS_PMA_INST_0_PIPE_PAL_PCIE_INST_0_I_APB_PCLK,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_003_G4X2_DWC_PCIE_CTL_INST_0_DBI_ACLK_UG,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_003_G4X2_DWC_PCIE_CTL_INST_0_MSTR_ACLK_UG,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_003_PCIE_SUB_CTRL_INST_0_I_DRIVER_APB_CLK,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_003_PCIE_SUB_CTRL_INST_0_PHY_REFCLK_IN,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_003_G4X2_DWC_PCIE_CTL_INST_0_SLV_ACLK_UG,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCS_PMA_INST_0_SF_PCIEPHY210X2_LN05LPE_QCH_TM_WRAPPER_INST_0_I_APB_PCLK,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_004_G4X1_DWC_PCIE_CTL_INST_0_DBI_ACLK_UG,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_004_G4X1_DWC_PCIE_CTL_INST_0_MSTR_ACLK_UG,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_004_G4X1_DWC_PCIE_CTL_INST_0_SLV_ACLK_UG,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_004_PCIE_SUB_CTRL_INST_0_I_DRIVER_APB_CLK,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCIE_004_PCIE_SUB_CTRL_INST_0_PHY_REFCLK_IN,
	GOUT_BLK_HSI1_UID_PCIE_GEN4_0_IPCLKPORT_PCS_PMA_INST_0_PHY_UDBG_I_APB_PCLK,
};
enum clk_id cmucal_vclk_ip_pcie_ia_gen4a_0[] = {
	GOUT_BLK_HSI1_UID_PCIE_IA_GEN4A_0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_pcie_ia_gen4b_0[] = {
	GOUT_BLK_HSI1_UID_PCIE_IA_GEN4B_0_IPCLKPORT_I_CLK,
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
enum clk_id cmucal_vclk_ip_qe_pcie_gen4a_hsi1[] = {
	GOUT_BLK_HSI1_UID_QE_PCIE_GEN4A_HSI1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_QE_PCIE_GEN4A_HSI1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_pcie_gen4b_hsi1[] = {
	GOUT_BLK_HSI1_UID_QE_PCIE_GEN4B_HSI1_IPCLKPORT_PCLK,
	GOUT_BLK_HSI1_UID_QE_PCIE_GEN4B_HSI1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen4a_dbi_0[] = {
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN4A_DBI_0_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN4A_DBI_0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen4a_slv_0[] = {
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN4A_SLV_0_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN4A_SLV_0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen4b_dbi_0[] = {
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN4B_DBI_0_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN4B_DBI_0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen4b_slv_0[] = {
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN4B_SLV_0_IPCLKPORT_ACLK,
	GOUT_BLK_HSI1_UID_UASC_PCIE_GEN4B_SLV_0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pcie_ia_gen4a_0[] = {
	CLK_BLK_HSI1_UID_SSMT_PCIE_IA_GEN4A_0_IPCLKPORT_PCLK,
	CLK_BLK_HSI1_UID_SSMT_PCIE_IA_GEN4A_0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pcie_ia_gen4b_0[] = {
	CLK_BLK_HSI1_UID_SSMT_PCIE_IA_GEN4B_0_IPCLKPORT_ACLK,
	CLK_BLK_HSI1_UID_SSMT_PCIE_IA_GEN4B_0_IPCLKPORT_PCLK,
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
enum clk_id cmucal_vclk_ip_lhs_acel_d_hsi2[] = {
	GOUT_BLK_HSI2_UID_LHS_ACEL_D_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_hsi2[] = {
	GOUT_BLK_HSI2_UID_LHM_AXI_P_HSI2_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_hsi2[] = {
	GOUT_BLK_HSI2_UID_XIU_D_HSI2_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_p_hsi2[] = {
	GOUT_BLK_HSI2_UID_XIU_P_HSI2_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_hsi2[] = {
	GOUT_BLK_HSI2_UID_PPMU_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_PPMU_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_pcie_gen4_1[] = {
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_003_G4X2_DWC_PCIE_CTL_INST_0_SLV_ACLK_UG,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_003_G4X2_DWC_PCIE_CTL_INST_0_DBI_ACLK_UG,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCS_PMA_INST_0_PIPE_PAL_PCIE_INST_0_I_APB_PCLK,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_003_PCIE_SUB_CTRL_INST_0_I_DRIVER_APB_CLK,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_003_G4X2_DWC_PCIE_CTL_INST_0_MSTR_ACLK_UG,
	CLK_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_003_PCIE_SUB_CTRL_INST_0_PHY_REFCLK_IN,
	CLK_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_004_PCIE_SUB_CTRL_INST_0_PHY_REFCLK_IN,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCS_PMA_INST_0_SF_PCIEPHY210X2_LN05LPE_QCH_TM_WRAPPER_INST_0_I_APB_PCLK,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_004_G4X1_DWC_PCIE_CTL_INST_0_DBI_ACLK_UG,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_004_G4X1_DWC_PCIE_CTL_INST_0_SLV_ACLK_UG,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_004_G4X1_DWC_PCIE_CTL_INST_0_MSTR_ACLK_UG,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCIE_004_PCIE_SUB_CTRL_INST_0_I_DRIVER_APB_CLK,
	GOUT_BLK_HSI2_UID_PCIE_GEN4_1_IPCLKPORT_PCS_PMA_INST_0_PHY_UDBG_I_APB_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_hsi2[] = {
	GOUT_BLK_HSI2_UID_SYSMMU_HSI2_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ssmt_hsi2[] = {
	GOUT_BLK_HSI2_UID_SSMT_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_SSMT_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_pcie_ia_gen4a_1[] = {
	GOUT_BLK_HSI2_UID_PCIE_IA_GEN4A_1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_hsi2[] = {
	GOUT_BLK_HSI2_UID_D_TZPC_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ufs_embd[] = {
	GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_ACLK,
	GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_FMP_CLK,
	GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_CLK_UNIPRO,
};
enum clk_id cmucal_vclk_ip_pcie_ia_gen4b_1[] = {
	GOUT_BLK_HSI2_UID_PCIE_IA_GEN4B_1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_hsi2[] = {
	GOUT_BLK_HSI2_UID_GPC_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mmc_card[] = {
	GOUT_BLK_HSI2_UID_MMC_CARD_IPCLKPORT_I_ACLK,
	GOUT_BLK_HSI2_UID_MMC_CARD_IPCLKPORT_SDCLKIN,
};
enum clk_id cmucal_vclk_ip_qe_pcie_gen4a_hsi2[] = {
	GOUT_BLK_HSI2_UID_QE_PCIE_GEN4A_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_QE_PCIE_GEN4A_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_pcie_gen4b_hsi2[] = {
	GOUT_BLK_HSI2_UID_QE_PCIE_GEN4B_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_QE_PCIE_GEN4B_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_ufs_embd_hsi2[] = {
	GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen4a_dbi_1[] = {
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN4A_DBI_1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN4A_DBI_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen4a_slv_1[] = {
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN4A_SLV_1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN4A_SLV_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen4b_dbi_1[] = {
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN4B_DBI_1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN4B_DBI_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_uasc_pcie_gen4b_slv_1[] = {
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN4B_SLV_1_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_UASC_PCIE_GEN4B_SLV_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_mmc_card_hsi2[] = {
	GOUT_BLK_HSI2_UID_QE_MMC_CARD_HSI2_IPCLKPORT_ACLK,
	GOUT_BLK_HSI2_UID_QE_MMC_CARD_HSI2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pcie_ia_gen4a_1[] = {
	CLK_BLK_HSI2_UID_SSMT_PCIE_IA_GEN4A_1_IPCLKPORT_ACLK,
	CLK_BLK_HSI2_UID_SSMT_PCIE_IA_GEN4A_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pcie_ia_gen4b_1[] = {
	CLK_BLK_HSI2_UID_SSMT_PCIE_IA_GEN4B_1_IPCLKPORT_ACLK,
	CLK_BLK_HSI2_UID_SSMT_PCIE_IA_GEN4B_1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ipp_cmu_ipp[] = {
	CLK_BLK_IPP_UID_IPP_CMU_IPP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_ipp[] = {
	GOUT_BLK_IPP_UID_D_TZPC_IPP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_ipp[] = {
	GOUT_BLK_IPP_UID_LHM_AXI_P_IPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_ipp[] = {
	GOUT_BLK_IPP_UID_SYSREG_IPP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_vo_ippdns[] = {
	GOUT_BLK_IPP_UID_LHS_AST_VO_IPPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_vo_pdpipp[] = {
	GOUT_BLK_IPP_UID_LHM_AST_VO_PDPIPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_ipp[] = {
	GOUT_BLK_IPP_UID_AD_APB_IPP_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_ipp[] = {
	GOUT_BLK_IPP_UID_LHS_AXI_D_IPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_sotf0_ippcsis[] = {
	GOUT_BLK_IPP_UID_LHS_AST_SOTF0_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_sotf1_ippcsis[] = {
	GOUT_BLK_IPP_UID_LHS_AST_SOTF1_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_sotf2_ippcsis[] = {
	GOUT_BLK_IPP_UID_LHS_AST_SOTF2_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_zotf0_ippcsis[] = {
	GOUT_BLK_IPP_UID_LHS_AST_ZOTF0_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_zotf1_ippcsis[] = {
	GOUT_BLK_IPP_UID_LHS_AST_ZOTF1_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_zotf2_ippcsis[] = {
	GOUT_BLK_IPP_UID_LHS_AST_ZOTF2_IPPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_ipp[] = {
	GOUT_BLK_IPP_UID_PPMU_IPP_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_PPMU_IPP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sipu_ipp[] = {
	GOUT_BLK_IPP_UID_SIPU_IPP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_ipp[] = {
	GOUT_BLK_IPP_UID_SYSMMU_IPP_IPCLKPORT_CLK_S1,
	GOUT_BLK_IPP_UID_SYSMMU_IPP_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_gpc_ipp[] = {
	GOUT_BLK_IPP_UID_GPC_IPP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_thstat[] = {
	GOUT_BLK_IPP_UID_SSMT_THSTAT_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_SSMT_THSTAT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_ippdns[] = {
	GOUT_BLK_IPP_UID_LHS_AXI_D_IPPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_msa[] = {
	GOUT_BLK_IPP_UID_PPMU_MSA_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_PPMU_MSA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_align0[] = {
	GOUT_BLK_IPP_UID_QE_ALIGN0_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_ALIGN0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_align1[] = {
	GOUT_BLK_IPP_UID_QE_ALIGN1_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_ALIGN1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_align0[] = {
	GOUT_BLK_IPP_UID_SSMT_ALIGN0_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_SSMT_ALIGN0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_align1[] = {
	GOUT_BLK_IPP_UID_SSMT_ALIGN1_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_SSMT_ALIGN1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d1_ipp[] = {
	GOUT_BLK_IPP_UID_XIU_D1_IPP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_tnr_a[] = {
	GOUT_BLK_IPP_UID_TNR_A_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_thstat[] = {
	GOUT_BLK_IPP_UID_QE_THSTAT_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_THSTAT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf0_pdpipp[] = {
	GOUT_BLK_IPP_UID_LHM_AST_OTF0_PDPIPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf1_pdpipp[] = {
	GOUT_BLK_IPP_UID_LHM_AST_OTF1_PDPIPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf2_pdpipp[] = {
	GOUT_BLK_IPP_UID_LHM_AST_OTF2_PDPIPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf_ippdns[] = {
	GOUT_BLK_IPP_UID_LHS_AST_OTF_IPPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d2_ipp[] = {
	GOUT_BLK_IPP_UID_XIU_D2_IPP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_xiu_d0_ipp[] = {
	GOUT_BLK_IPP_UID_XIU_D0_IPP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_fdpig[] = {
	GOUT_BLK_IPP_UID_SSMT_FDPIG_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_SSMT_FDPIG_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_rgbh0[] = {
	GOUT_BLK_IPP_UID_SSMT_RGBH0_IPCLKPORT_PCLK,
	GOUT_BLK_IPP_UID_SSMT_RGBH0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_rgbh1[] = {
	GOUT_BLK_IPP_UID_SSMT_RGBH1_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_SSMT_RGBH1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_rgbh2[] = {
	GOUT_BLK_IPP_UID_SSMT_RGBH2_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_SSMT_RGBH2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_align2[] = {
	GOUT_BLK_IPP_UID_SSMT_ALIGN2_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_SSMT_ALIGN2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_align3[] = {
	GOUT_BLK_IPP_UID_SSMT_ALIGN3_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_SSMT_ALIGN3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_fdpig[] = {
	GOUT_BLK_IPP_UID_QE_FDPIG_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_FDPIG_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_rgbh0[] = {
	GOUT_BLK_IPP_UID_QE_RGBH0_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_RGBH0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_rgbh1[] = {
	GOUT_BLK_IPP_UID_QE_RGBH1_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_RGBH1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_rgbh2[] = {
	GOUT_BLK_IPP_UID_QE_RGBH2_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_RGBH2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_align2[] = {
	GOUT_BLK_IPP_UID_QE_ALIGN2_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_ALIGN2_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_align3[] = {
	GOUT_BLK_IPP_UID_QE_ALIGN3_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_ALIGN3_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_tnr_msa0[] = {
	GOUT_BLK_IPP_UID_SSMT_TNR_MSA0_IPCLKPORT_PCLK,
	GOUT_BLK_IPP_UID_SSMT_TNR_MSA0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_aln_stat[] = {
	GOUT_BLK_IPP_UID_SSMT_ALN_STAT_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_SSMT_ALN_STAT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_tnr_msa0[] = {
	GOUT_BLK_IPP_UID_QE_TNR_MSA0_IPCLKPORT_PCLK,
	GOUT_BLK_IPP_UID_QE_TNR_MSA0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_aln_stat[] = {
	GOUT_BLK_IPP_UID_QE_ALN_STAT_IPCLKPORT_PCLK,
	GOUT_BLK_IPP_UID_QE_ALN_STAT_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_tnr_msa1[] = {
	GOUT_BLK_IPP_UID_SSMT_TNR_MSA1_IPCLKPORT_PCLK,
	GOUT_BLK_IPP_UID_SSMT_TNR_MSA1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_tnr_msa1[] = {
	GOUT_BLK_IPP_UID_QE_TNR_MSA1_IPCLKPORT_ACLK,
	GOUT_BLK_IPP_UID_QE_TNR_MSA1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_itp_cmu_itp[] = {
	CLK_BLK_ITP_UID_ITP_CMU_ITP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_itp[] = {
	GOUT_BLK_ITP_UID_AD_APB_ITP_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_itp[] = {
	GOUT_BLK_ITP_UID_D_TZPC_ITP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_itp[] = {
	GOUT_BLK_ITP_UID_GPC_ITP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_itp[] = {
	GOUT_BLK_ITP_UID_ITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_itp[] = {
	GOUT_BLK_ITP_UID_LHM_AXI_P_ITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_itp[] = {
	GOUT_BLK_ITP_UID_SYSREG_ITP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_ctl_dnsitp[] = {
	GOUT_BLK_ITP_UID_LHM_AST_CTL_DNSITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf0_dnsitp[] = {
	GOUT_BLK_ITP_UID_LHM_AST_OTF0_DNSITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf1_dnsitp[] = {
	GOUT_BLK_ITP_UID_LHM_AST_OTF1_DNSITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf2_dnsitp[] = {
	GOUT_BLK_ITP_UID_LHM_AST_OTF2_DNSITP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_ctl_itpdns[] = {
	GOUT_BLK_ITP_UID_LHS_AST_CTL_ITPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf_itpdns[] = {
	GOUT_BLK_ITP_UID_LHS_AST_OTF_ITPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_mcsc[] = {
	GOUT_BLK_MCSC_UID_LHM_AXI_P_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d0_mcsc[] = {
	GOUT_BLK_MCSC_UID_LHS_AXI_D0_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_mcsc[] = {
	GOUT_BLK_MCSC_UID_SYSREG_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_mcsc_cmu_mcsc[] = {
	CLK_BLK_MCSC_UID_MCSC_CMU_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf0_dnsmcsc[] = {
	GOUT_BLK_MCSC_UID_LHM_AST_OTF0_DNSMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_mcsc[] = {
	GOUT_BLK_MCSC_UID_D_TZPC_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf_mcsctnr[] = {
	GOUT_BLK_MCSC_UID_LHS_AST_OTF_MCSCTNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf1_dnsmcsc[] = {
	GOUT_BLK_MCSC_UID_LHM_AST_OTF1_DNSMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_mcsc[] = {
	GOUT_BLK_MCSC_UID_GPC_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_itsc[] = {
	GOUT_BLK_MCSC_UID_ITSC_IPCLKPORT_CLK,
	GOUT_BLK_MCSC_UID_ITSC_IPCLKPORT_C2CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_mcsc[] = {
	GOUT_BLK_MCSC_UID_SSMT_D0_MCSC_IPCLKPORT_PCLK,
	GOUT_BLK_MCSC_UID_SSMT_D0_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d0_mcsc[] = {
	GOUT_BLK_MCSC_UID_SYSMMU_D0_MCSC_IPCLKPORT_CLK_S1,
	GOUT_BLK_MCSC_UID_SYSMMU_D0_MCSC_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_mcsc[] = {
	GOUT_BLK_MCSC_UID_PPMU_D0_MCSC_IPCLKPORT_PCLK,
	GOUT_BLK_MCSC_UID_PPMU_D0_MCSC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d0_itsc[] = {
	GOUT_BLK_MCSC_UID_SSMT_D0_ITSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_SSMT_D0_ITSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_itsc[] = {
	GOUT_BLK_MCSC_UID_PPMU_D1_ITSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_PPMU_D1_ITSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_itsc[] = {
	GOUT_BLK_MCSC_UID_PPMU_D0_ITSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_PPMU_D0_ITSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_vo_gdcmcsc[] = {
	GOUT_BLK_MCSC_UID_LHM_AST_VO_GDCMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_mcscdns[] = {
	GOUT_BLK_MCSC_UID_LHS_AXI_D_MCSCDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_itsc[] = {
	GOUT_BLK_MCSC_UID_AD_APB_ITSC_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ad_apb_mcsc[] = {
	GOUT_BLK_MCSC_UID_AD_APB_MCSC_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_mcsc[] = {
	GOUT_BLK_MCSC_UID_MCSC_IPCLKPORT_CLK,
	GOUT_BLK_MCSC_UID_MCSC_IPCLKPORT_C2CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d1_mcsc[] = {
	GOUT_BLK_MCSC_UID_LHS_AXI_D1_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d1_mcsc[] = {
	GOUT_BLK_MCSC_UID_SYSMMU_D1_MCSC_IPCLKPORT_CLK_S1,
	GOUT_BLK_MCSC_UID_SYSMMU_D1_MCSC_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf2_dnsmcsc[] = {
	GOUT_BLK_MCSC_UID_LHM_AST_OTF2_DNSMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_int_itscmcsc[] = {
	GOUT_BLK_MCSC_UID_LHM_AST_INT_ITSCMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf_tnrmcsc[] = {
	GOUT_BLK_MCSC_UID_LHM_AST_OTF_TNRMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_int_itscmcsc[] = {
	GOUT_BLK_MCSC_UID_LHS_AST_INT_ITSCMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_vo_mcsccsis[] = {
	GOUT_BLK_MCSC_UID_LHS_AST_VO_MCSCCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_itsc[] = {
	GOUT_BLK_MCSC_UID_SSMT_D1_ITSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_SSMT_D1_ITSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_mcsc[] = {
	GOUT_BLK_MCSC_UID_PPMU_D1_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_PPMU_D1_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d1_mcsc[] = {
	GOUT_BLK_MCSC_UID_SSMT_D1_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_SSMT_D1_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d1_itsc[] = {
	GOUT_BLK_MCSC_UID_QE_D1_ITSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_QE_D1_ITSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d2_itsc[] = {
	GOUT_BLK_MCSC_UID_QE_D2_ITSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_QE_D2_ITSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d0_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D0_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_QE_D0_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d1_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D1_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_QE_D1_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d2_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D2_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_QE_D2_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_d3_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D3_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_QE_D3_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d2_mcsc[] = {
	GOUT_BLK_MCSC_UID_SYSMMU_D2_MCSC_IPCLKPORT_CLK_S1,
	GOUT_BLK_MCSC_UID_SYSMMU_D2_MCSC_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d2_mcsc[] = {
	GOUT_BLK_MCSC_UID_LHS_AXI_D2_MCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_qe_d4_mcsc[] = {
	GOUT_BLK_MCSC_UID_QE_D4_MCSC_IPCLKPORT_ACLK,
	GOUT_BLK_MCSC_UID_QE_D4_MCSC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_c2r_mcsc[] = {
	GOUT_BLK_MCSC_UID_C2R_MCSC_IPCLKPORT_C2CLK,
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
enum clk_id cmucal_vclk_ip_lhs_axi_d0_mfc[] = {
	GOUT_BLK_MFC_UID_LHS_AXI_D0_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d1_mfc[] = {
	GOUT_BLK_MFC_UID_LHS_AXI_D1_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_mfc[] = {
	GOUT_BLK_MFC_UID_LHM_AXI_P_MFC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d0_mfc[] = {
	GOUT_BLK_MFC_UID_SYSMMU_D0_MFC_IPCLKPORT_CLK_S1,
	GOUT_BLK_MFC_UID_SYSMMU_D0_MFC_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysmmu_d1_mfc[] = {
	GOUT_BLK_MFC_UID_SYSMMU_D1_MFC_IPCLKPORT_CLK_S1,
	GOUT_BLK_MFC_UID_SYSMMU_D1_MFC_IPCLKPORT_CLK_S2,
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
enum clk_id cmucal_vclk_ip_mif_cmu_mif[] = {
	CLK_BLK_MIF_UID_MIF_CMU_MIF_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ddrphy[] = {
	GOUT_BLK_MIF_UID_DDRPHY_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysreg_mif[] = {
	GOUT_BLK_MIF_UID_SYSREG_MIF_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_mif[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_P_MIF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_axi2apb_mif[] = {
	GOUT_BLK_MIF_UID_AXI2APB_MIF_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_apbbr_ddrphy[] = {
	GOUT_BLK_MIF_UID_APBBR_DDRPHY_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_apbbr_dmc[] = {
	GOUT_BLK_MIF_UID_APBBR_DMC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_dmc[] = {
	GOUT_BLK_MIF_UID_DMC_IPCLKPORT_PCLK,
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
enum clk_id cmucal_vclk_ip_otp_con_bisr[] = {
	CLK_BLK_MISC_UID_OTP_CON_BISR_IPCLKPORT_I_OSCCLK,
	GOUT_BLK_MISC_UID_OTP_CON_BISR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_dit[] = {
	GOUT_BLK_MISC_UID_DIT_IPCLKPORT_ICLKL2A,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_misc[] = {
	GOUT_BLK_MISC_UID_LHM_AXI_P_MISC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_acel_d_misc[] = {
	GOUT_BLK_MISC_UID_LHS_ACEL_D_MISC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_pdma[] = {
	GOUT_BLK_MISC_UID_PDMA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_misc[] = {
	GOUT_BLK_MISC_UID_PPMU_MISC_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_PPMU_MISC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_dit[] = {
	GOUT_BLK_MISC_UID_QE_DIT_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_QE_DIT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_pdma[] = {
	GOUT_BLK_MISC_UID_QE_PDMA_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_QE_PDMA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_misc_cmu_misc[] = {
	CLK_BLK_MISC_UID_MISC_CMU_MISC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_rtic[] = {
	GOUT_BLK_MISC_UID_QE_RTIC_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_QE_RTIC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_spdma[] = {
	GOUT_BLK_MISC_UID_QE_SPDMA_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_QE_SPDMA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_sss[] = {
	GOUT_BLK_MISC_UID_QE_SSS_IPCLKPORT_PCLK,
	GOUT_BLK_MISC_UID_QE_SSS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_rtic[] = {
	GOUT_BLK_MISC_UID_RTIC_IPCLKPORT_I_ACLK,
	GOUT_BLK_MISC_UID_RTIC_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_ip_spdma[] = {
	GOUT_BLK_MISC_UID_SPDMA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sss[] = {
	GOUT_BLK_MISC_UID_SSS_IPCLKPORT_I_PCLK,
	GOUT_BLK_MISC_UID_SSS_IPCLKPORT_I_ACLK,
};
enum clk_id cmucal_vclk_ip_ssmt_sss[] = {
	GOUT_BLK_MISC_UID_SSMT_SSS_IPCLKPORT_PCLK,
	GOUT_BLK_MISC_UID_SSMT_SSS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_gpc_misc[] = {
	GOUT_BLK_MISC_UID_GPC_MISC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_dit[] = {
	GOUT_BLK_MISC_UID_AD_APB_DIT_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_ppmu_dma[] = {
	GOUT_BLK_MISC_UID_PPMU_DMA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_ppmu_dma[] = {
	GOUT_BLK_MISC_UID_QE_PPMU_DMA_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_QE_PPMU_DMA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_adm_ahb_sss[] = {
	GOUT_BLK_MISC_UID_ADM_AHB_SSS_IPCLKPORT_HCLKM,
};
enum clk_id cmucal_vclk_ip_ad_apb_puf[] = {
	GOUT_BLK_MISC_UID_AD_APB_PUF_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lhm_ast_icc_cpugic[] = {
	GOUT_BLK_MISC_UID_LHM_AST_ICC_CPUGIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_d_sss[] = {
	GOUT_BLK_MISC_UID_LHM_AXI_D_SSS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_iri_giccpu[] = {
	GOUT_BLK_MISC_UID_LHS_AST_IRI_GICCPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_sss[] = {
	GOUT_BLK_MISC_UID_LHS_AXI_D_SSS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_puf[] = {
	GOUT_BLK_MISC_UID_PUF_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_misc[] = {
	GOUT_BLK_MISC_UID_XIU_D_MISC_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_misc[] = {
	GOUT_BLK_MISC_UID_SYSMMU_MISC_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysmmu_sss[] = {
	GOUT_BLK_MISC_UID_SYSMMU_SSS_IPCLKPORT_CLK_S1,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_gic[] = {
	GOUT_BLK_MISC_UID_LHM_AXI_P_GIC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_rtic[] = {
	GOUT_BLK_MISC_UID_SSMT_RTIC_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_SSMT_RTIC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_spdma[] = {
	GOUT_BLK_MISC_UID_SSMT_SPDMA_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_SSMT_SPDMA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pdma[] = {
	GOUT_BLK_MISC_UID_SSMT_PDMA_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_SSMT_PDMA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_ppmu_dma[] = {
	GOUT_BLK_MISC_UID_SSMT_PPMU_DMA_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_SSMT_PPMU_DMA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_dit[] = {
	GOUT_BLK_MISC_UID_SSMT_DIT_IPCLKPORT_ACLK,
	GOUT_BLK_MISC_UID_SSMT_DIT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_pdp_cmu_pdp[] = {
	CLK_BLK_PDP_UID_PDP_CMU_PDP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_pdp[] = {
	GOUT_BLK_PDP_UID_D_TZPC_PDP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf0_csispdp[] = {
	GOUT_BLK_PDP_UID_LHM_AST_OTF0_CSISPDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf1_csispdp[] = {
	GOUT_BLK_PDP_UID_LHM_AST_OTF1_CSISPDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf2_csispdp[] = {
	GOUT_BLK_PDP_UID_LHM_AST_OTF2_CSISPDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_pdp[] = {
	GOUT_BLK_PDP_UID_LHM_AXI_P_PDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_gpc_pdp[] = {
	GOUT_BLK_PDP_UID_GPC_PDP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_pdp_top[] = {
	GOUT_BLK_PDP_UID_PDP_TOP_IPCLKPORT_C2CLK,
	GOUT_BLK_PDP_UID_PDP_TOP_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_pdp_stat[] = {
	GOUT_BLK_PDP_UID_SSMT_PDP_STAT_IPCLKPORT_ACLK,
	GOUT_BLK_PDP_UID_SSMT_PDP_STAT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_pdp_stat0[] = {
	GOUT_BLK_PDP_UID_QE_PDP_STAT0_IPCLKPORT_ACLK,
	GOUT_BLK_PDP_UID_QE_PDP_STAT0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_c2_pdp[] = {
	GOUT_BLK_PDP_UID_AD_APB_C2_PDP_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf0_pdpipp[] = {
	GOUT_BLK_PDP_UID_LHS_AST_OTF0_PDPIPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf1_pdpipp[] = {
	GOUT_BLK_PDP_UID_LHS_AST_OTF1_PDPIPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf2_pdpipp[] = {
	GOUT_BLK_PDP_UID_LHS_AST_OTF2_PDPIPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf0_pdpcsis[] = {
	GOUT_BLK_PDP_UID_LHS_AST_OTF0_PDPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf1_pdpcsis[] = {
	GOUT_BLK_PDP_UID_LHS_AST_OTF1_PDPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf2_pdpcsis[] = {
	GOUT_BLK_PDP_UID_LHS_AST_OTF2_PDPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf0_pdpg3aa[] = {
	GOUT_BLK_PDP_UID_LHS_AST_OTF0_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf1_pdpg3aa[] = {
	GOUT_BLK_PDP_UID_LHS_AST_OTF1_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf2_pdpg3aa[] = {
	GOUT_BLK_PDP_UID_LHS_AST_OTF2_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_yotf0_pdpg3aa[] = {
	GOUT_BLK_PDP_UID_LHS_AST_YOTF0_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_yotf1_pdpg3aa[] = {
	GOUT_BLK_PDP_UID_LHS_AST_YOTF1_PDPG3AA_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_vo_csispdp[] = {
	GOUT_BLK_PDP_UID_LHM_AST_VO_CSISPDP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_vo_pdpipp[] = {
	GOUT_BLK_PDP_UID_LHS_AST_VO_PDPIPP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_pdpcsis[] = {
	GOUT_BLK_PDP_UID_LHS_AXI_D_PDPCSIS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_pdp[] = {
	GOUT_BLK_PDP_UID_SYSREG_PDP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_xiu_d_pdp[] = {
	GOUT_BLK_PDP_UID_XIU_D_PDP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_qe_pdp_stat1[] = {
	GOUT_BLK_PDP_UID_QE_PDP_STAT1_IPCLKPORT_ACLK,
	GOUT_BLK_PDP_UID_QE_PDP_STAT1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_pdp_af0[] = {
	GOUT_BLK_PDP_UID_QE_PDP_AF0_IPCLKPORT_ACLK,
	GOUT_BLK_PDP_UID_QE_PDP_AF0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_qe_pdp_af1[] = {
	GOUT_BLK_PDP_UID_QE_PDP_AF1_IPCLKPORT_ACLK,
	GOUT_BLK_PDP_UID_QE_PDP_AF1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ad_apb_vra[] = {
	GOUT_BLK_PDP_UID_AD_APB_VRA_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_qe_vra[] = {
	CLK_BLK_PDP_UID_QE_VRA_IPCLKPORT_ACLK,
	CLK_BLK_PDP_UID_QE_VRA_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_vra[] = {
	CLK_BLK_PDP_UID_VRA_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_ssmt_vra[] = {
	CLK_BLK_PDP_UID_SSMT_VRA_IPCLKPORT_PCLK,
	CLK_BLK_PDP_UID_SSMT_VRA_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_pdpdns[] = {
	CLK_BLK_PDP_UID_LHS_AXI_D_PDPDNS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_vra[] = {
	CLK_BLK_PDP_UID_PPMU_VRA_IPCLKPORT_ACLK,
	CLK_BLK_PDP_UID_PPMU_VRA_IPCLKPORT_PCLK,
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
enum clk_id cmucal_vclk_ip_lhm_axi_p_peric0[] = {
	GOUT_BLK_PERIC0_UID_LHM_AXI_P_PERIC0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_peric0_top0[] = {
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_0,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_1,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_2,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_3,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_4,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_5,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_8,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_9,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_10,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_10,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_11,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_11,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_12,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_12,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_13,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_13,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_14,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_14,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_15,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_15,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_8,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_9,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_10,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_10,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_11,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_11,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_12,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_12,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_13,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_13,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_14,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_14,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_15,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_15,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_7,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_PCLK_6,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_5,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_6,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_7,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_0,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_1,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_2,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_3,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP0_IPCLKPORT_IPCLK_4,
};
enum clk_id cmucal_vclk_ip_d_tzpc_peric0[] = {
	GOUT_BLK_PERIC0_UID_D_TZPC_PERIC0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_peric0_top1[] = {
	GOUT_BLK_PERIC0_UID_PERIC0_TOP1_IPCLKPORT_PCLK_0,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP1_IPCLKPORT_PCLK_2,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP1_IPCLKPORT_IPCLK_0,
	GOUT_BLK_PERIC0_UID_PERIC0_TOP1_IPCLKPORT_IPCLK_2,
};
enum clk_id cmucal_vclk_ip_gpc_peric0[] = {
	GOUT_BLK_PERIC0_UID_GPC_PERIC0_IPCLKPORT_PCLK,
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
enum clk_id cmucal_vclk_ip_lhm_axi_p_peric1[] = {
	GOUT_BLK_PERIC1_UID_LHM_AXI_P_PERIC1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_peric1[] = {
	GOUT_BLK_PERIC1_UID_D_TZPC_PERIC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_peric1_top0[] = {
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_IPCLK_8,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_IPCLK_1,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_IPCLK_2,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_IPCLK_3,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_IPCLK_4,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_IPCLK_5,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_PCLK_1,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_PCLK_15,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_PCLK_15,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_PCLK_2,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_PCLK_3,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_PCLK_4,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_PCLK_5,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_PCLK_8,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_IPCLK_6,
	GOUT_BLK_PERIC1_UID_PERIC1_TOP0_IPCLKPORT_PCLK_6,
};
enum clk_id cmucal_vclk_ip_gpc_peric1[] = {
	GOUT_BLK_PERIC1_UID_GPC_PERIC1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_s2d_cmu_s2d[] = {
	CLK_BLK_S2D_UID_S2D_CMU_S2D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_bis_s2d[] = {
	GOUT_BLK_S2D_UID_BIS_S2D_IPCLKPORT_SCLK,
	GOUT_BLK_S2D_UID_BIS_S2D_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_g_scan2dram[] = {
	GOUT_BLK_S2D_UID_LHM_AXI_G_SCAN2DRAM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_apb_async_sysmmu_d0_s1_ns_tnr[] = {
	GOUT_BLK_TNR_UID_APB_ASYNC_SYSMMU_D0_S1_NS_TNR_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_d_tzpc_tnr[] = {
	GOUT_BLK_TNR_UID_D_TZPC_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_vo_dnstnr[] = {
	GOUT_BLK_TNR_UID_LHM_AST_VO_DNSTNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_tnr[] = {
	GOUT_BLK_TNR_UID_LHM_AXI_P_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf_tnrmcsc[] = {
	GOUT_BLK_TNR_UID_LHS_AST_OTF_TNRMCSC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d0_tnr[] = {
	GOUT_BLK_TNR_UID_LHS_AXI_D0_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d1_tnr[] = {
	GOUT_BLK_TNR_UID_LHS_AXI_D1_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d0_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D0_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D0_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d1_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D1_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D1_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d0_tnr[] = {
	GOUT_BLK_TNR_UID_SYSMMU_D0_TNR_IPCLKPORT_CLK_S1,
	GOUT_BLK_TNR_UID_SYSMMU_D0_TNR_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysmmu_d1_tnr[] = {
	GOUT_BLK_TNR_UID_SYSMMU_D1_TNR_IPCLKPORT_CLK_S1,
	GOUT_BLK_TNR_UID_SYSMMU_D1_TNR_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysreg_tnr[] = {
	GOUT_BLK_TNR_UID_SYSREG_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_tnr_cmu_tnr[] = {
	CLK_BLK_TNR_UID_TNR_CMU_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_vo_tnrgdc[] = {
	GOUT_BLK_TNR_UID_LHS_AST_VO_TNRGDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_tnr[] = {
	GOUT_BLK_TNR_UID_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_TNR_IPCLKPORT_C2CLK,
};
enum clk_id cmucal_vclk_ip_lhm_ast_otf_mcsctnr[] = {
	GOUT_BLK_TNR_UID_LHM_AST_OTF_MCSCTNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d2_tnr[] = {
	GOUT_BLK_TNR_UID_LHS_AXI_D2_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d3_tnr[] = {
	GOUT_BLK_TNR_UID_LHS_AXI_D3_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d2_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D2_TNR_IPCLKPORT_PCLK,
	GOUT_BLK_TNR_UID_PPMU_D2_TNR_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d3_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D3_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D3_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d2_tnr[] = {
	GOUT_BLK_TNR_UID_SYSMMU_D2_TNR_IPCLKPORT_CLK_S1,
	GOUT_BLK_TNR_UID_SYSMMU_D2_TNR_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_sysmmu_d3_tnr[] = {
	GOUT_BLK_TNR_UID_SYSMMU_D3_TNR_IPCLKPORT_CLK_S1,
	GOUT_BLK_TNR_UID_SYSMMU_D3_TNR_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ppmu_d4_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D4_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D4_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d5_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D5_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D5_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d6_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D6_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D6_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ppmu_d7_tnr[] = {
	GOUT_BLK_TNR_UID_PPMU_D7_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_PPMU_D7_TNR_IPCLKPORT_PCLK,
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
enum clk_id cmucal_vclk_ip_lhs_axi_d4_tnr[] = {
	GOUT_BLK_TNR_UID_LHS_AXI_D4_TNR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_d4_tnr[] = {
	GOUT_BLK_TNR_UID_SYSMMU_D4_TNR_IPCLKPORT_CLK_S1,
	GOUT_BLK_TNR_UID_SYSMMU_D4_TNR_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ssmt_d4_tnr[] = {
	GOUT_BLK_TNR_UID_SSMT_D4_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_SSMT_D4_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d5_tnr[] = {
	GOUT_BLK_TNR_UID_SSMT_D5_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_SSMT_D5_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d6_tnr[] = {
	GOUT_BLK_TNR_UID_SSMT_D6_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_SSMT_D6_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_d7_tnr[] = {
	GOUT_BLK_TNR_UID_SSMT_D7_TNR_IPCLKPORT_ACLK,
	GOUT_BLK_TNR_UID_SSMT_D7_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_gpc_tnr[] = {
	GOUT_BLK_TNR_UID_GPC_TNR_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_ast_otf_tnrgdc[] = {
	GOUT_BLK_TNR_UID_LHS_AST_OTF_TNRGDC_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_tpu_cmu_tpu[] = {
	CLK_BLK_TPU_UID_TPU_CMU_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhm_axi_p_tpu[] = {
	GOUT_BLK_TPU_UID_LHM_AXI_P_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_d_tzpc_tpu[] = {
	GOUT_BLK_TPU_UID_D_TZPC_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_axi_d_tpu[] = {
	GOUT_BLK_TPU_UID_LHS_AXI_D_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_sysreg_tpu[] = {
	GOUT_BLK_TPU_UID_SYSREG_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_sysmmu_tpu[] = {
	GOUT_BLK_TPU_UID_SYSMMU_TPU_IPCLKPORT_CLK_S1,
	GOUT_BLK_TPU_UID_SYSMMU_TPU_IPCLKPORT_CLK_S2,
};
enum clk_id cmucal_vclk_ip_ppmu_tpu[] = {
	GOUT_BLK_TPU_UID_PPMU_TPU_IPCLKPORT_ACLK,
	GOUT_BLK_TPU_UID_PPMU_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_ssmt_tpu[] = {
	GOUT_BLK_TPU_UID_SSMT_TPU_IPCLKPORT_PCLK,
	GOUT_BLK_TPU_UID_SSMT_TPU_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ip_gpc_tpu[] = {
	GOUT_BLK_TPU_UID_GPC_TPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_as_apb_sysmmu_ns_tpu[] = {
	GOUT_BLK_TPU_UID_AS_APB_SYSMMU_NS_TPU_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_tpu[] = {
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_TPU_CLK,
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_APB_PCLK,
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_DBG_UART_SCLK,
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_AXI_CLK,
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_TPU_CTL_CLK,
	CLK_BLK_TPU_UID_TPU_IPCLKPORT_DROOPDETECTORIO_CK_IN,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t0_tpucpucl0[] = {
	GOUT_BLK_TPU_UID_LHS_ATB_T0_TPUCPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_t1_tpucpucl0[] = {
	GOUT_BLK_TPU_UID_LHS_ATB_T1_TPUCPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_async_apbm_tpu[] = {
	GOUT_BLK_TPU_UID_ASYNC_APBM_TPU_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_async_apb_int_tpu[] = {
	GOUT_BLK_TPU_UID_ASYNC_APB_INT_TPU_IPCLKPORT_PCLKS,
	GOUT_BLK_TPU_UID_ASYNC_APB_INT_TPU_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ip_lhm_atb_int_t0_tpu[] = {
	GOUT_BLK_TPU_UID_LHM_ATB_INT_T0_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhm_atb_int_t1_tpu[] = {
	GOUT_BLK_TPU_UID_LHM_ATB_INT_T1_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_hpm_tpu[] = {
	CLK_BLK_TPU_UID_HPM_TPU_IPCLKPORT_HPM_TARGETCLK_C,
};
enum clk_id cmucal_vclk_ip_busif_hpmtpu[] = {
	GOUT_BLK_TPU_UID_BUSIF_HPMTPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_int_t0_tpu[] = {
	GOUT_BLK_TPU_UID_LHS_ATB_INT_T0_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_lhs_atb_int_t1_tpu[] = {
	GOUT_BLK_TPU_UID_LHS_ATB_INT_T1_TPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_ip_busif_dddtpu[] = {
	CLK_BLK_TPU_UID_BUSIF_DDDTPU_IPCLKPORT_CK_IN,
};

/* DVFS VCLK -> LUT List */
struct vclk_lut cmucal_vclk_vdd_int_lut[] = {
	{400000, vdd_int_uud_lut_params},
	{300000, vdd_int_ud_lut_params},
	{200000, vdd_int_sud_lut_params},
	{100000, vdd_int_nm_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_mif_lut[] = {
	{1200000, vdd_mif_od_lut_params},
	{1200000, vdd_mif_sud_lut_params},
	{1200000, vdd_mif_ud_lut_params},
	{1200000, vdd_mif_uud_lut_params},
	{950000, vdd_mif_nm_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_g3d_lut[] = {
	{850000, vdd_g3d_nm_lut_params},
	{666000, vdd_g3d_ud_lut_params},
	{466000, vdd_g3d_sud_lut_params},
	{225000, vdd_g3d_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_cpucl0_lut[] = {
	{2100000, vdd_cpucl0_sod_lut_params},
	{1870000, vdd_cpucl0_od_lut_params},
	{1200000, vdd_cpucl0_nm_lut_params},
	{1100000, vdd_cpucl0_ud_lut_params},
	{600000, vdd_cpucl0_sud_lut_params},
	{380000, vdd_cpucl0_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_cpucl1_lut[] = {
	{2470000, vdd_cpucl1_sod_lut_params},
	{2200000, vdd_cpucl1_od_lut_params},
	{1920000, vdd_cpucl1_nm_lut_params},
	{1360000, vdd_cpucl1_ud_lut_params},
	{700000, vdd_cpucl1_sud_lut_params},
	{470000, vdd_cpucl1_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_tpu_lut[] = {
	{1400000, vdd_tpu_od_lut_params},
	{900000, vdd_tpu_ud_lut_params},
	{560000, vdd_tpu_uud_lut_params},
};
struct vclk_lut cmucal_vclk_vdd_cpucl2_lut[] = {
	{3000000, vdd_cpucl2_sod_lut_params},
	{2400000, vdd_cpucl2_od_lut_params},
	{2000000, vdd_cpucl2_nm_lut_params},
	{1500000, vdd_cpucl2_ud_lut_params},
	{1200000, vdd_cpucl2_sud_lut_params},
	{800000, vdd_cpucl2_uud_lut_params},
};

/* SPECIAL VCLK -> LUT List */
struct vclk_lut cmucal_vclk_mux_bus0_cmuref_lut[] = {
	{201000, mux_bus0_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_bus1_cmuref_lut[] = {
	{201000, mux_bus1_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_cmu_cmuref_lut[] = {
	{533250, mux_cmu_cmuref_nm_lut_params},
	{400000, mux_cmu_cmuref_ud_lut_params},
	{266625, mux_cmu_cmuref_sud_lut_params},
	{133313, mux_cmu_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_core_cmuref_lut[] = {
	{201000, mux_core_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_cpucl1_cmuref_lut[] = {
	{201000, mux_cpucl1_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_cpucl2_cmuref_lut[] = {
	{201000, mux_cpucl2_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clk_hsi0_usb20_ref_lut[] = {
	{26000, mux_clk_hsi0_usb20_ref_nm_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_hsi0_usbdpdbg_lut[] = {
	{400000, mux_clkcmu_hsi0_usbdpdbg_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_mif_cmuref_lut[] = {
	{201000, mux_mif_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_clkcmu_hsi0_dpgtc_lut[] = {
	{133313, clkcmu_hsi0_dpgtc_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_hsi1_pcie_lut[] = {
	{400000, mux_clkcmu_hsi1_pcie_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_hsi2_pcie_lut[] = {
	{400000, mux_clkcmu_hsi2_pcie_uud_lut_params},
};
struct vclk_lut cmucal_vclk_clkcmu_tpu_uart_lut[] = {
	{100000, clkcmu_tpu_uart_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_apm_usi0_usi_lut[] = {
	{402000, div_clk_apm_usi0_usi_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_apm_usi0_uart_lut[] = {
	{201000, div_clk_apm_usi0_uart_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_apm_usi1_uart_lut[] = {
	{201000, div_clk_apm_usi1_uart_nm_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_hpm_lut[] = {
	{400000, mux_clkcmu_hpm_uud_lut_params},
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
struct vclk_lut cmucal_vclk_div_clk_slc_dclk_lut[] = {
	{600000, div_clk_slc_dclk_od_lut_params},
	{533250, div_clk_slc_dclk_uud_lut_params},
	{475000, div_clk_slc_dclk_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_slc1_dclk_lut[] = {
	{600000, div_clk_slc1_dclk_od_lut_params},
	{533250, div_clk_slc1_dclk_uud_lut_params},
	{475000, div_clk_slc1_dclk_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_slc2_dclk_lut[] = {
	{600000, div_clk_slc2_dclk_od_lut_params},
	{533250, div_clk_slc2_dclk_uud_lut_params},
	{475000, div_clk_slc2_dclk_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_slc3_dclk_lut[] = {
	{600000, div_clk_slc3_dclk_od_lut_params},
	{533250, div_clk_slc3_dclk_uud_lut_params},
	{475000, div_clk_slc3_dclk_nm_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_cpucl0_cmuref_lut[] = {
	{1050000, div_clk_cpucl0_cmuref_sod_lut_params},
	{935000, div_clk_cpucl0_cmuref_od_lut_params},
	{600000, div_clk_cpucl0_cmuref_nm_lut_params},
	{550000, div_clk_cpucl0_cmuref_ud_lut_params},
	{300000, div_clk_cpucl0_cmuref_sud_lut_params},
	{190000, div_clk_cpucl0_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_cluster0_periphclk_lut[] = {
	{1050000, div_clk_cluster0_periphclk_sod_lut_params},
	{935000, div_clk_cluster0_periphclk_od_lut_params},
	{600000, div_clk_cluster0_periphclk_nm_lut_params},
	{550000, div_clk_cluster0_periphclk_ud_lut_params},
	{300000, div_clk_cluster0_periphclk_sud_lut_params},
	{190000, div_clk_cluster0_periphclk_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_cpucl1_cmuref_lut[] = {
	{1235000, div_clk_cpucl1_cmuref_sod_lut_params},
	{1100000, div_clk_cpucl1_cmuref_od_lut_params},
	{960000, div_clk_cpucl1_cmuref_nm_lut_params},
	{680000, div_clk_cpucl1_cmuref_ud_lut_params},
	{350000, div_clk_cpucl1_cmuref_sud_lut_params},
	{235000, div_clk_cpucl1_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_cpucl2_cmuref_lut[] = {
	{1500000, div_clk_cpucl2_cmuref_sod_lut_params},
	{1200000, div_clk_cpucl2_cmuref_od_lut_params},
	{1000000, div_clk_cpucl2_cmuref_nm_lut_params},
	{750000, div_clk_cpucl2_cmuref_ud_lut_params},
	{600000, div_clk_cpucl2_cmuref_sud_lut_params},
	{400000, div_clk_cpucl2_cmuref_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_pericx_usixx_usi_lut[] = {
	{400000, div_clk_peric_400_lut_params},
	{200000, div_clk_peric_200_lut_params},
	{133000, div_clk_peric_133_lut_params},
	{100000, div_clk_peric_100_lut_params},
	{66000, div_clk_peric_66_lut_params},
	{50000, div_clk_peric_50_lut_params},
	{40000, div_clk_peric_40_lut_params},
	{24576, div_clk_peric_24_lut_params},
	{12288, div_clk_peric_12_lut_params},
	{8192, div_clk_peric_8_lut_params},
	{6144, div_clk_peric_6_lut_params},
	{4000, div_clk_peric_4_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_peric0_ip_lut[] = {
	{400000, mux_clkcmu_peric0_ip_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_peric1_usi11_usi_lut[] = {
	{400000, div_clk_peric1_usi11_usi_uud_lut_params},
};
struct vclk_lut cmucal_vclk_mux_clkcmu_peric1_ip_lut[] = {
	{400000, mux_clkcmu_peric1_ip_uud_lut_params},
};
struct vclk_lut cmucal_vclk_div_clk_top_hsi0_bus_lut[] = {
	{266500, div_clk_top_hsi0_bus_266_params},
	{177666, div_clk_top_hsi0_bus_177_params},
	{106600, div_clk_top_hsi0_bus_106_params},
	{80000, div_clk_top_hsi0_bus_80_params},
	{66666, div_clk_top_hsi0_bus_66_params},
};

/* COMMON VCLK -> LUT List */
struct vclk_lut cmucal_vclk_blk_cmu_lut[] = {
	{800000, blk_cmu_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_hsi0_lut[] = {
	{266625, blk_hsi0_nm_lut_params},
};
struct vclk_lut cmucal_vclk_blk_s2d_lut[] = {
	{400000, blk_s2d_nm_lut_params},
};
struct vclk_lut cmucal_vclk_blk_apm_lut[] = {
	{201000, blk_apm_nm_lut_params},
};
struct vclk_lut cmucal_vclk_blk_bus0_lut[] = {
	{266625, blk_bus0_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_core_lut[] = {
	{400000, blk_core_od_lut_params},
	{355500, blk_core_uud_lut_params},
	{316667, blk_core_nm_lut_params},
};
struct vclk_lut cmucal_vclk_blk_cpucl0_lut[] = {
	{2100000, blk_cpucl0_sod_lut_params},
	{1870000, blk_cpucl0_od_lut_params},
	{1200000, blk_cpucl0_nm_lut_params},
	{1100000, blk_cpucl0_ud_lut_params},
	{600000, blk_cpucl0_sud_lut_params},
	{400000, blk_cpucl0_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_cpucl1_lut[] = {
	{2470000, blk_cpucl1_sod_lut_params},
	{2200000, blk_cpucl1_od_lut_params},
	{1920000, blk_cpucl1_nm_lut_params},
	{1360000, blk_cpucl1_ud_lut_params},
	{700000, blk_cpucl1_sud_lut_params},
	{470000, blk_cpucl1_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_cpucl2_lut[] = {
	{3000000, blk_cpucl2_sod_lut_params},
	{2400000, blk_cpucl2_od_lut_params},
	{2000000, blk_cpucl2_nm_lut_params},
	{1500000, blk_cpucl2_ud_lut_params},
	{1200000, blk_cpucl2_sud_lut_params},
	{800000, blk_cpucl2_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_g3d_lut[] = {
	{1066500, blk_g3d_nm_lut_params},
	{830000, blk_g3d_ud_lut_params},
	{640000, blk_g3d_sud_lut_params},
	{320000, blk_g3d_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_bo_lut[] = {
	{333000, blk_bo_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_bus1_lut[] = {
	{311000, blk_bus1_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_csis_lut[] = {
	{333000, blk_csis_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_disp_lut[] = {
	{166500, blk_disp_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_dns_lut[] = {
	{333000, blk_dns_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_dpu_lut[] = {
	{166500, blk_dpu_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_eh_lut[] = {
	{355500, blk_eh_uud_lut_params},
	{316667, blk_eh_od_lut_params},
};
struct vclk_lut cmucal_vclk_blk_g2d_lut[] = {
	{266625, blk_g2d_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_g3aa_lut[] = {
	{333000, blk_g3aa_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_gdc_lut[] = {
	{333000, blk_gdc_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_ipp_lut[] = {
	{333000, blk_ipp_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_itp_lut[] = {
	{333000, blk_itp_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_mcsc_lut[] = {
	{333000, blk_mcsc_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_mfc_lut[] = {
	{177750, blk_mfc_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_pdp_lut[] = {
	{333000, blk_pdp_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_tnr_lut[] = {
	{333000, blk_tnr_uud_lut_params},
};
struct vclk_lut cmucal_vclk_blk_tpu_lut[] = {
	{350000, blk_tpu_od_lut_params},
	{266625, blk_tpu_ud_lut_params},
};

/* Switch VCLK -> LUT Parameter List */
struct switch_lut mux_clk_core_bus_lut[] = {
	{1066500, 0, 0},
	{711000, 4, 0},
	{355500, 0, 2},
	{213300, 6, 1},
};
struct switch_lut mux_clk_g3d_stacks_lut[] = {
	{800000, 0, 0},
	{533250, 4, 0},
	{400000, 0, 1},
	{200000, 0, 3},
};
struct switch_lut mux_clk_g3d_l2_glb_lut[] = {
	{1066500, 0, 0},
	{800000, 2, 0},
	{622000, 5, 0},
	{311000, 5, 1},
};
struct switch_lut mux_clk_tpu_tpu_lut[] = {
	{1066500, 0, 0},
	{666000, 3, 0},
	{533250, 0, 1},
	{200000, 2, 3},
};
struct switch_lut mux_clk_tpu_tpuctl_lut[] = {
	{1066500, 0, 0},
};
/*================================ SWPLL List =================================*/
struct vclk_switch switch_vdd_mif[] = {
	{MUX_CLK_CORE_BUS, MUX_CLKCMU_CORE_BUS, CLKCMU_CORE_BUS, GATE_CLKCMU_CORE_BUS, MUX_CLKCMU_CORE_BUS_USER, mux_clk_core_bus_lut, 4},
};
struct vclk_switch switch_vdd_g3d[] = {
	{MUX_CLK_G3D_STACKS, MUX_CLKCMU_G3D_SWITCH, CLKCMU_G3D_SWITCH, GATE_CLKCMU_G3D_SWITCH, MUX_CLKCMU_G3D_SWITCH_USER, mux_clk_g3d_stacks_lut, 4},
	{MUX_CLK_G3D_L2_GLB, MUX_CLKCMU_G3D_GLB, CLKCMU_G3D_GLB, GATE_CLKCMU_G3D_GLB, MUX_CLKCMU_G3D_GLB_USER, mux_clk_g3d_l2_glb_lut, 4},
};
struct vclk_switch switch_vdd_tpu[] = {
	{MUX_CLK_TPU_TPU, MUX_CLKCMU_TPU_TPU, CLKCMU_TPU_TPU, GATE_CLKCMU_TPU_TPU, MUX_CLKCMU_TPU_TPU_USER, mux_clk_tpu_tpu_lut, 4},
	{MUX_CLK_TPU_TPUCTL, MUX_CLKCMU_TPU_TPUCTL, CLKCMU_TPU_TPUCTL, GATE_CLKCMU_TPU_TPUCTL, MUX_CLKCMU_TPU_TPUCTL_USER, mux_clk_tpu_tpuctl_lut, 1},
};

/*================================ VCLK List =================================*/
unsigned int cmucal_vclk_size = 944;
struct vclk cmucal_vclk_list[] = {

/* DVFS VCLK*/
	CMUCAL_VCLK(VCLK_VDD_INT, cmucal_vclk_vdd_int_lut, cmucal_vclk_vdd_int, NULL, NULL),
	CMUCAL_VCLK(VCLK_VDD_MIF, cmucal_vclk_vdd_mif_lut, cmucal_vclk_vdd_mif, NULL, switch_vdd_mif),
	CMUCAL_VCLK(VCLK_VDD_G3D, cmucal_vclk_vdd_g3d_lut, cmucal_vclk_vdd_g3d, NULL, switch_vdd_g3d),
	CMUCAL_VCLK(VCLK_VDD_CPUCL0, cmucal_vclk_vdd_cpucl0_lut, cmucal_vclk_vdd_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_VDD_CPUCL1, cmucal_vclk_vdd_cpucl1_lut, cmucal_vclk_vdd_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_VDD_TPU, cmucal_vclk_vdd_tpu_lut, cmucal_vclk_vdd_tpu, NULL, switch_vdd_tpu),
	CMUCAL_VCLK(VCLK_VDD_CPUCL2, cmucal_vclk_vdd_cpucl2_lut, cmucal_vclk_vdd_cpucl2, NULL, NULL),

/* SPECIAL VCLK*/
	CMUCAL_VCLK(VCLK_MUX_BUS0_CMUREF, cmucal_vclk_mux_bus0_cmuref_lut, cmucal_vclk_mux_bus0_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_BUS1_CMUREF, cmucal_vclk_mux_bus1_cmuref_lut, cmucal_vclk_mux_bus1_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CMU_CMUREF, cmucal_vclk_mux_cmu_cmuref_lut, cmucal_vclk_mux_cmu_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CORE_CMUREF, cmucal_vclk_mux_core_cmuref_lut, cmucal_vclk_mux_core_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CPUCL1_CMUREF, cmucal_vclk_mux_cpucl1_cmuref_lut, cmucal_vclk_mux_cpucl1_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CPUCL2_CMUREF, cmucal_vclk_mux_cpucl2_cmuref_lut, cmucal_vclk_mux_cpucl2_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLK_HSI0_USB20_REF, cmucal_vclk_mux_clk_hsi0_usb20_ref_lut, cmucal_vclk_mux_clk_hsi0_usb20_ref, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_HSI0_USBDPDBG, cmucal_vclk_mux_clkcmu_hsi0_usbdpdbg_lut, cmucal_vclk_mux_clkcmu_hsi0_usbdpdbg, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_MIF_CMUREF, cmucal_vclk_mux_mif_cmuref_lut, cmucal_vclk_mux_mif_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLKCMU_HSI0_DPGTC, cmucal_vclk_clkcmu_hsi0_dpgtc_lut, cmucal_vclk_clkcmu_hsi0_dpgtc, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_HSI1_PCIE, cmucal_vclk_mux_clkcmu_hsi1_pcie_lut, cmucal_vclk_mux_clkcmu_hsi1_pcie, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_HSI2_PCIE, cmucal_vclk_mux_clkcmu_hsi2_pcie_lut, cmucal_vclk_mux_clkcmu_hsi2_pcie, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLKCMU_TPU_UART, cmucal_vclk_clkcmu_tpu_uart_lut, cmucal_vclk_clkcmu_tpu_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_APM_USI0_USI, cmucal_vclk_div_clk_apm_usi0_usi_lut, cmucal_vclk_div_clk_apm_usi0_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_APM_USI0_UART, cmucal_vclk_div_clk_apm_usi0_uart_lut, cmucal_vclk_div_clk_apm_usi0_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_APM_USI1_UART, cmucal_vclk_div_clk_apm_usi1_uart_lut, cmucal_vclk_div_clk_apm_usi1_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_HPM, cmucal_vclk_mux_clkcmu_hpm_lut, cmucal_vclk_mux_clkcmu_hpm, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK0, cmucal_vclk_mux_clkcmu_cis_clk0_lut, cmucal_vclk_mux_clkcmu_cis_clk0, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK1, cmucal_vclk_mux_clkcmu_cis_clk1_lut, cmucal_vclk_mux_clkcmu_cis_clk1, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK2, cmucal_vclk_mux_clkcmu_cis_clk2_lut, cmucal_vclk_mux_clkcmu_cis_clk2, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK3, cmucal_vclk_mux_clkcmu_cis_clk3_lut, cmucal_vclk_mux_clkcmu_cis_clk3, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK4, cmucal_vclk_mux_clkcmu_cis_clk4_lut, cmucal_vclk_mux_clkcmu_cis_clk4, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK5, cmucal_vclk_mux_clkcmu_cis_clk5_lut, cmucal_vclk_mux_clkcmu_cis_clk5, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK6, cmucal_vclk_mux_clkcmu_cis_clk6_lut, cmucal_vclk_mux_clkcmu_cis_clk6, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_CIS_CLK7, cmucal_vclk_mux_clkcmu_cis_clk7_lut, cmucal_vclk_mux_clkcmu_cis_clk7, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_SLC_DCLK, cmucal_vclk_div_clk_slc_dclk_lut, cmucal_vclk_div_clk_slc_dclk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_SLC1_DCLK, cmucal_vclk_div_clk_slc1_dclk_lut, cmucal_vclk_div_clk_slc1_dclk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_SLC2_DCLK, cmucal_vclk_div_clk_slc2_dclk_lut, cmucal_vclk_div_clk_slc2_dclk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_SLC3_DCLK, cmucal_vclk_div_clk_slc3_dclk_lut, cmucal_vclk_div_clk_slc3_dclk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_CPUCL0_CMUREF, cmucal_vclk_div_clk_cpucl0_cmuref_lut, cmucal_vclk_div_clk_cpucl0_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_CLUSTER0_PERIPHCLK, cmucal_vclk_div_clk_cluster0_periphclk_lut, cmucal_vclk_div_clk_cluster0_periphclk, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_CPUCL1_CMUREF, cmucal_vclk_div_clk_cpucl1_cmuref_lut, cmucal_vclk_div_clk_cpucl1_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_CPUCL2_CMUREF, cmucal_vclk_div_clk_cpucl2_cmuref_lut, cmucal_vclk_div_clk_cpucl2_cmuref, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI0_UART, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi0_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI1_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi1_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI2_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi2_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI3_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi3_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI4_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi4_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI5_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi5_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI6_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi6_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI7_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi7_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI8_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi8_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_USI14_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_usi14_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC0_I3C, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric0_i3c, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_PERIC0_IP, cmucal_vclk_mux_clkcmu_peric0_ip_lut, cmucal_vclk_mux_clkcmu_peric0_ip, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI0_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric1_usi0_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI9_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric1_usi9_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI10_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric1_usi10_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI11_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric1_usi11_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI12_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric1_usi12_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_USI13_USI, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric1_usi13_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_PERIC1_I3C, cmucal_vclk_div_clk_pericx_usixx_usi_lut, cmucal_vclk_div_clk_peric1_i3c, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_PERIC1_IP, cmucal_vclk_mux_clkcmu_peric1_ip_lut, cmucal_vclk_mux_clkcmu_peric1_ip, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_TOP_HSI0_BUS, cmucal_vclk_div_clk_top_hsi0_bus_lut, cmucal_vclk_div_clk_top_hsi0_bus, NULL, NULL),

/* COMMON VCLK*/
	CMUCAL_VCLK(VCLK_BLK_CMU, cmucal_vclk_blk_cmu_lut, cmucal_vclk_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_HSI0, cmucal_vclk_blk_hsi0_lut, cmucal_vclk_blk_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_S2D, cmucal_vclk_blk_s2d_lut, cmucal_vclk_blk_s2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_APM, cmucal_vclk_blk_apm_lut, cmucal_vclk_blk_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_BUS0, cmucal_vclk_blk_bus0_lut, cmucal_vclk_blk_bus0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CORE, cmucal_vclk_blk_core_lut, cmucal_vclk_blk_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CPUCL0, cmucal_vclk_blk_cpucl0_lut, cmucal_vclk_blk_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CPUCL1, cmucal_vclk_blk_cpucl1_lut, cmucal_vclk_blk_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CPUCL2, cmucal_vclk_blk_cpucl2_lut, cmucal_vclk_blk_cpucl2, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_G3D, cmucal_vclk_blk_g3d_lut, cmucal_vclk_blk_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_BO, cmucal_vclk_blk_bo_lut, cmucal_vclk_blk_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_BUS1, cmucal_vclk_blk_bus1_lut, cmucal_vclk_blk_bus1, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CSIS, cmucal_vclk_blk_csis_lut, cmucal_vclk_blk_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_DISP, cmucal_vclk_blk_disp_lut, cmucal_vclk_blk_disp, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_DNS, cmucal_vclk_blk_dns_lut, cmucal_vclk_blk_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_DPU, cmucal_vclk_blk_dpu_lut, cmucal_vclk_blk_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_EH, cmucal_vclk_blk_eh_lut, cmucal_vclk_blk_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_G2D, cmucal_vclk_blk_g2d_lut, cmucal_vclk_blk_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_G3AA, cmucal_vclk_blk_g3aa_lut, cmucal_vclk_blk_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_GDC, cmucal_vclk_blk_gdc_lut, cmucal_vclk_blk_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_IPP, cmucal_vclk_blk_ipp_lut, cmucal_vclk_blk_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_ITP, cmucal_vclk_blk_itp_lut, cmucal_vclk_blk_itp, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_MCSC, cmucal_vclk_blk_mcsc_lut, cmucal_vclk_blk_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_MFC, cmucal_vclk_blk_mfc_lut, cmucal_vclk_blk_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_PDP, cmucal_vclk_blk_pdp_lut, cmucal_vclk_blk_pdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_TNR, cmucal_vclk_blk_tnr_lut, cmucal_vclk_blk_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_TPU, cmucal_vclk_blk_tpu_lut, cmucal_vclk_blk_tpu, NULL, NULL),

/* GATE VCLK*/
	CMUCAL_VCLK(VCLK_IP_AOC_CMU_AOC, NULL, cmucal_vclk_ip_aoc_cmu_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BAAW_AOC, NULL, cmucal_vclk_ip_baaw_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_AOC, NULL, cmucal_vclk_ip_d_tzpc_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_AOC, NULL, cmucal_vclk_ip_gpc_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_HSI0AOC, NULL, cmucal_vclk_ip_lhm_axi_d_hsi0aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_AOC, NULL, cmucal_vclk_ip_lhm_axi_p_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_AOC, NULL, cmucal_vclk_ip_lhs_axi_d_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_AOCAPM, NULL, cmucal_vclk_ip_lhs_axi_p_aocapm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_AOCHSI0, NULL, cmucal_vclk_ip_lhs_axi_p_aochsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_AOC, NULL, cmucal_vclk_ip_ppmu_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_USB, NULL, cmucal_vclk_ip_ppmu_usb, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_AOC, NULL, cmucal_vclk_ip_ssmt_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_AOC, NULL, cmucal_vclk_ip_sysmmu_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_AOC, NULL, cmucal_vclk_ip_sysreg_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_AOC, NULL, cmucal_vclk_ip_uasc_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_DP_AOC, NULL, cmucal_vclk_ip_xiu_dp_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P_AOC, NULL, cmucal_vclk_ip_xiu_p_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T_AOC, NULL, cmucal_vclk_ip_lhs_atb_t_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_AOC, NULL, cmucal_vclk_ip_lhm_axi_g_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AOC_SYSCTRL_APB, NULL, cmucal_vclk_ip_aoc_sysctrl_apb, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_APM, NULL, cmucal_vclk_ip_lhs_axi_d_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_APM, NULL, cmucal_vclk_ip_lhm_axi_p_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_WDT_APM, NULL, cmucal_vclk_ip_wdt_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_APM, NULL, cmucal_vclk_ip_sysreg_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_AP, NULL, cmucal_vclk_ip_mailbox_apm_ap, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_PMU_ALIVE, NULL, cmucal_vclk_ip_apbif_pmu_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_INTMEM, NULL, cmucal_vclk_ip_intmem, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_G_SCAN2DRAM, NULL, cmucal_vclk_ip_lhs_axi_g_scan2dram, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PMU_INTR_GEN, NULL, cmucal_vclk_ip_pmu_intr_gen, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SPEEDY_APM, NULL, cmucal_vclk_ip_speedy_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_DP_APM, NULL, cmucal_vclk_ip_xiu_dp_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_CMU_APM, NULL, cmucal_vclk_ip_apm_cmu_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GREBEINTEGRATION, NULL, cmucal_vclk_ip_grebeintegration, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_GPIO_ALIVE, NULL, cmucal_vclk_ip_apbif_gpio_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_TRTC, NULL, cmucal_vclk_ip_apbif_trtc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_APM, NULL, cmucal_vclk_ip_d_tzpc_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_AOCAPM, NULL, cmucal_vclk_ip_lhm_axi_p_aocapm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_AOC, NULL, cmucal_vclk_ip_mailbox_apm_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_DBGCORE, NULL, cmucal_vclk_ip_mailbox_ap_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_G_DBGCORE, NULL, cmucal_vclk_ip_lhs_axi_g_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_RTC, NULL, cmucal_vclk_ip_apbif_rtc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SPEEDY_SUB_APM, NULL, cmucal_vclk_ip_speedy_sub_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_GSA, NULL, cmucal_vclk_ip_mailbox_apm_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_AP_AOC, NULL, cmucal_vclk_ip_mailbox_ap_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D_APM, NULL, cmucal_vclk_ip_ssmt_d_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G_DBGCORE, NULL, cmucal_vclk_ip_ssmt_g_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D_APM, NULL, cmucal_vclk_ip_sysmmu_d_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_APM, NULL, cmucal_vclk_ip_gpc_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_APM, NULL, cmucal_vclk_ip_uasc_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_DBGCORE, NULL, cmucal_vclk_ip_uasc_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_P_APM, NULL, cmucal_vclk_ip_uasc_p_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_P_AOCAPM, NULL, cmucal_vclk_ip_uasc_p_aocapm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBIF_GPIO_FAR_ALIVE, NULL, cmucal_vclk_ip_apbif_gpio_far_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ROM_CRC32_HOST, NULL, cmucal_vclk_ip_rom_crc32_host, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SS_DBGCORE, NULL, cmucal_vclk_ip_ss_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_SWD, NULL, cmucal_vclk_ip_mailbox_apm_swd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MAILBOX_APM_TPU, NULL, cmucal_vclk_ip_mailbox_apm_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_SWD, NULL, cmucal_vclk_ip_lhm_axi_g_swd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_G_SWD, NULL, cmucal_vclk_ip_uasc_g_swd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_USI0_UART, NULL, cmucal_vclk_ip_apm_usi0_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_USI1_UART, NULL, cmucal_vclk_ip_apm_usi1_uart, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APM_USI0_USI, NULL, cmucal_vclk_ip_apm_usi0_usi, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BO_CMU_BO, NULL, cmucal_vclk_ip_bo_cmu_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_BO, NULL, cmucal_vclk_ip_lhs_axi_d_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_BO, NULL, cmucal_vclk_ip_lhm_axi_p_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_BO, NULL, cmucal_vclk_ip_ppmu_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_BO, NULL, cmucal_vclk_ip_sysmmu_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_SYSMMU_S1_NS_BO, NULL, cmucal_vclk_ip_as_apb_sysmmu_s1_ns_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_BO, NULL, cmucal_vclk_ip_sysreg_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_BO, NULL, cmucal_vclk_ip_ssmt_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_BO, NULL, cmucal_vclk_ip_d_tzpc_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_BO, NULL, cmucal_vclk_ip_gpc_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_BO, NULL, cmucal_vclk_ip_uasc_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_AXI_P_BO, NULL, cmucal_vclk_ip_as_axi_p_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BO, NULL, cmucal_vclk_ip_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BUS0_CMU_BUS0, NULL, cmucal_vclk_ip_bus0_cmu_bus0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_BUS0, NULL, cmucal_vclk_ip_trex_d_bus0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_BUS0, NULL, cmucal_vclk_ip_d_tzpc_bus0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D_HSI0, NULL, cmucal_vclk_ip_lhm_acel_d_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D_HSI1, NULL, cmucal_vclk_ip_lhm_acel_d_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_AOC, NULL, cmucal_vclk_ip_lhm_axi_d_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_APM, NULL, cmucal_vclk_ip_lhm_axi_d_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_GSA, NULL, cmucal_vclk_ip_lhm_axi_d_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_AOC, NULL, cmucal_vclk_ip_lhs_axi_p_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_GSA, NULL, cmucal_vclk_ip_lhs_axi_p_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_HSI0, NULL, cmucal_vclk_ip_lhs_axi_p_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_HSI1, NULL, cmucal_vclk_ip_lhs_axi_p_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_BUS0, NULL, cmucal_vclk_ip_sysreg_bus0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_P_BUS0, NULL, cmucal_vclk_ip_trex_p_bus0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_BUS0, NULL, cmucal_vclk_ip_gpc_bus0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_CSSYS, NULL, cmucal_vclk_ip_lhm_axi_g_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_DBG_G_BUS0, NULL, cmucal_vclk_ip_lhs_dbg_g_bus0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_AOC, NULL, cmucal_vclk_ip_ppc_event_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CYCLE_AOC, NULL, cmucal_vclk_ip_ppc_cycle_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BUS1_CMU_BUS1, NULL, cmucal_vclk_ip_bus1_cmu_bus1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_BUS1, NULL, cmucal_vclk_ip_sysreg_bus1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D0_G2D, NULL, cmucal_vclk_ip_lhm_axi_d0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D1_G2D, NULL, cmucal_vclk_ip_lhm_axi_d1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D2_G2D, NULL, cmucal_vclk_ip_lhm_acel_d2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D0_CSIS, NULL, cmucal_vclk_ip_lhm_axi_d0_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D_MISC, NULL, cmucal_vclk_ip_lhm_acel_d_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D0_DPU, NULL, cmucal_vclk_ip_lhm_axi_d0_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D0_MFC, NULL, cmucal_vclk_ip_lhm_axi_d0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D1_DPU, NULL, cmucal_vclk_ip_lhm_axi_d1_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D1_MFC, NULL, cmucal_vclk_ip_lhm_axi_d1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D2_DPU, NULL, cmucal_vclk_ip_lhm_axi_d2_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_DPU, NULL, cmucal_vclk_ip_lhs_axi_p_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_TNR, NULL, cmucal_vclk_ip_lhs_axi_p_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_HSI2, NULL, cmucal_vclk_ip_lhs_axi_p_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_G2D, NULL, cmucal_vclk_ip_lhs_axi_p_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_DNS, NULL, cmucal_vclk_ip_lhs_axi_p_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_ITP, NULL, cmucal_vclk_ip_lhs_axi_p_itp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_MFC, NULL, cmucal_vclk_ip_lhs_axi_p_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D1_CSIS, NULL, cmucal_vclk_ip_lhm_axi_d1_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_MCSC, NULL, cmucal_vclk_ip_lhs_axi_p_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D_HSI2, NULL, cmucal_vclk_ip_lhm_acel_d_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_BO, NULL, cmucal_vclk_ip_lhm_axi_d_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_BUS1, NULL, cmucal_vclk_ip_d_tzpc_bus1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_BUS1, NULL, cmucal_vclk_ip_trex_d_bus1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_BO, NULL, cmucal_vclk_ip_lhs_axi_p_bo, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_CSIS, NULL, cmucal_vclk_ip_lhs_axi_p_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_G3AA, NULL, cmucal_vclk_ip_lhs_axi_p_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_IPP, NULL, cmucal_vclk_ip_lhs_axi_p_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_BUS1, NULL, cmucal_vclk_ip_gpc_bus1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_G3AA, NULL, cmucal_vclk_ip_lhm_axi_d_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_DNS, NULL, cmucal_vclk_ip_lhm_axi_d_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_IPP, NULL, cmucal_vclk_ip_lhm_axi_d_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D0_MCSC, NULL, cmucal_vclk_ip_lhm_axi_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D0_TNR, NULL, cmucal_vclk_ip_lhm_axi_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D1_MCSC, NULL, cmucal_vclk_ip_lhm_axi_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D1_TNR, NULL, cmucal_vclk_ip_lhm_axi_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_P_BUS1, NULL, cmucal_vclk_ip_trex_p_bus1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D0_GDC, NULL, cmucal_vclk_ip_lhm_axi_d0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D1_GDC, NULL, cmucal_vclk_ip_lhm_axi_d1_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D2_GDC, NULL, cmucal_vclk_ip_lhm_axi_d2_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D2_TNR, NULL, cmucal_vclk_ip_lhm_axi_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D3_TNR, NULL, cmucal_vclk_ip_lhm_axi_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_DBG_G_BUS1, NULL, cmucal_vclk_ip_lhs_dbg_g_bus1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D2_MCSC, NULL, cmucal_vclk_ip_lhm_axi_d2_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D4_TNR, NULL, cmucal_vclk_ip_lhm_axi_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_DISP, NULL, cmucal_vclk_ip_lhs_axi_p_disp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_PDP, NULL, cmucal_vclk_ip_lhs_axi_p_pdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_GDC, NULL, cmucal_vclk_ip_lhs_axi_p_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BUS2_CMU_BUS2, NULL, cmucal_vclk_ip_bus2_cmu_bus2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_BUS2, NULL, cmucal_vclk_ip_trex_d_bus2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_BUS2, NULL, cmucal_vclk_ip_sysreg_bus2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D0_G3D, NULL, cmucal_vclk_ip_lhm_acel_d0_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_BUS2, NULL, cmucal_vclk_ip_d_tzpc_bus2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D1_G3D, NULL, cmucal_vclk_ip_lhm_acel_d1_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D2_G3D, NULL, cmucal_vclk_ip_lhm_acel_d2_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D3_G3D, NULL, cmucal_vclk_ip_lhm_acel_d3_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G3D0, NULL, cmucal_vclk_ip_ssmt_g3d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_TPU, NULL, cmucal_vclk_ip_lhm_axi_d_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_G3D0, NULL, cmucal_vclk_ip_sysmmu_g3d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_G3D, NULL, cmucal_vclk_ip_lhs_axi_p_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_TPU, NULL, cmucal_vclk_ip_lhs_axi_p_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_BUS2, NULL, cmucal_vclk_ip_gpc_bus2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G3D1, NULL, cmucal_vclk_ip_ssmt_g3d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G3D2, NULL, cmucal_vclk_ip_ssmt_g3d2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G3D3, NULL, cmucal_vclk_ip_ssmt_g3d3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_G3D1, NULL, cmucal_vclk_ip_sysmmu_g3d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPCFW_G3D0, NULL, cmucal_vclk_ip_ppcfw_g3d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_G3D2, NULL, cmucal_vclk_ip_sysmmu_g3d2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_G3D3, NULL, cmucal_vclk_ip_sysmmu_g3d3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_SYSMMU_G3D0, NULL, cmucal_vclk_ip_ad_apb_sysmmu_g3d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_P_BUS2, NULL, cmucal_vclk_ip_trex_p_bus2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_DBG_G_BUS2, NULL, cmucal_vclk_ip_lhs_dbg_g_bus2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_BUS2_S0, NULL, cmucal_vclk_ip_ppc_event_bus2_s0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_BUS2_S1, NULL, cmucal_vclk_ip_ppc_event_bus2_s1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_BUS2_S2, NULL, cmucal_vclk_ip_ppc_event_bus2_s2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_BUS2_S3, NULL, cmucal_vclk_ip_ppc_event_bus2_s3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_G3D0, NULL, cmucal_vclk_ip_ppc_event_g3d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_G3D1, NULL, cmucal_vclk_ip_ppc_event_g3d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_G3D2, NULL, cmucal_vclk_ip_ppc_event_g3d2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_G3D3, NULL, cmucal_vclk_ip_ppc_event_g3d3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EVENT_TPU, NULL, cmucal_vclk_ip_ppc_event_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CYCLE_BUS2_S0, NULL, cmucal_vclk_ip_ppc_cycle_bus2_s0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CYCLE_G3D0, NULL, cmucal_vclk_ip_ppc_cycle_g3d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CYCLE_TPU, NULL, cmucal_vclk_ip_ppc_cycle_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPCFW_G3D1, NULL, cmucal_vclk_ip_ppcfw_g3d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CORE_CMU_CORE, NULL, cmucal_vclk_ip_core_cmu_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_CORE, NULL, cmucal_vclk_ip_sysreg_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_P_CORE, NULL, cmucal_vclk_ip_trex_p_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_MIF1, NULL, cmucal_vclk_ip_lhs_axi_p_mif1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_PERIC0, NULL, cmucal_vclk_ip_lhs_axi_p_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_MIF0, NULL, cmucal_vclk_ip_lhs_axi_p_mif0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACE_D0_CLUSTER0, NULL, cmucal_vclk_ip_lhm_ace_d0_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACE_D1_CLUSTER0, NULL, cmucal_vclk_ip_lhm_ace_d1_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_D_CORE, NULL, cmucal_vclk_ip_trex_d_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_PERIC1, NULL, cmucal_vclk_ip_lhs_axi_p_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_CCI, NULL, cmucal_vclk_ip_ad_apb_cci, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_APM, NULL, cmucal_vclk_ip_lhs_axi_p_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_CORE, NULL, cmucal_vclk_ip_d_tzpc_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_MISC, NULL, cmucal_vclk_ip_lhs_axi_p_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_MIF2, NULL, cmucal_vclk_ip_lhs_axi_p_mif2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_MIF3, NULL, cmucal_vclk_ip_lhs_axi_p_mif3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BDU, NULL, cmucal_vclk_ip_bdu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_CORE, NULL, cmucal_vclk_ip_gpc_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T_BDU, NULL, cmucal_vclk_ip_lhs_atb_t_bdu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T_SLC, NULL, cmucal_vclk_ip_lhs_atb_t_slc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_ACE_CPUCL0, NULL, cmucal_vclk_ip_ppmu_ace_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_ACE_CPUCL1, NULL, cmucal_vclk_ip_ppmu_ace_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SFR_APBIF_CMU_TOPC, NULL, cmucal_vclk_ip_sfr_apbif_cmu_topc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_BUS2_M0_EVENT, NULL, cmucal_vclk_ip_ppc_bus2_m0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_BUS2_M1_EVENT, NULL, cmucal_vclk_ip_ppc_bus2_m1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_BUS2_M2_EVENT, NULL, cmucal_vclk_ip_ppc_bus2_m2_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_BUS2_M3_EVENT, NULL, cmucal_vclk_ip_ppc_bus2_m3_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_BUS0_M0_EVENT, NULL, cmucal_vclk_ip_ppc_bus0_m0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CPUCL0_D0_CYCLE, NULL, cmucal_vclk_ip_ppc_cpucl0_d0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CB, NULL, cmucal_vclk_ip_slc_cb, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CCI, NULL, cmucal_vclk_ip_cci, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ACEL_D_EH, NULL, cmucal_vclk_ip_lhm_acel_d_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_CPUCL0, NULL, cmucal_vclk_ip_lhs_axi_p_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_EH, NULL, cmucal_vclk_ip_lhs_axi_p_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EH_CYCLE, NULL, cmucal_vclk_ip_ppc_eh_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_GIC, NULL, cmucal_vclk_ip_lhs_axi_p_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_IO_EVENT, NULL, cmucal_vclk_ip_ppc_io_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_EH_EVENT, NULL, cmucal_vclk_ip_ppc_eh_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CPUCL0_D0_EVENT, NULL, cmucal_vclk_ip_ppc_cpucl0_d0_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CCI_M1_EVENT, NULL, cmucal_vclk_ip_ppc_cci_m1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CCI_M2_EVENT, NULL, cmucal_vclk_ip_ppc_cci_m2_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CCI_M3_EVENT, NULL, cmucal_vclk_ip_ppc_cci_m3_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CCI_M4_EVENT, NULL, cmucal_vclk_ip_ppc_cci_m4_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_IO_CYCLE, NULL, cmucal_vclk_ip_ppc_io_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CCI_M1_CYCLE, NULL, cmucal_vclk_ip_ppc_cci_m1_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_BUS2_M0_CYCLE, NULL, cmucal_vclk_ip_ppc_bus2_m0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_BUS0_M0_CYCLE, NULL, cmucal_vclk_ip_ppc_bus0_m0_cycle, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_DBG_CC, NULL, cmucal_vclk_ip_ppc_dbg_cc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MPACE_ASB_D0_MIF, NULL, cmucal_vclk_ip_mpace_asb_d0_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MPACE_ASB_D1_MIF, NULL, cmucal_vclk_ip_mpace_asb_d1_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MPACE_ASB_D2_MIF, NULL, cmucal_vclk_ip_mpace_asb_d2_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MPACE_ASB_D3_MIF, NULL, cmucal_vclk_ip_mpace_asb_d3_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_CPUCL0_D1_EVENT, NULL, cmucal_vclk_ip_ppc_cpucl0_d1_event, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CH, NULL, cmucal_vclk_ip_slc_ch, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CH1, NULL, cmucal_vclk_ip_slc_ch1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CH2, NULL, cmucal_vclk_ip_slc_ch2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SLC_CH3, NULL, cmucal_vclk_ip_slc_ch3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPE425, NULL, cmucal_vclk_ip_cpe425, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GRAY2BIN_ATB_TSVALUE, NULL, cmucal_vclk_ip_gray2bin_atb_tsvalue, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_CORE, NULL, cmucal_vclk_ip_lhm_axi_g_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_CPUCL0, NULL, cmucal_vclk_ip_sysreg_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HPM_APBIF_CPUCL0, NULL, cmucal_vclk_ip_hpm_apbif_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CSSYS, NULL, cmucal_vclk_ip_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T_AOC, NULL, cmucal_vclk_ip_lhm_atb_t_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T_BDU, NULL, cmucal_vclk_ip_lhm_atb_t_bdu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T0_CLUSTER0, NULL, cmucal_vclk_ip_lhm_atb_t0_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T6_CLUSTER0, NULL, cmucal_vclk_ip_lhm_atb_t6_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T1_CLUSTER0, NULL, cmucal_vclk_ip_lhm_atb_t1_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T7_CLUSTER0, NULL, cmucal_vclk_ip_lhm_atb_t7_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T2_CLUSTER0, NULL, cmucal_vclk_ip_lhm_atb_t2_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T3_CLUSTER0, NULL, cmucal_vclk_ip_lhm_atb_t3_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_CPUCL0, NULL, cmucal_vclk_ip_lhm_axi_p_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ACE_D0_CLUSTER0, NULL, cmucal_vclk_ip_lhs_ace_d0_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T0_CLUSTER0, NULL, cmucal_vclk_ip_lhs_atb_t0_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T1_CLUSTER0, NULL, cmucal_vclk_ip_lhs_atb_t1_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T2_CLUSTER0, NULL, cmucal_vclk_ip_lhs_atb_t2_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T3_CLUSTER0, NULL, cmucal_vclk_ip_lhs_atb_t3_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADM_APB_G_CLUSTER0, NULL, cmucal_vclk_ip_adm_apb_g_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL0_CMU_CPUCL0, NULL, cmucal_vclk_ip_cpucl0_cmu_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CLUSTER0, NULL, cmucal_vclk_ip_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T4_CLUSTER0, NULL, cmucal_vclk_ip_lhm_atb_t4_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T5_CLUSTER0, NULL, cmucal_vclk_ip_lhm_atb_t5_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ACE_D1_CLUSTER0, NULL, cmucal_vclk_ip_lhs_ace_d1_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T4_CLUSTER0, NULL, cmucal_vclk_ip_lhs_atb_t4_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T5_CLUSTER0, NULL, cmucal_vclk_ip_lhs_atb_t5_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_CPUCL0, NULL, cmucal_vclk_ip_d_tzpc_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_G_INT_CSSYS, NULL, cmucal_vclk_ip_lhs_axi_g_int_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_INT_CSSYS, NULL, cmucal_vclk_ip_lhm_axi_g_int_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P_CPUCL0, NULL, cmucal_vclk_ip_xiu_p_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HPM_CPUCL0_1, NULL, cmucal_vclk_ip_hpm_cpucl0_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HPM_CPUCL0_0, NULL, cmucal_vclk_ip_hpm_cpucl0_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APB_ASYNC_P_CSSYS_0, NULL, cmucal_vclk_ip_apb_async_p_cssys_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BPS_CPUCL0, NULL, cmucal_vclk_ip_bps_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T6_CLUSTER0, NULL, cmucal_vclk_ip_lhs_atb_t6_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T_GSA, NULL, cmucal_vclk_ip_lhm_atb_t_gsa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T_SLC, NULL, cmucal_vclk_ip_lhm_atb_t_slc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_G_CSSYS, NULL, cmucal_vclk_ip_lhs_axi_g_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_CPUCL0, NULL, cmucal_vclk_ip_gpc_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_DBGCORE, NULL, cmucal_vclk_ip_lhm_axi_g_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_G_INT_DBGCORE, NULL, cmucal_vclk_ip_lhs_axi_g_int_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_DP_CSSYS, NULL, cmucal_vclk_ip_xiu_dp_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_INT_DBGCORE, NULL, cmucal_vclk_ip_lhm_axi_g_int_dbgcore, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_IRI_GICCPU, NULL, cmucal_vclk_ip_lhm_ast_iri_giccpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_ICC_CPUGIC, NULL, cmucal_vclk_ip_lhs_ast_icc_cpugic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_CPUCL0, NULL, cmucal_vclk_ip_ssmt_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_S2_CPUCL0, NULL, cmucal_vclk_ip_sysmmu_s2_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_G_ETR_HSI0, NULL, cmucal_vclk_ip_lhs_axi_g_etr_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_INT_HSI0, NULL, cmucal_vclk_ip_lhm_axi_g_int_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APB_ASYNC_P_CSSYS_4, NULL, cmucal_vclk_ip_apb_async_p_cssys_4, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T0_TPUCPUCL0, NULL, cmucal_vclk_ip_lhm_atb_t0_tpucpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_T1_TPUCPUCL0, NULL, cmucal_vclk_ip_lhm_atb_t1_tpucpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_G_INT_HSI0, NULL, cmucal_vclk_ip_lhs_axi_g_int_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TREX_CPUCL0, NULL, cmucal_vclk_ip_trex_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_G_INT_STM, NULL, cmucal_vclk_ip_lhs_axi_g_int_stm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T7_CLUSTER0, NULL, cmucal_vclk_ip_lhs_atb_t7_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_INT_STM, NULL, cmucal_vclk_ip_lhm_axi_g_int_stm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL1, NULL, cmucal_vclk_ip_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL1_CMU_CPUCL1, NULL, cmucal_vclk_ip_cpucl1_cmu_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL2_CMU_CPUCL2, NULL, cmucal_vclk_ip_cpucl2_cmu_cpucl2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CMU_CPUCL2_SHORTSTOP, NULL, cmucal_vclk_ip_cmu_cpucl2_shortstop, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CPUCL2, NULL, cmucal_vclk_ip_cpucl2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D0_CSIS, NULL, cmucal_vclk_ip_lhs_axi_d0_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_CSIS, NULL, cmucal_vclk_ip_lhm_axi_p_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_CSIS, NULL, cmucal_vclk_ip_sysreg_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CSIS_CMU_CSIS, NULL, cmucal_vclk_ip_csis_cmu_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_ZOTF2_IPPCSIS, NULL, cmucal_vclk_ip_lhm_ast_zotf2_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MIPI_PHY_LINK_WRAP, NULL, cmucal_vclk_ip_mipi_phy_link_wrap, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_CSIS, NULL, cmucal_vclk_ip_d_tzpc_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_ZOTF1_IPPCSIS, NULL, cmucal_vclk_ip_lhm_ast_zotf1_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0, NULL, cmucal_vclk_ip_ppmu_d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_ZOTF0_IPPCSIS, NULL, cmucal_vclk_ip_lhm_ast_zotf0_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_SOTF0_IPPCSIS, NULL, cmucal_vclk_ip_lhm_ast_sotf0_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_SOTF1_IPPCSIS, NULL, cmucal_vclk_ip_lhm_ast_sotf1_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_SOTF2_IPPCSIS, NULL, cmucal_vclk_ip_lhm_ast_sotf2_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF0_CSISPDP, NULL, cmucal_vclk_ip_lhs_ast_otf0_csispdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF1_CSISPDP, NULL, cmucal_vclk_ip_lhs_ast_otf1_csispdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF2_CSISPDP, NULL, cmucal_vclk_ip_lhs_ast_otf2_csispdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_CSIS, NULL, cmucal_vclk_ip_gpc_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_CSIS0, NULL, cmucal_vclk_ip_ad_apb_csis0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1, NULL, cmucal_vclk_ip_ppmu_d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D0_CSIS, NULL, cmucal_vclk_ip_sysmmu_d0_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D1_CSIS, NULL, cmucal_vclk_ip_sysmmu_d1_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1, NULL, cmucal_vclk_ip_ssmt_d1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0, NULL, cmucal_vclk_ip_ssmt_d0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_ZSL1, NULL, cmucal_vclk_ip_qe_zsl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_ZSL2, NULL, cmucal_vclk_ip_qe_zsl2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_ZSL0, NULL, cmucal_vclk_ip_qe_zsl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_STRP0, NULL, cmucal_vclk_ip_qe_strp0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_CSIS, NULL, cmucal_vclk_ip_xiu_d0_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_CSIS, NULL, cmucal_vclk_ip_xiu_d1_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_VO_MCSCCSIS, NULL, cmucal_vclk_ip_lhm_ast_vo_mcsccsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D1_CSIS, NULL, cmucal_vclk_ip_lhs_axi_d1_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF0_PDPCSIS, NULL, cmucal_vclk_ip_lhm_ast_otf0_pdpcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF1_PDPCSIS, NULL, cmucal_vclk_ip_lhm_ast_otf1_pdpcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF2_PDPCSIS, NULL, cmucal_vclk_ip_lhm_ast_otf2_pdpcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_VO_CSISPDP, NULL, cmucal_vclk_ip_lhs_ast_vo_csispdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_PDPCSIS, NULL, cmucal_vclk_ip_lhm_axi_d_pdpcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_STRP2, NULL, cmucal_vclk_ip_qe_strp2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_STRP1, NULL, cmucal_vclk_ip_qe_strp1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D2_CSIS, NULL, cmucal_vclk_ip_xiu_d2_csis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_CSISX8, NULL, cmucal_vclk_ip_csisx8, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_CSIS_DMA0, NULL, cmucal_vclk_ip_qe_csis_dma0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_CSIS_DMA1, NULL, cmucal_vclk_ip_qe_csis_dma1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_CSIS_DMA2, NULL, cmucal_vclk_ip_qe_csis_dma2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_CSIS_DMA3, NULL, cmucal_vclk_ip_qe_csis_dma3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DISP_CMU_DISP, NULL, cmucal_vclk_ip_disp_cmu_disp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_DECON_MAIN, NULL, cmucal_vclk_ip_ad_apb_decon_main, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DPUB, NULL, cmucal_vclk_ip_dpub, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_DISP, NULL, cmucal_vclk_ip_lhm_axi_p_disp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_DISP, NULL, cmucal_vclk_ip_d_tzpc_disp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_DISP, NULL, cmucal_vclk_ip_gpc_disp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_DISP, NULL, cmucal_vclk_ip_sysreg_disp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_DNS, NULL, cmucal_vclk_ip_ad_apb_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_DNS, NULL, cmucal_vclk_ip_d_tzpc_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DNS, NULL, cmucal_vclk_ip_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_DNS, NULL, cmucal_vclk_ip_gpc_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_DNS, NULL, cmucal_vclk_ip_lhm_axi_p_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_DNS, NULL, cmucal_vclk_ip_lhs_axi_d_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_DNS, NULL, cmucal_vclk_ip_ppmu_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_DNS, NULL, cmucal_vclk_ip_ssmt_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_DNS, NULL, cmucal_vclk_ip_sysmmu_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_DNS, NULL, cmucal_vclk_ip_sysreg_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF0_DNSITP, NULL, cmucal_vclk_ip_lhs_ast_otf0_dnsitp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF1_DNSITP, NULL, cmucal_vclk_ip_lhs_ast_otf1_dnsitp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF2_DNSITP, NULL, cmucal_vclk_ip_lhs_ast_otf2_dnsitp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF0_DNSMCSC, NULL, cmucal_vclk_ip_lhs_ast_otf0_dnsmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF1_DNSMCSC, NULL, cmucal_vclk_ip_lhs_ast_otf1_dnsmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF2_DNSMCSC, NULL, cmucal_vclk_ip_lhs_ast_otf2_dnsmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF_ITPDNS, NULL, cmucal_vclk_ip_lhm_ast_otf_itpdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF_DNSGDC, NULL, cmucal_vclk_ip_lhs_ast_otf_dnsgdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_CTL_ITPDNS, NULL, cmucal_vclk_ip_lhm_ast_ctl_itpdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_CTL_DNSITP, NULL, cmucal_vclk_ip_lhs_ast_ctl_dnsitp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_VO_IPPDNS, NULL, cmucal_vclk_ip_lhm_ast_vo_ippdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_VO_DNSTNR, NULL, cmucal_vclk_ip_lhs_ast_vo_dnstnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_PDPDNS, NULL, cmucal_vclk_ip_lhm_axi_d_pdpdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_DNS, NULL, cmucal_vclk_ip_xiu_d_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_IPPDNS, NULL, cmucal_vclk_ip_lhm_axi_d_ippdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_MCSCDNS, NULL, cmucal_vclk_ip_lhm_axi_d_mcscdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_DNS, NULL, cmucal_vclk_ip_qe_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF_IPPDNS, NULL, cmucal_vclk_ip_lhm_ast_otf_ippdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DNS_CMU_DNS, NULL, cmucal_vclk_ip_dns_cmu_dns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DPU_CMU_DPU, NULL, cmucal_vclk_ip_dpu_cmu_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_DPU, NULL, cmucal_vclk_ip_sysreg_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_DPUD0, NULL, cmucal_vclk_ip_sysmmu_dpud0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_DPU, NULL, cmucal_vclk_ip_lhm_axi_p_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D1_DPU, NULL, cmucal_vclk_ip_lhs_axi_d1_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D2_DPU, NULL, cmucal_vclk_ip_lhs_axi_d2_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_DPUD2, NULL, cmucal_vclk_ip_sysmmu_dpud2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_DPUD1, NULL, cmucal_vclk_ip_sysmmu_dpud1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_DPUD0, NULL, cmucal_vclk_ip_ppmu_dpud0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_DPUD1, NULL, cmucal_vclk_ip_ppmu_dpud1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_DPUD2, NULL, cmucal_vclk_ip_ppmu_dpud2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D0_DPU, NULL, cmucal_vclk_ip_lhs_axi_d0_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DPUF, NULL, cmucal_vclk_ip_dpuf, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_DPU, NULL, cmucal_vclk_ip_d_tzpc_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_DPU_DMA, NULL, cmucal_vclk_ip_ad_apb_dpu_dma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_DPU0, NULL, cmucal_vclk_ip_ssmt_dpu0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_DPU1, NULL, cmucal_vclk_ip_ssmt_dpu1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_DPU2, NULL, cmucal_vclk_ip_ssmt_dpu2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_DPU, NULL, cmucal_vclk_ip_gpc_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_EH_CMU_EH, NULL, cmucal_vclk_ip_eh_cmu_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_P_SYSMMU_S2_EH, NULL, cmucal_vclk_ip_as_p_sysmmu_s2_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_EH, NULL, cmucal_vclk_ip_d_tzpc_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_EH, NULL, cmucal_vclk_ip_gpc_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_EH, NULL, cmucal_vclk_ip_lhm_axi_p_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ACEL_D_EH, NULL, cmucal_vclk_ip_lhs_acel_d_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_EH, NULL, cmucal_vclk_ip_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_EH, NULL, cmucal_vclk_ip_ssmt_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_EH, NULL, cmucal_vclk_ip_ppmu_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_EH, NULL, cmucal_vclk_ip_sysmmu_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_EH, NULL, cmucal_vclk_ip_sysreg_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_EH, NULL, cmucal_vclk_ip_uasc_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_AXI_P_EH, NULL, cmucal_vclk_ip_ad_axi_p_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_EH, NULL, cmucal_vclk_ip_qe_eh, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_G2D_CMU_G2D, NULL, cmucal_vclk_ip_g2d_cmu_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_G2D, NULL, cmucal_vclk_ip_ppmu_d0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_G2D, NULL, cmucal_vclk_ip_ppmu_d1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D0_G2D, NULL, cmucal_vclk_ip_sysmmu_d0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_G2D, NULL, cmucal_vclk_ip_sysreg_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D0_G2D, NULL, cmucal_vclk_ip_lhs_axi_d0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D1_G2D, NULL, cmucal_vclk_ip_lhs_axi_d1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D2_G2D, NULL, cmucal_vclk_ip_sysmmu_d2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_G2D, NULL, cmucal_vclk_ip_ppmu_d2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ACEL_D2_G2D, NULL, cmucal_vclk_ip_lhs_acel_d2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_G2D, NULL, cmucal_vclk_ip_ssmt_d0_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_G2D, NULL, cmucal_vclk_ip_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D1_G2D, NULL, cmucal_vclk_ip_sysmmu_d1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_JPEG, NULL, cmucal_vclk_ip_jpeg, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_G2D, NULL, cmucal_vclk_ip_d_tzpc_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_G2D, NULL, cmucal_vclk_ip_ssmt_d1_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D2_G2D, NULL, cmucal_vclk_ip_ssmt_d2_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_G2D, NULL, cmucal_vclk_ip_gpc_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_G2D, NULL, cmucal_vclk_ip_lhm_axi_p_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_G2D, NULL, cmucal_vclk_ip_as_apb_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_JPEG, NULL, cmucal_vclk_ip_as_apb_jpeg, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_G3AA, NULL, cmucal_vclk_ip_lhm_axi_p_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_G3AA, NULL, cmucal_vclk_ip_lhs_axi_d_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APB_ASYNC_TOP_G3AA, NULL, cmucal_vclk_ip_apb_async_top_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_G3AA, NULL, cmucal_vclk_ip_sysreg_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_G3AA_CMU_G3AA, NULL, cmucal_vclk_ip_g3aa_cmu_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_G3AA, NULL, cmucal_vclk_ip_ppmu_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_G3AA, NULL, cmucal_vclk_ip_d_tzpc_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_G3AA, NULL, cmucal_vclk_ip_gpc_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_G3AA, NULL, cmucal_vclk_ip_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_G3AA, NULL, cmucal_vclk_ip_ssmt_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_G3AA, NULL, cmucal_vclk_ip_sysmmu_g3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF0_PDPG3AA, NULL, cmucal_vclk_ip_lhm_ast_otf0_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_YOTF0_PDPG3AA, NULL, cmucal_vclk_ip_lhm_ast_yotf0_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF1_PDPG3AA, NULL, cmucal_vclk_ip_lhm_ast_otf1_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF2_PDPG3AA, NULL, cmucal_vclk_ip_lhm_ast_otf2_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_YOTF1_PDPG3AA, NULL, cmucal_vclk_ip_lhm_ast_yotf1_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_G3D, NULL, cmucal_vclk_ip_lhm_axi_p_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BUSIF_HPMG3D, NULL, cmucal_vclk_ip_busif_hpmg3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HPM_G3D, NULL, cmucal_vclk_ip_hpm_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_G3D, NULL, cmucal_vclk_ip_sysreg_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_G3D_CMU_G3D, NULL, cmucal_vclk_ip_g3d_cmu_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_P_INT_G3D, NULL, cmucal_vclk_ip_lhs_axi_p_int_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPU, NULL, cmucal_vclk_ip_gpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_INT_G3D, NULL, cmucal_vclk_ip_lhm_axi_p_int_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GRAY2BIN_G3D, NULL, cmucal_vclk_ip_gray2bin_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_G3D, NULL, cmucal_vclk_ip_d_tzpc_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_G3D, NULL, cmucal_vclk_ip_gpc_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_G3D, NULL, cmucal_vclk_ip_uasc_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD_APBIF_G3D, NULL, cmucal_vclk_ip_add_apbif_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADD_G3D, NULL, cmucal_vclk_ip_add_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ASB_G3D, NULL, cmucal_vclk_ip_asb_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GDC_CMU_GDC, NULL, cmucal_vclk_ip_gdc_cmu_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_GDC0, NULL, cmucal_vclk_ip_ad_apb_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_GDC1, NULL, cmucal_vclk_ip_ad_apb_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_SCSC, NULL, cmucal_vclk_ip_ad_apb_scsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_GDC, NULL, cmucal_vclk_ip_d_tzpc_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GDC0, NULL, cmucal_vclk_ip_gdc0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GDC1, NULL, cmucal_vclk_ip_gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_GDC, NULL, cmucal_vclk_ip_gpc_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D2_GDC, NULL, cmucal_vclk_ip_lhs_axi_d2_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_GDC, NULL, cmucal_vclk_ip_ppmu_d0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_GDC, NULL, cmucal_vclk_ip_ppmu_d1_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SCSC, NULL, cmucal_vclk_ip_scsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_GDC, NULL, cmucal_vclk_ip_ssmt_d0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_GDC, NULL, cmucal_vclk_ip_ssmt_d1_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D_SCSC, NULL, cmucal_vclk_ip_ssmt_d_scsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D2_GDC, NULL, cmucal_vclk_ip_sysmmu_d2_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_GDC, NULL, cmucal_vclk_ip_sysreg_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_INT_GDC0GDC1, NULL, cmucal_vclk_ip_lhm_ast_int_gdc0gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_INT_GDC1SCSC, NULL, cmucal_vclk_ip_lhm_ast_int_gdc1scsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF_DNSGDC, NULL, cmucal_vclk_ip_lhm_ast_otf_dnsgdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF_TNRGDC, NULL, cmucal_vclk_ip_lhm_ast_otf_tnrgdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_VO_TNRGDC, NULL, cmucal_vclk_ip_lhm_ast_vo_tnrgdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_INT_GDC0GDC1, NULL, cmucal_vclk_ip_lhs_ast_int_gdc0gdc1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_INT_GDC1SCSC, NULL, cmucal_vclk_ip_lhs_ast_int_gdc1scsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_VO_GDCMCSC, NULL, cmucal_vclk_ip_lhs_ast_vo_gdcmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D0_GDC, NULL, cmucal_vclk_ip_sysmmu_d0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D1_GDC, NULL, cmucal_vclk_ip_sysmmu_d1_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D0_GDC, NULL, cmucal_vclk_ip_lhs_axi_d0_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D_SCSC, NULL, cmucal_vclk_ip_ppmu_d_scsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_GDC, NULL, cmucal_vclk_ip_xiu_d_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D1_SCSC, NULL, cmucal_vclk_ip_qe_d1_scsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_SCSC, NULL, cmucal_vclk_ip_qe_d0_scsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_GDC, NULL, cmucal_vclk_ip_lhm_axi_p_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D1_GDC, NULL, cmucal_vclk_ip_lhs_axi_d1_gdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HSI0_CMU_HSI0, NULL, cmucal_vclk_ip_hsi0_cmu_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_USB31DRD, NULL, cmucal_vclk_ip_usb31drd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_ETR_HSI0, NULL, cmucal_vclk_ip_lhm_axi_g_etr_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DP_LINK, NULL, cmucal_vclk_ip_dp_link, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_HSI0, NULL, cmucal_vclk_ip_xiu_d0_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ETR_MIU, NULL, cmucal_vclk_ip_etr_miu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_HSI0_BUS0, NULL, cmucal_vclk_ip_ppmu_hsi0_bus0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_HSI0_AOC, NULL, cmucal_vclk_ip_ppmu_hsi0_aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_HSI0AOC, NULL, cmucal_vclk_ip_lhs_axi_d_hsi0aoc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ACEL_D_HSI0, NULL, cmucal_vclk_ip_lhs_acel_d_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_AOCHSI0, NULL, cmucal_vclk_ip_lhm_axi_p_aochsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_HSI0, NULL, cmucal_vclk_ip_lhm_axi_p_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_HSI0, NULL, cmucal_vclk_ip_gpc_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_HSI0, NULL, cmucal_vclk_ip_d_tzpc_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_USB, NULL, cmucal_vclk_ip_ssmt_usb, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_USB, NULL, cmucal_vclk_ip_sysmmu_usb, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_HSI0, NULL, cmucal_vclk_ip_sysreg_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P_HSI0, NULL, cmucal_vclk_ip_xiu_p_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_HSI0, NULL, cmucal_vclk_ip_xiu_d1_hsi0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_HSI0_CTRL, NULL, cmucal_vclk_ip_uasc_hsi0_ctrl, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_HSI0_LINK, NULL, cmucal_vclk_ip_uasc_hsi0_link, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HSI1_CMU_HSI1, NULL, cmucal_vclk_ip_hsi1_cmu_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ACEL_D_HSI1, NULL, cmucal_vclk_ip_lhs_acel_d_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_HSI1, NULL, cmucal_vclk_ip_lhm_axi_p_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_HSI1, NULL, cmucal_vclk_ip_sysreg_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_HSI1, NULL, cmucal_vclk_ip_xiu_d_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_HSI1, NULL, cmucal_vclk_ip_ppmu_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_HSI1, NULL, cmucal_vclk_ip_sysmmu_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P_HSI1, NULL, cmucal_vclk_ip_xiu_p_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_GEN4_0, NULL, cmucal_vclk_ip_pcie_gen4_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_IA_GEN4A_0, NULL, cmucal_vclk_ip_pcie_ia_gen4a_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_IA_GEN4B_0, NULL, cmucal_vclk_ip_pcie_ia_gen4b_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_HSI1, NULL, cmucal_vclk_ip_d_tzpc_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_HSI1, NULL, cmucal_vclk_ip_gpc_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_HSI1, NULL, cmucal_vclk_ip_ssmt_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_HSI1, NULL, cmucal_vclk_ip_gpio_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PCIE_GEN4A_HSI1, NULL, cmucal_vclk_ip_qe_pcie_gen4a_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PCIE_GEN4B_HSI1, NULL, cmucal_vclk_ip_qe_pcie_gen4b_hsi1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN4A_DBI_0, NULL, cmucal_vclk_ip_uasc_pcie_gen4a_dbi_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN4A_SLV_0, NULL, cmucal_vclk_ip_uasc_pcie_gen4a_slv_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN4B_DBI_0, NULL, cmucal_vclk_ip_uasc_pcie_gen4b_dbi_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN4B_SLV_0, NULL, cmucal_vclk_ip_uasc_pcie_gen4b_slv_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PCIE_IA_GEN4A_0, NULL, cmucal_vclk_ip_ssmt_pcie_ia_gen4a_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PCIE_IA_GEN4B_0, NULL, cmucal_vclk_ip_ssmt_pcie_ia_gen4b_0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HSI2_CMU_HSI2, NULL, cmucal_vclk_ip_hsi2_cmu_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_HSI2, NULL, cmucal_vclk_ip_sysreg_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_HSI2, NULL, cmucal_vclk_ip_gpio_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ACEL_D_HSI2, NULL, cmucal_vclk_ip_lhs_acel_d_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_HSI2, NULL, cmucal_vclk_ip_lhm_axi_p_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_HSI2, NULL, cmucal_vclk_ip_xiu_d_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_P_HSI2, NULL, cmucal_vclk_ip_xiu_p_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_HSI2, NULL, cmucal_vclk_ip_ppmu_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_GEN4_1, NULL, cmucal_vclk_ip_pcie_gen4_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_HSI2, NULL, cmucal_vclk_ip_sysmmu_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_HSI2, NULL, cmucal_vclk_ip_ssmt_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_IA_GEN4A_1, NULL, cmucal_vclk_ip_pcie_ia_gen4a_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_HSI2, NULL, cmucal_vclk_ip_d_tzpc_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UFS_EMBD, NULL, cmucal_vclk_ip_ufs_embd, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PCIE_IA_GEN4B_1, NULL, cmucal_vclk_ip_pcie_ia_gen4b_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_HSI2, NULL, cmucal_vclk_ip_gpc_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MMC_CARD, NULL, cmucal_vclk_ip_mmc_card, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PCIE_GEN4A_HSI2, NULL, cmucal_vclk_ip_qe_pcie_gen4a_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PCIE_GEN4B_HSI2, NULL, cmucal_vclk_ip_qe_pcie_gen4b_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_UFS_EMBD_HSI2, NULL, cmucal_vclk_ip_qe_ufs_embd_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN4A_DBI_1, NULL, cmucal_vclk_ip_uasc_pcie_gen4a_dbi_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN4A_SLV_1, NULL, cmucal_vclk_ip_uasc_pcie_gen4a_slv_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN4B_DBI_1, NULL, cmucal_vclk_ip_uasc_pcie_gen4b_dbi_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_UASC_PCIE_GEN4B_SLV_1, NULL, cmucal_vclk_ip_uasc_pcie_gen4b_slv_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_MMC_CARD_HSI2, NULL, cmucal_vclk_ip_qe_mmc_card_hsi2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PCIE_IA_GEN4A_1, NULL, cmucal_vclk_ip_ssmt_pcie_ia_gen4a_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PCIE_IA_GEN4B_1, NULL, cmucal_vclk_ip_ssmt_pcie_ia_gen4b_1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_IPP_CMU_IPP, NULL, cmucal_vclk_ip_ipp_cmu_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_IPP, NULL, cmucal_vclk_ip_d_tzpc_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_IPP, NULL, cmucal_vclk_ip_lhm_axi_p_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_IPP, NULL, cmucal_vclk_ip_sysreg_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_VO_IPPDNS, NULL, cmucal_vclk_ip_lhs_ast_vo_ippdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_VO_PDPIPP, NULL, cmucal_vclk_ip_lhm_ast_vo_pdpipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_IPP, NULL, cmucal_vclk_ip_ad_apb_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_IPP, NULL, cmucal_vclk_ip_lhs_axi_d_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_SOTF0_IPPCSIS, NULL, cmucal_vclk_ip_lhs_ast_sotf0_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_SOTF1_IPPCSIS, NULL, cmucal_vclk_ip_lhs_ast_sotf1_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_SOTF2_IPPCSIS, NULL, cmucal_vclk_ip_lhs_ast_sotf2_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_ZOTF0_IPPCSIS, NULL, cmucal_vclk_ip_lhs_ast_zotf0_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_ZOTF1_IPPCSIS, NULL, cmucal_vclk_ip_lhs_ast_zotf1_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_ZOTF2_IPPCSIS, NULL, cmucal_vclk_ip_lhs_ast_zotf2_ippcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_IPP, NULL, cmucal_vclk_ip_ppmu_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SIPU_IPP, NULL, cmucal_vclk_ip_sipu_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_IPP, NULL, cmucal_vclk_ip_sysmmu_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_IPP, NULL, cmucal_vclk_ip_gpc_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_THSTAT, NULL, cmucal_vclk_ip_ssmt_thstat, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_IPPDNS, NULL, cmucal_vclk_ip_lhs_axi_d_ippdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_MSA, NULL, cmucal_vclk_ip_ppmu_msa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_ALIGN0, NULL, cmucal_vclk_ip_qe_align0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_ALIGN1, NULL, cmucal_vclk_ip_qe_align1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_ALIGN0, NULL, cmucal_vclk_ip_ssmt_align0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_ALIGN1, NULL, cmucal_vclk_ip_ssmt_align1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D1_IPP, NULL, cmucal_vclk_ip_xiu_d1_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TNR_A, NULL, cmucal_vclk_ip_tnr_a, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_THSTAT, NULL, cmucal_vclk_ip_qe_thstat, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF0_PDPIPP, NULL, cmucal_vclk_ip_lhm_ast_otf0_pdpipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF1_PDPIPP, NULL, cmucal_vclk_ip_lhm_ast_otf1_pdpipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF2_PDPIPP, NULL, cmucal_vclk_ip_lhm_ast_otf2_pdpipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF_IPPDNS, NULL, cmucal_vclk_ip_lhs_ast_otf_ippdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D2_IPP, NULL, cmucal_vclk_ip_xiu_d2_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D0_IPP, NULL, cmucal_vclk_ip_xiu_d0_ipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_FDPIG, NULL, cmucal_vclk_ip_ssmt_fdpig, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_RGBH0, NULL, cmucal_vclk_ip_ssmt_rgbh0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_RGBH1, NULL, cmucal_vclk_ip_ssmt_rgbh1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_RGBH2, NULL, cmucal_vclk_ip_ssmt_rgbh2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_ALIGN2, NULL, cmucal_vclk_ip_ssmt_align2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_ALIGN3, NULL, cmucal_vclk_ip_ssmt_align3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_FDPIG, NULL, cmucal_vclk_ip_qe_fdpig, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_RGBH0, NULL, cmucal_vclk_ip_qe_rgbh0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_RGBH1, NULL, cmucal_vclk_ip_qe_rgbh1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_RGBH2, NULL, cmucal_vclk_ip_qe_rgbh2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_ALIGN2, NULL, cmucal_vclk_ip_qe_align2, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_ALIGN3, NULL, cmucal_vclk_ip_qe_align3, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_TNR_MSA0, NULL, cmucal_vclk_ip_ssmt_tnr_msa0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_ALN_STAT, NULL, cmucal_vclk_ip_ssmt_aln_stat, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_TNR_MSA0, NULL, cmucal_vclk_ip_qe_tnr_msa0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_ALN_STAT, NULL, cmucal_vclk_ip_qe_aln_stat, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_TNR_MSA1, NULL, cmucal_vclk_ip_ssmt_tnr_msa1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_TNR_MSA1, NULL, cmucal_vclk_ip_qe_tnr_msa1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ITP_CMU_ITP, NULL, cmucal_vclk_ip_itp_cmu_itp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_ITP, NULL, cmucal_vclk_ip_ad_apb_itp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_ITP, NULL, cmucal_vclk_ip_d_tzpc_itp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_ITP, NULL, cmucal_vclk_ip_gpc_itp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ITP, NULL, cmucal_vclk_ip_itp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_ITP, NULL, cmucal_vclk_ip_lhm_axi_p_itp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_ITP, NULL, cmucal_vclk_ip_sysreg_itp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_CTL_DNSITP, NULL, cmucal_vclk_ip_lhm_ast_ctl_dnsitp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF0_DNSITP, NULL, cmucal_vclk_ip_lhm_ast_otf0_dnsitp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF1_DNSITP, NULL, cmucal_vclk_ip_lhm_ast_otf1_dnsitp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF2_DNSITP, NULL, cmucal_vclk_ip_lhm_ast_otf2_dnsitp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_CTL_ITPDNS, NULL, cmucal_vclk_ip_lhs_ast_ctl_itpdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF_ITPDNS, NULL, cmucal_vclk_ip_lhs_ast_otf_itpdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_MCSC, NULL, cmucal_vclk_ip_lhm_axi_p_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D0_MCSC, NULL, cmucal_vclk_ip_lhs_axi_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_MCSC, NULL, cmucal_vclk_ip_sysreg_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MCSC_CMU_MCSC, NULL, cmucal_vclk_ip_mcsc_cmu_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF0_DNSMCSC, NULL, cmucal_vclk_ip_lhm_ast_otf0_dnsmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_MCSC, NULL, cmucal_vclk_ip_d_tzpc_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF_MCSCTNR, NULL, cmucal_vclk_ip_lhs_ast_otf_mcsctnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF1_DNSMCSC, NULL, cmucal_vclk_ip_lhm_ast_otf1_dnsmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_MCSC, NULL, cmucal_vclk_ip_gpc_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ITSC, NULL, cmucal_vclk_ip_itsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_MCSC, NULL, cmucal_vclk_ip_ssmt_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D0_MCSC, NULL, cmucal_vclk_ip_sysmmu_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_MCSC, NULL, cmucal_vclk_ip_ppmu_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_ITSC, NULL, cmucal_vclk_ip_ssmt_d0_itsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_ITSC, NULL, cmucal_vclk_ip_ppmu_d1_itsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_ITSC, NULL, cmucal_vclk_ip_ppmu_d0_itsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_VO_GDCMCSC, NULL, cmucal_vclk_ip_lhm_ast_vo_gdcmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_MCSCDNS, NULL, cmucal_vclk_ip_lhs_axi_d_mcscdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_ITSC, NULL, cmucal_vclk_ip_ad_apb_itsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_MCSC, NULL, cmucal_vclk_ip_ad_apb_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MCSC, NULL, cmucal_vclk_ip_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D1_MCSC, NULL, cmucal_vclk_ip_lhs_axi_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D1_MCSC, NULL, cmucal_vclk_ip_sysmmu_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF2_DNSMCSC, NULL, cmucal_vclk_ip_lhm_ast_otf2_dnsmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_INT_ITSCMCSC, NULL, cmucal_vclk_ip_lhm_ast_int_itscmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF_TNRMCSC, NULL, cmucal_vclk_ip_lhm_ast_otf_tnrmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_INT_ITSCMCSC, NULL, cmucal_vclk_ip_lhs_ast_int_itscmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_VO_MCSCCSIS, NULL, cmucal_vclk_ip_lhs_ast_vo_mcsccsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_ITSC, NULL, cmucal_vclk_ip_ssmt_d1_itsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_MCSC, NULL, cmucal_vclk_ip_ppmu_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_MCSC, NULL, cmucal_vclk_ip_ssmt_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D1_ITSC, NULL, cmucal_vclk_ip_qe_d1_itsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D2_ITSC, NULL, cmucal_vclk_ip_qe_d2_itsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D0_MCSC, NULL, cmucal_vclk_ip_qe_d0_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D1_MCSC, NULL, cmucal_vclk_ip_qe_d1_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D2_MCSC, NULL, cmucal_vclk_ip_qe_d2_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D3_MCSC, NULL, cmucal_vclk_ip_qe_d3_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D2_MCSC, NULL, cmucal_vclk_ip_sysmmu_d2_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D2_MCSC, NULL, cmucal_vclk_ip_lhs_axi_d2_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_D4_MCSC, NULL, cmucal_vclk_ip_qe_d4_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_C2R_MCSC, NULL, cmucal_vclk_ip_c2r_mcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MFC_CMU_MFC, NULL, cmucal_vclk_ip_mfc_cmu_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_MFC, NULL, cmucal_vclk_ip_as_apb_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_MFC, NULL, cmucal_vclk_ip_sysreg_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D0_MFC, NULL, cmucal_vclk_ip_lhs_axi_d0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D1_MFC, NULL, cmucal_vclk_ip_lhs_axi_d1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_MFC, NULL, cmucal_vclk_ip_lhm_axi_p_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D0_MFC, NULL, cmucal_vclk_ip_sysmmu_d0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D1_MFC, NULL, cmucal_vclk_ip_sysmmu_d1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_MFC, NULL, cmucal_vclk_ip_ppmu_d0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_MFC, NULL, cmucal_vclk_ip_ppmu_d1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D0_MFC, NULL, cmucal_vclk_ip_ssmt_d0_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MFC, NULL, cmucal_vclk_ip_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_MFC, NULL, cmucal_vclk_ip_d_tzpc_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D1_MFC, NULL, cmucal_vclk_ip_ssmt_d1_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_MFC, NULL, cmucal_vclk_ip_gpc_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MIF_CMU_MIF, NULL, cmucal_vclk_ip_mif_cmu_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DDRPHY, NULL, cmucal_vclk_ip_ddrphy, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_MIF, NULL, cmucal_vclk_ip_sysreg_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_MIF, NULL, cmucal_vclk_ip_lhm_axi_p_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AXI2APB_MIF, NULL, cmucal_vclk_ip_axi2apb_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBBR_DDRPHY, NULL, cmucal_vclk_ip_apbbr_ddrphy, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APBBR_DMC, NULL, cmucal_vclk_ip_apbbr_dmc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DMC, NULL, cmucal_vclk_ip_dmc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QCH_ADAPTER_PPC_DEBUG, NULL, cmucal_vclk_ip_qch_adapter_ppc_debug, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_MIF, NULL, cmucal_vclk_ip_gpc_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_MIF, NULL, cmucal_vclk_ip_d_tzpc_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPC_DEBUG, NULL, cmucal_vclk_ip_ppc_debug, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GEN_WREN_SECURE, NULL, cmucal_vclk_ip_gen_wren_secure, NULL, NULL),
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
	CMUCAL_VCLK(VCLK_IP_OTP_CON_BISR, NULL, cmucal_vclk_ip_otp_con_bisr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_DIT, NULL, cmucal_vclk_ip_dit, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_MISC, NULL, cmucal_vclk_ip_lhm_axi_p_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ACEL_D_MISC, NULL, cmucal_vclk_ip_lhs_acel_d_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PDMA, NULL, cmucal_vclk_ip_pdma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_MISC, NULL, cmucal_vclk_ip_ppmu_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_DIT, NULL, cmucal_vclk_ip_qe_dit, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PDMA, NULL, cmucal_vclk_ip_qe_pdma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_MISC_CMU_MISC, NULL, cmucal_vclk_ip_misc_cmu_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_RTIC, NULL, cmucal_vclk_ip_qe_rtic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_SPDMA, NULL, cmucal_vclk_ip_qe_spdma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_SSS, NULL, cmucal_vclk_ip_qe_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_RTIC, NULL, cmucal_vclk_ip_rtic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SPDMA, NULL, cmucal_vclk_ip_spdma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSS, NULL, cmucal_vclk_ip_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_SSS, NULL, cmucal_vclk_ip_ssmt_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_MISC, NULL, cmucal_vclk_ip_gpc_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_DIT, NULL, cmucal_vclk_ip_ad_apb_dit, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_DMA, NULL, cmucal_vclk_ip_ppmu_dma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PPMU_DMA, NULL, cmucal_vclk_ip_qe_ppmu_dma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ADM_AHB_SSS, NULL, cmucal_vclk_ip_adm_ahb_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_PUF, NULL, cmucal_vclk_ip_ad_apb_puf, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_ICC_CPUGIC, NULL, cmucal_vclk_ip_lhm_ast_icc_cpugic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_D_SSS, NULL, cmucal_vclk_ip_lhm_axi_d_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_IRI_GICCPU, NULL, cmucal_vclk_ip_lhs_ast_iri_giccpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_SSS, NULL, cmucal_vclk_ip_lhs_axi_d_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PUF, NULL, cmucal_vclk_ip_puf, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_MISC, NULL, cmucal_vclk_ip_xiu_d_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_MISC, NULL, cmucal_vclk_ip_sysmmu_misc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_SSS, NULL, cmucal_vclk_ip_sysmmu_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_GIC, NULL, cmucal_vclk_ip_lhm_axi_p_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_RTIC, NULL, cmucal_vclk_ip_ssmt_rtic, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_SPDMA, NULL, cmucal_vclk_ip_ssmt_spdma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PDMA, NULL, cmucal_vclk_ip_ssmt_pdma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PPMU_DMA, NULL, cmucal_vclk_ip_ssmt_ppmu_dma, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_DIT, NULL, cmucal_vclk_ip_ssmt_dit, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PDP_CMU_PDP, NULL, cmucal_vclk_ip_pdp_cmu_pdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_PDP, NULL, cmucal_vclk_ip_d_tzpc_pdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF0_CSISPDP, NULL, cmucal_vclk_ip_lhm_ast_otf0_csispdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF1_CSISPDP, NULL, cmucal_vclk_ip_lhm_ast_otf1_csispdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF2_CSISPDP, NULL, cmucal_vclk_ip_lhm_ast_otf2_csispdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_PDP, NULL, cmucal_vclk_ip_lhm_axi_p_pdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_PDP, NULL, cmucal_vclk_ip_gpc_pdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PDP_TOP, NULL, cmucal_vclk_ip_pdp_top, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_PDP_STAT, NULL, cmucal_vclk_ip_ssmt_pdp_stat, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PDP_STAT0, NULL, cmucal_vclk_ip_qe_pdp_stat0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_C2_PDP, NULL, cmucal_vclk_ip_ad_apb_c2_pdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF0_PDPIPP, NULL, cmucal_vclk_ip_lhs_ast_otf0_pdpipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF1_PDPIPP, NULL, cmucal_vclk_ip_lhs_ast_otf1_pdpipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF2_PDPIPP, NULL, cmucal_vclk_ip_lhs_ast_otf2_pdpipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF0_PDPCSIS, NULL, cmucal_vclk_ip_lhs_ast_otf0_pdpcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF1_PDPCSIS, NULL, cmucal_vclk_ip_lhs_ast_otf1_pdpcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF2_PDPCSIS, NULL, cmucal_vclk_ip_lhs_ast_otf2_pdpcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF0_PDPG3AA, NULL, cmucal_vclk_ip_lhs_ast_otf0_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF1_PDPG3AA, NULL, cmucal_vclk_ip_lhs_ast_otf1_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF2_PDPG3AA, NULL, cmucal_vclk_ip_lhs_ast_otf2_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_YOTF0_PDPG3AA, NULL, cmucal_vclk_ip_lhs_ast_yotf0_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_YOTF1_PDPG3AA, NULL, cmucal_vclk_ip_lhs_ast_yotf1_pdpg3aa, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_VO_CSISPDP, NULL, cmucal_vclk_ip_lhm_ast_vo_csispdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_VO_PDPIPP, NULL, cmucal_vclk_ip_lhs_ast_vo_pdpipp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_PDPCSIS, NULL, cmucal_vclk_ip_lhs_axi_d_pdpcsis, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_PDP, NULL, cmucal_vclk_ip_sysreg_pdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_XIU_D_PDP, NULL, cmucal_vclk_ip_xiu_d_pdp, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PDP_STAT1, NULL, cmucal_vclk_ip_qe_pdp_stat1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PDP_AF0, NULL, cmucal_vclk_ip_qe_pdp_af0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_PDP_AF1, NULL, cmucal_vclk_ip_qe_pdp_af1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AD_APB_VRA, NULL, cmucal_vclk_ip_ad_apb_vra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_QE_VRA, NULL, cmucal_vclk_ip_qe_vra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_VRA, NULL, cmucal_vclk_ip_vra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_VRA, NULL, cmucal_vclk_ip_ssmt_vra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_PDPDNS, NULL, cmucal_vclk_ip_lhs_axi_d_pdpdns, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_VRA, NULL, cmucal_vclk_ip_ppmu_vra, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_PERIC0, NULL, cmucal_vclk_ip_gpio_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_PERIC0, NULL, cmucal_vclk_ip_sysreg_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PERIC0_CMU_PERIC0, NULL, cmucal_vclk_ip_peric0_cmu_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_PERIC0, NULL, cmucal_vclk_ip_lhm_axi_p_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PERIC0_TOP0, NULL, cmucal_vclk_ip_peric0_top0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_PERIC0, NULL, cmucal_vclk_ip_d_tzpc_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PERIC0_TOP1, NULL, cmucal_vclk_ip_peric0_top1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_PERIC0, NULL, cmucal_vclk_ip_gpc_peric0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPIO_PERIC1, NULL, cmucal_vclk_ip_gpio_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_PERIC1, NULL, cmucal_vclk_ip_sysreg_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PERIC1_CMU_PERIC1, NULL, cmucal_vclk_ip_peric1_cmu_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_PERIC1, NULL, cmucal_vclk_ip_lhm_axi_p_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_PERIC1, NULL, cmucal_vclk_ip_d_tzpc_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PERIC1_TOP0, NULL, cmucal_vclk_ip_peric1_top0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_PERIC1, NULL, cmucal_vclk_ip_gpc_peric1, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_S2D_CMU_S2D, NULL, cmucal_vclk_ip_s2d_cmu_s2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BIS_S2D, NULL, cmucal_vclk_ip_bis_s2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_G_SCAN2DRAM, NULL, cmucal_vclk_ip_lhm_axi_g_scan2dram, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_APB_ASYNC_SYSMMU_D0_S1_NS_TNR, NULL, cmucal_vclk_ip_apb_async_sysmmu_d0_s1_ns_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_TNR, NULL, cmucal_vclk_ip_d_tzpc_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_VO_DNSTNR, NULL, cmucal_vclk_ip_lhm_ast_vo_dnstnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_TNR, NULL, cmucal_vclk_ip_lhm_axi_p_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF_TNRMCSC, NULL, cmucal_vclk_ip_lhs_ast_otf_tnrmcsc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D0_TNR, NULL, cmucal_vclk_ip_lhs_axi_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D1_TNR, NULL, cmucal_vclk_ip_lhs_axi_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D0_TNR, NULL, cmucal_vclk_ip_ppmu_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D1_TNR, NULL, cmucal_vclk_ip_ppmu_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D0_TNR, NULL, cmucal_vclk_ip_sysmmu_d0_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D1_TNR, NULL, cmucal_vclk_ip_sysmmu_d1_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_TNR, NULL, cmucal_vclk_ip_sysreg_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TNR_CMU_TNR, NULL, cmucal_vclk_ip_tnr_cmu_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_VO_TNRGDC, NULL, cmucal_vclk_ip_lhs_ast_vo_tnrgdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TNR, NULL, cmucal_vclk_ip_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AST_OTF_MCSCTNR, NULL, cmucal_vclk_ip_lhm_ast_otf_mcsctnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D2_TNR, NULL, cmucal_vclk_ip_lhs_axi_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D3_TNR, NULL, cmucal_vclk_ip_lhs_axi_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D2_TNR, NULL, cmucal_vclk_ip_ppmu_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D3_TNR, NULL, cmucal_vclk_ip_ppmu_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D2_TNR, NULL, cmucal_vclk_ip_sysmmu_d2_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D3_TNR, NULL, cmucal_vclk_ip_sysmmu_d3_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D4_TNR, NULL, cmucal_vclk_ip_ppmu_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D5_TNR, NULL, cmucal_vclk_ip_ppmu_d5_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D6_TNR, NULL, cmucal_vclk_ip_ppmu_d6_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_D7_TNR, NULL, cmucal_vclk_ip_ppmu_d7_tnr, NULL, NULL),
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
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D4_TNR, NULL, cmucal_vclk_ip_lhs_axi_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_D4_TNR, NULL, cmucal_vclk_ip_sysmmu_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D4_TNR, NULL, cmucal_vclk_ip_ssmt_d4_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D5_TNR, NULL, cmucal_vclk_ip_ssmt_d5_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D6_TNR, NULL, cmucal_vclk_ip_ssmt_d6_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_D7_TNR, NULL, cmucal_vclk_ip_ssmt_d7_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_TNR, NULL, cmucal_vclk_ip_gpc_tnr, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AST_OTF_TNRGDC, NULL, cmucal_vclk_ip_lhs_ast_otf_tnrgdc, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TPU_CMU_TPU, NULL, cmucal_vclk_ip_tpu_cmu_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_AXI_P_TPU, NULL, cmucal_vclk_ip_lhm_axi_p_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_D_TZPC_TPU, NULL, cmucal_vclk_ip_d_tzpc_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_AXI_D_TPU, NULL, cmucal_vclk_ip_lhs_axi_d_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSREG_TPU, NULL, cmucal_vclk_ip_sysreg_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SYSMMU_TPU, NULL, cmucal_vclk_ip_sysmmu_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_PPMU_TPU, NULL, cmucal_vclk_ip_ppmu_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_SSMT_TPU, NULL, cmucal_vclk_ip_ssmt_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_GPC_TPU, NULL, cmucal_vclk_ip_gpc_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_AS_APB_SYSMMU_NS_TPU, NULL, cmucal_vclk_ip_as_apb_sysmmu_ns_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_TPU, NULL, cmucal_vclk_ip_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T0_TPUCPUCL0, NULL, cmucal_vclk_ip_lhs_atb_t0_tpucpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_T1_TPUCPUCL0, NULL, cmucal_vclk_ip_lhs_atb_t1_tpucpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ASYNC_APBM_TPU, NULL, cmucal_vclk_ip_async_apbm_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_ASYNC_APB_INT_TPU, NULL, cmucal_vclk_ip_async_apb_int_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_INT_T0_TPU, NULL, cmucal_vclk_ip_lhm_atb_int_t0_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHM_ATB_INT_T1_TPU, NULL, cmucal_vclk_ip_lhm_atb_int_t1_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_HPM_TPU, NULL, cmucal_vclk_ip_hpm_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BUSIF_HPMTPU, NULL, cmucal_vclk_ip_busif_hpmtpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_INT_T0_TPU, NULL, cmucal_vclk_ip_lhs_atb_int_t0_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_LHS_ATB_INT_T1_TPU, NULL, cmucal_vclk_ip_lhs_atb_int_t1_tpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BUSIF_DDDTPU, NULL, cmucal_vclk_ip_busif_dddtpu, NULL, NULL),
};
