#include "../cmucal.h"
#include "cmucal-vclklut.h"


/* DVFS VCLK -> LUT Parameter List */
unsigned int vdd_int_uud_lut_params[] = {
	7, 2, 4, 3, 2, 3, 9, 3, 5, 5, 1, 5, 5, 3, 5, 3, 3, 5, 3, 2, 3, 5, 3, 0, 3, 2, 5, 7, 0, 1, 3, 0, 5, 3, 5, 3, 2, 2, 3, 1, 3, 5, 3, 5, 3, 5, 3, 5, 3, 2, 1, 5, 3, 0, 7, 1, 4, 6, 1, 5, 3, 5, 4, 0, 2, 2, 3, 0, 6, 2, 1, 0, 3, 7, 7, 7, 7, 7, 5, 1, 1, 7, 1, 7, 7, 3, 7, 7, 7, 0, 1, 0, 0,
};
unsigned int vdd_int_ud_lut_params[] = {
	0, 0, 0, 3, 0, 0, 7, 0, 5, 4, 0, 0, 4, 0, 4, 0, 0, 4, 0, 2, 0, 0, 1, 1, 0, 1, 3, 0, 4, 4, 2, 2, 4, 0, 4, 0, 4, 1, 0, 0, 0, 4, 0, 4, 0, 4, 0, 4, 0, 0, 0, 4, 0, 1, 1, 4, 0, 4, 0, 4, 0, 2, 3, 2, 3, 3, 2, 2, 4, 1, 2, 1, 0, 3, 3, 3, 3, 3, 2, 0, 0, 0, 0, 3, 3, 1, 3, 3, 3, 1, 0, 1, 1,
};
unsigned int vdd_int_sud_lut_params[] = {
	1, 1, 1, 2, 1, 1, 7, 1, 5, 4, 1, 2, 4, 1, 4, 1, 1, 4, 1, 2, 1, 1, 2, 1, 1, 1, 3, 1, 2, 2, 6, 2, 4, 1, 4, 1, 3, 2, 1, 2, 1, 4, 1, 4, 1, 4, 1, 4, 1, 1, 1, 4, 1, 0, 3, 2, 1, 4, 1, 4, 1, 5, 1, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 3, 3, 3, 3, 3, 2, 0, 0, 0, 0, 3, 3, 1, 3, 3, 3, 1, 0, 1, 1,
};
unsigned int vdd_int_nm_lut_params[] = {
	0, 0, 0, 0, 0, 0, 5, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 2, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 3, 3, 3, 3, 3, 2, 0, 0, 0, 0, 3, 3, 1, 3, 3, 3, 1, 0, 1, 1,
};
unsigned int vdd_mif_od_lut_params[] = {
	1200000, 1, 6400000, 6400000, 1,
};
unsigned int vdd_mif_sud_lut_params[] = {
	1200000, 0, 1422000, 1422000, 1,
};
unsigned int vdd_mif_ud_lut_params[] = {
	1200000, 0, 2688000, 2843000, 1,
};
unsigned int vdd_mif_uud_lut_params[] = {
	1200000, 0, 842000, 842000, 0,
};
unsigned int vdd_mif_nm_lut_params[] = {
	950000, 0, 4266000, 4266000, 1,
};
unsigned int vdd_g3d_nm_lut_params[] = {
	850000, 1066000,
};
unsigned int vdd_g3d_ud_lut_params[] = {
	666000, 830000,
};
unsigned int vdd_g3d_sud_lut_params[] = {
	466000, 640000,
};
unsigned int vdd_g3d_uud_lut_params[] = {
	225000, 320000,
};
unsigned int vdd_cpucl0_sod_lut_params[] = {
	2100000, 0,
};
unsigned int vdd_cpucl0_od_lut_params[] = {
	1870000, 0,
};
unsigned int vdd_cpucl0_nm_lut_params[] = {
	1200000, 1,
};
unsigned int vdd_cpucl0_ud_lut_params[] = {
	1100000, 0,
};
unsigned int vdd_cpucl0_sud_lut_params[] = {
	600000, 0,
};
unsigned int vdd_cpucl0_uud_lut_params[] = {
	380000, 0,
};
unsigned int vdd_cpucl1_sod_lut_params[] = {
	2470000,
};
unsigned int vdd_cpucl1_od_lut_params[] = {
	2200000,
};
unsigned int vdd_cpucl1_nm_lut_params[] = {
	1920000,
};
unsigned int vdd_cpucl1_ud_lut_params[] = {
	1360000,
};
unsigned int vdd_cpucl1_sud_lut_params[] = {
	700000,
};
unsigned int vdd_cpucl1_uud_lut_params[] = {
	470000,
};
unsigned int vdd_tpu_od_lut_params[] = {
	1400000,
};
unsigned int vdd_tpu_ud_lut_params[] = {
	900000,
};
unsigned int vdd_tpu_uud_lut_params[] = {
	560000,
};
unsigned int vdd_cpucl2_sod_lut_params[] = {
	3000000,
};
unsigned int vdd_cpucl2_od_lut_params[] = {
	2400000,
};
unsigned int vdd_cpucl2_nm_lut_params[] = {
	2000000,
};
unsigned int vdd_cpucl2_ud_lut_params[] = {
	1500000,
};
unsigned int vdd_cpucl2_sud_lut_params[] = {
	1200000,
};
unsigned int vdd_cpucl2_uud_lut_params[] = {
	800000,
};

/* SPECIAL VCLK -> LUT Parameter List */
unsigned int mux_bus0_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_bus1_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_cmu_cmuref_nm_lut_params[] = {
	1, 0,
};
unsigned int mux_cmu_cmuref_ud_lut_params[] = {
	1, 0,
};
unsigned int mux_cmu_cmuref_sud_lut_params[] = {
	1, 0,
};
unsigned int mux_cmu_cmuref_uud_lut_params[] = {
	1, 0,
};
unsigned int mux_core_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_cpucl1_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_cpucl2_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_clk_hsi0_usb20_ref_nm_lut_params[] = {
	1,
};
unsigned int mux_clkcmu_hsi0_usbdpdbg_uud_lut_params[] = {
	1,
};
unsigned int mux_mif_cmuref_uud_lut_params[] = {
	1,
};
unsigned int clkcmu_hsi0_dpgtc_uud_lut_params[] = {
	3, 1,
};
unsigned int mux_clkcmu_hsi1_pcie_uud_lut_params[] = {
	1,
};
unsigned int mux_clkcmu_hsi2_pcie_uud_lut_params[] = {
	1,
};
unsigned int clkcmu_tpu_uart_uud_lut_params[] = {
	3, 1,
};
unsigned int div_clk_apm_usi0_usi_nm_lut_params[] = {
	0,
};
unsigned int div_clk_apm_usi0_uart_nm_lut_params[] = {
	1,
};
unsigned int div_clk_apm_usi1_uart_nm_lut_params[] = {
	1,
};
unsigned int mux_clkcmu_hpm_uud_lut_params[] = {
	3,
};
unsigned int mux_clkcmu_cis_clk0_uud_lut_params[] = {
	3,
};
unsigned int mux_clkcmu_cis_clk1_uud_lut_params[] = {
	3,
};
unsigned int mux_clkcmu_cis_clk2_uud_lut_params[] = {
	3,
};
unsigned int mux_clkcmu_cis_clk3_uud_lut_params[] = {
	3,
};
unsigned int mux_clkcmu_cis_clk4_uud_lut_params[] = {
	3,
};
unsigned int mux_clkcmu_cis_clk5_uud_lut_params[] = {
	3,
};
unsigned int mux_clkcmu_cis_clk6_uud_lut_params[] = {
	3,
};
unsigned int mux_clkcmu_cis_clk7_uud_lut_params[] = {
	3,
};
unsigned int div_clk_slc_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc_dclk_nm_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_nm_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_nm_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_nm_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl0_cmuref_sod_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl0_cmuref_od_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl0_cmuref_nm_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl0_cmuref_ud_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl0_cmuref_sud_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl0_cmuref_uud_lut_params[] = {
	1,
};
unsigned int div_clk_cluster0_periphclk_sod_lut_params[] = {
	1,
};
unsigned int div_clk_cluster0_periphclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_cluster0_periphclk_nm_lut_params[] = {
	1,
};
unsigned int div_clk_cluster0_periphclk_ud_lut_params[] = {
	1,
};
unsigned int div_clk_cluster0_periphclk_sud_lut_params[] = {
	1,
};
unsigned int div_clk_cluster0_periphclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl1_cmuref_sod_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl1_cmuref_od_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl1_cmuref_nm_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl1_cmuref_ud_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl1_cmuref_sud_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl1_cmuref_uud_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl2_cmuref_sod_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl2_cmuref_od_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl2_cmuref_nm_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl2_cmuref_ud_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl2_cmuref_sud_lut_params[] = {
	1,
};
unsigned int div_clk_cpucl2_cmuref_uud_lut_params[] = {
	1,
};
unsigned int div_clk_peric_400_lut_params[] = {
	0, 1,
};
unsigned int div_clk_peric_200_lut_params[] = {
	1, 1,
};
unsigned int div_clk_peric_133_lut_params[] = {
	2, 1,
};
unsigned int div_clk_peric_100_lut_params[] = {
	3, 1,
};
unsigned int div_clk_peric_66_lut_params[] = {
	5, 1,
};
unsigned int div_clk_peric_50_lut_params[] = {
	7, 1,
};
unsigned int div_clk_peric_40_lut_params[] = {
	9, 1,
};
unsigned int div_clk_peric_24_lut_params[] = {
	0, 0,
};
unsigned int div_clk_peric_12_lut_params[] = {
	1, 0,
};
unsigned int div_clk_peric_8_lut_params[] = {
	2, 0,
};
unsigned int div_clk_peric_6_lut_params[] = {
	3, 0,
};
unsigned int div_clk_peric_4_lut_params[] = {
	5, 0,
};
unsigned int mux_clkcmu_peric0_ip_uud_lut_params[] = {
	1,
};
unsigned int div_clk_peric1_usi11_usi_uud_lut_params[] = {
	0, 1, 0, 0, 0, 0, 0,
};
unsigned int mux_clkcmu_peric1_ip_uud_lut_params[] = {
	1,
};
unsigned int div_clk_top_hsi0_bus_266_params[] = {
	1, 0,
};
unsigned int div_clk_top_hsi0_bus_177_params[] = {
	2, 0,
};
unsigned int div_clk_top_hsi0_bus_106_params[] = {
	4, 0,
};
unsigned int div_clk_top_hsi0_bus_80_params[] = {
	4, 2,
};
unsigned int div_clk_top_hsi0_bus_66_params[] = {
	5, 2,
};


/* COMMON VCLK -> LUT Parameter List */
unsigned int blk_cmu_uud_lut_params[] = {
	2133000, 1866000, 800000, 666000, 2400000, 1, 1, 0, 0, 2, 0, 1, 7, 1, 1, 1, 1, 7, 1, 2, 2, 1, 0, 4,
};
unsigned int blk_hsi0_nm_lut_params[] = {
	19200, 1, 0, 2,
};
unsigned int blk_s2d_nm_lut_params[] = {
	400000,
};
unsigned int blk_apm_nm_lut_params[] = {
	1, 0, 1,
};
unsigned int blk_bus0_uud_lut_params[] = {
	0, 1,
};
unsigned int blk_core_od_lut_params[] = {
	0, 2,
};
unsigned int blk_core_uud_lut_params[] = {
	0, 2,
};
unsigned int blk_core_nm_lut_params[] = {
	0, 2,
};
unsigned int blk_cpucl0_sod_lut_params[] = {
	0, 0, 1, 1, 3, 7, 1,
};
unsigned int blk_cpucl0_od_lut_params[] = {
	0, 0, 1, 1, 3, 7, 1,
};
unsigned int blk_cpucl0_nm_lut_params[] = {
	0, 0, 1, 1, 3, 7, 1,
};
unsigned int blk_cpucl0_ud_lut_params[] = {
	0, 0, 1, 1, 3, 7, 1,
};
unsigned int blk_cpucl0_sud_lut_params[] = {
	0, 0, 1, 1, 3, 7, 1,
};
unsigned int blk_cpucl0_uud_lut_params[] = {
	0, 0, 1, 1, 3, 7, 1,
};
unsigned int blk_cpucl1_sod_lut_params[] = {
	0,
};
unsigned int blk_cpucl1_od_lut_params[] = {
	0,
};
unsigned int blk_cpucl1_nm_lut_params[] = {
	0,
};
unsigned int blk_cpucl1_ud_lut_params[] = {
	0,
};
unsigned int blk_cpucl1_sud_lut_params[] = {
	0,
};
unsigned int blk_cpucl1_uud_lut_params[] = {
	0,
};
unsigned int blk_cpucl2_sod_lut_params[] = {
	0,
};
unsigned int blk_cpucl2_od_lut_params[] = {
	0,
};
unsigned int blk_cpucl2_nm_lut_params[] = {
	0,
};
unsigned int blk_cpucl2_ud_lut_params[] = {
	0,
};
unsigned int blk_cpucl2_sud_lut_params[] = {
	0,
};
unsigned int blk_cpucl2_uud_lut_params[] = {
	0,
};
unsigned int blk_g3d_nm_lut_params[] = {
	2, 2, 0,
};
unsigned int blk_g3d_ud_lut_params[] = {
	2, 2, 0,
};
unsigned int blk_g3d_sud_lut_params[] = {
	2, 2, 0,
};
unsigned int blk_g3d_uud_lut_params[] = {
	2, 2, 0,
};
unsigned int blk_bo_uud_lut_params[] = {
	1,
};
unsigned int blk_bus1_uud_lut_params[] = {
	1,
};
unsigned int blk_csis_uud_lut_params[] = {
	1,
};
unsigned int blk_disp_uud_lut_params[] = {
	3,
};
unsigned int blk_dns_uud_lut_params[] = {
	1,
};
unsigned int blk_dpu_uud_lut_params[] = {
	3,
};
unsigned int blk_eh_uud_lut_params[] = {
	2,
};
unsigned int blk_eh_od_lut_params[] = {
	2,
};
unsigned int blk_g2d_uud_lut_params[] = {
	1,
};
unsigned int blk_g3aa_uud_lut_params[] = {
	1,
};
unsigned int blk_gdc_uud_lut_params[] = {
	1,
};
unsigned int blk_ipp_uud_lut_params[] = {
	1,
};
unsigned int blk_itp_uud_lut_params[] = {
	1,
};
unsigned int blk_mcsc_uud_lut_params[] = {
	1,
};
unsigned int blk_mfc_uud_lut_params[] = {
	3,
};
unsigned int blk_pdp_uud_lut_params[] = {
	1,
};
unsigned int blk_tnr_uud_lut_params[] = {
	1,
};
unsigned int blk_tpu_od_lut_params[] = {
	3, 3,
};
unsigned int blk_tpu_ud_lut_params[] = {
	3, 3,
};
