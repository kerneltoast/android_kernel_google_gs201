// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 */

#include "../cmucal.h"
#include "cmucal-vclklut.h"


/* DVFS VCLK -> LUT Parameter List */
unsigned int vdd_int_nm_lut_params[] = {
	0, 0, 0, 0, 0, 0, 5, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 3, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 2, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 2133000, 0, 0, 0, 1, 0, 0, 0, 0, 3, 3, 3, 3, 3, 1, 2, 1, 0, 0, 0, 1, 3, 3, 1, 3, 3, 3, 0, 0, 0, 1, 1, 1,
};
unsigned int vdd_int_ud_lut_params[] = {
	0, 0, 0, 3, 0, 0, 7, 0, 5, 4, 0, 6, 4, 0, 4, 0, 0, 4, 0, 2, 0, 0, 1, 1, 0, 1, 3, 0, 4, 4, 2, 2, 4, 0, 4, 0, 3, 1, 0, 0, 0, 4, 0, 4, 0, 4, 0, 4, 0, 0, 0, 1, 4, 0, 1, 1, 4, 0, 5, 0, 4, 0, 2133000, 4, 0, 4, 3, 2, 3, 3, 4, 3, 3, 3, 3, 3, 1, 2, 1, 0, 0, 0, 1, 3, 3, 1, 3, 3, 3, 0, 0, 0, 1, 1, 1,
};
unsigned int vdd_int_sud_lut_params[] = {
	1, 1, 1, 2, 1, 1, 7, 1, 5, 4, 1, 6, 4, 1, 4, 1, 1, 4, 1, 2, 1, 2, 2, 1, 1, 1, 3, 1, 2, 2, 4, 2, 4, 1, 4, 1, 3, 2, 1, 2, 1, 4, 1, 4, 1, 4, 1, 4, 1, 1, 1, 3, 4, 1, 0, 3, 2, 1, 5, 1, 4, 1, 1420000, 4, 1, 6, 1, 0, 0, 0, 0, 3, 3, 3, 3, 3, 1, 2, 1, 0, 0, 0, 1, 3, 3, 1, 3, 3, 3, 0, 0, 0, 1, 1, 1,
};
unsigned int vdd_int_uud_lut_params[] = {
	3, 11, 4, 4, 3, 3, 9, 5, 5, 5, 3, 5, 5, 5, 5, 5, 5, 5, 3, 2, 3, 2, 6, 0, 3, 2, 5, 9, 2, 1, 4, 0, 5, 5, 5, 5, 3, 2, 3, 5, 3, 5, 5, 5, 5, 5, 5, 5, 5, 11, 19, 2, 5, 5, 0, 7, 1, 4, 0, 7, 5, 5, 711000, 0, 5, 4, 1, 0, 3, 3, 0, 0, 7, 7, 7, 7, 0, 0, 0, 1, 1, 1, 0, 0, 7, 3, 7, 7, 7, 2, 1, 2, 0, 0, 0,
};
unsigned int vdd_mif_od_lut_params[] = {
	6400000, 6400000, 1067000, 2, 2, 1,
};
unsigned int vdd_mif_nm_lut_params[] = {
	3732000, 3732000, 980000, 2, 2, 1,
};
unsigned int vdd_mif_ud_lut_params[] = {
	2688000, 2688000, 640000, 2, 2, 1,
};
unsigned int vdd_mif_sud_lut_params[] = {
	1422000, 1422000, 320000, 2, 2, 1,
};
unsigned int vdd_mif_uud_lut_params[] = {
	710000, 842000, 133000, 1, 1, 0,
};
unsigned int vdd_g3d_nm_lut_params[] = {
	850000, 1000000, 2,
};
unsigned int vdd_g3d_ud_lut_params[] = {
	700000, 750000, 1,
};
unsigned int vdd_g3d_sud_lut_params[] = {
	470000, 470000, 1,
};
unsigned int vdd_g3d_uud_lut_params[] = {
	150000, 150000, 1,
};
unsigned int vdd_cam_nm_lut_params[] = {
	1150000, 3,
};
unsigned int vdd_cam_ud_lut_params[] = {
	747000, 3,
};
unsigned int vdd_cam_sud_lut_params[] = {
	373000, 3,
};
unsigned int vdd_cam_uud_lut_params[] = {
	178000, 1,
};
unsigned int vdd_cpucl0_sod_lut_params[] = {
	2100000, 1, 0,
};
unsigned int vdd_cpucl0_od_lut_params[] = {
	1800000, 1, 0,
};
unsigned int vdd_cpucl0_nm_lut_params[] = {
	1400000, 0, 1,
};
unsigned int vdd_cpucl0_ud_lut_params[] = {
	930000, 0, 0,
};
unsigned int vdd_cpucl0_sud_lut_params[] = {
	580000, 0, 0,
};
unsigned int vdd_cpucl0_uud_lut_params[] = {
	300000, 0, 0,
};
unsigned int vdd_cpucl1_sod_lut_params[] = {
	2350000,
};
unsigned int vdd_cpucl1_od_lut_params[] = {
	2000000,
};
unsigned int vdd_cpucl1_nm_lut_params[] = {
	1495000,
};
unsigned int vdd_cpucl1_ud_lut_params[] = {
	1027000,
};
unsigned int vdd_cpucl1_sud_lut_params[] = {
	700000,
};
unsigned int vdd_cpucl1_uud_lut_params[] = {
	400000,
};
unsigned int vdd_tpu_nm_lut_params[] = {
	1067000, 2,
};
unsigned int vdd_tpu_ud_lut_params[] = {
	833000, 3,
};
unsigned int vdd_tpu_sud_lut_params[] = {
	622000, 2,
};
unsigned int vdd_tpu_uud_lut_params[] = {
	350000, 2,
};
unsigned int vdd_cpucl2_sod_lut_params[] = {
	2850000,
};
unsigned int vdd_cpucl2_od_lut_params[] = {
	2250000,
};
unsigned int vdd_cpucl2_nm_lut_params[] = {
	1825000,
};
unsigned int vdd_cpucl2_ud_lut_params[] = {
	1275000,
};
unsigned int vdd_cpucl2_sud_lut_params[] = {
	850000,
};
unsigned int vdd_cpucl2_uud_lut_params[] = {
	500000,
};

/* SPECIAL VCLK -> LUT Parameter List */
unsigned int mux_cmu_cmuref_ud_lut_params[] = {
	1, 0, 0,
};
unsigned int mux_cmu_cmuref_sud_lut_params[] = {
	1, 0, 0,
};
unsigned int mux_cmu_cmuref_uud_lut_params[] = {
	1, 0, 0,
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
unsigned int mux_nocl0_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_nocl1b_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_nocl2a_cmuref_uud_lut_params[] = {
	1,
};
unsigned int clkcmu_hsi0_dpgtc_uud_lut_params[] = {
	3, 1,
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
unsigned int div_clk_apm_i3c_pmic_nm_lut_params[] = {
	1,
};
unsigned int clk_aur_add_ch_clk_uud_lut_params[] = {
	11,
};
unsigned int clkcmu_hpm_uud_lut_params[] = {
	0, 3,
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
unsigned int clk_g3d_add_ch_clk_uud_lut_params[] = {
	11,
};
unsigned int div_clk_gsacore_spi_fps_nm_lut_params[] = {
	1,
};
unsigned int div_clk_gsacore_spi_gsc_nm_lut_params[] = {
	1,
};
unsigned int div_clk_gsacore_uart_nm_lut_params[] = {
	3,
};
unsigned int div_clk_slc_dclk_nm_lut_params[] = {
	1,
};
unsigned int div_clk_slc_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc_dclk_ud_lut_params[] = {
	1,
};
unsigned int div_clk_slc_dclk_sud_lut_params[] = {
	1,
};
unsigned int div_clk_slc_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_nm_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_ud_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_sud_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_nm_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_ud_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_sud_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_nm_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_ud_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_sud_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_peric0_usi6_usi_uud_lut_params[] = {
	0,
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
unsigned int div_clk_peric0_usi3_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric0_usi4_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric0_usi5_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric0_usi14_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric0_usi7_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric0_usi8_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric0_usi1_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric0_usi0_uart_uud_lut_params[] = {
	1,
};
unsigned int div_clk_peric0_usi2_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric1_usi11_usi_uud_lut_params[] = {
	0,
};
unsigned int mux_clkcmu_peric1_ip_uud_lut_params[] = {
	1,
};
unsigned int div_clk_peric1_i3c_uud_lut_params[] = {
	1,
};
unsigned int div_clk_peric1_usi12_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric1_usi0_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric1_usi9_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric1_usi10_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric1_usi13_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric1_usi15_usi_uud_lut_params[] = {
	0,
};
unsigned int div_clk_peric1_usi16_usi_uud_lut_params[] = {
	0,
};

/* COMMON VCLK -> LUT Parameter List */
unsigned int blk_cmu_uud_lut_params[] = {
	2133000, 1866000, 800000, 666000, 2400000, 1, 0, 0, 0, 2, 0, 1, 7, 7, 0, 1, 4, 0, 1, 1, 2, 1, 1, 2, 1, 1,
};
unsigned int blk_hsi0_nm_lut_params[] = {
	614400, 1, 0, 2, 31, 0,
};
unsigned int blk_s2d_nm_lut_params[] = {
	400000, 0,
};
unsigned int blk_apm_nm_lut_params[] = {
	1, 0, 1, 0,
};
unsigned int blk_cpucl0_sod_lut_params[] = {
	0, 1, 1, 1, 3, 7, 1, 0, 0, 0, 0,
};
unsigned int blk_cpucl0_od_lut_params[] = {
	0, 1, 1, 1, 3, 7, 1, 0, 0, 0, 0,
};
unsigned int blk_cpucl0_uud_lut_params[] = {
	0, 1, 1, 1, 3, 7, 1, 0, 0, 0, 0,
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
unsigned int blk_eh_uud_lut_params[] = {
	1, 0,
};
unsigned int blk_eh_ud_lut_params[] = {
	1, 0,
};
unsigned int blk_gsacore_nm_lut_params[] = {
	0, 3, 1, 0, 0,
};
unsigned int blk_gsactrl_nm_lut_params[] = {
	1, 1, 1, 1, 0,
};
unsigned int blk_nocl0_nm_lut_params[] = {
	0, 0, 0,
};
unsigned int blk_nocl0_od_lut_params[] = {
	0, 0, 0,
};
unsigned int blk_nocl0_ud_lut_params[] = {
	0, 0, 0,
};
unsigned int blk_nocl0_sud_lut_params[] = {
	0, 0, 0,
};
unsigned int blk_nocl0_uud_lut_params[] = {
	0, 0, 0,
};
unsigned int blk_nocl1b_uud_lut_params[] = {
	0, 1, 0, 0,
};
unsigned int blk_aoc_nm_lut_params[] = {
	0, 0,
};
unsigned int blk_aoc_ud_lut_params[] = {
	0, 0,
};
unsigned int blk_aoc_sud_lut_params[] = {
	0, 0,
};
unsigned int blk_aoc_uud_lut_params[] = {
	0, 0,
};
unsigned int blk_aur_uud_lut_params[] = {
	3, 0, 0,
};
unsigned int blk_bo_uud_lut_params[] = {
	1,
};
unsigned int blk_csis_uud_lut_params[] = {
	1,
};
unsigned int blk_disp_uud_lut_params[] = {
	1,
};
unsigned int blk_dns_uud_lut_params[] = {
	1,
};
unsigned int blk_dpu_uud_lut_params[] = {
	1,
};
unsigned int blk_g2d_uud_lut_params[] = {
	1,
};
unsigned int blk_g3aa_uud_lut_params[] = {
	1,
};
unsigned int blk_g3d_nm_lut_params[] = {
	2, 0, 0,
};
unsigned int blk_g3d_uud_lut_params[] = {
	2, 0, 0,
};
unsigned int blk_gdc_uud_lut_params[] = {
	1,
};
unsigned int blk_hsi1_uud_lut_params[] = {
	0,
};
unsigned int blk_hsi2_uud_lut_params[] = {
	0,
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
unsigned int blk_mif_uud_lut_params[] = {
	0, 0,
};
unsigned int blk_mif_nm_lut_params[] = {
	0, 0,
};
unsigned int blk_misc_uud_lut_params[] = {
	1, 1, 0, 0,
};
unsigned int blk_nocl1a_uud_lut_params[] = {
	2, 0, 0,
};
unsigned int blk_nocl2a_uud_lut_params[] = {
	1, 0, 0,
};
unsigned int blk_pdp_uud_lut_params[] = {
	1,
};
unsigned int blk_peric0_uud_lut_params[] = {
	1, 0,
};
unsigned int blk_peric1_uud_lut_params[] = {
	0,
};
unsigned int blk_tnr_uud_lut_params[] = {
	1,
};
unsigned int blk_tpu_uud_lut_params[] = {
	3, 0,
};
