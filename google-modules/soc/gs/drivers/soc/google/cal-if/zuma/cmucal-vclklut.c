// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 */

#include "../cmucal.h"
#include "cmucal-vclklut.h"

/* DVFS VCLK -> LUT Parameter List */
unsigned int vdd_int_nm_lut_params[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 3, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 2133000, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 3, 2, 3, 3, 3, 3, 1, 2, 1, 0, 0, 0, 1, 3, 3, 1, 0, 0, 3, 3, 3, 0, 0, 0, 0, 2, 1, 1,
};
unsigned int vdd_int_ud_lut_params[] = {
	0, 0, 4, 0, 0, 0, 5, 3, 3, 0, 6, 4, 0, 0, 4, 0, 2, 0, 5, 3, 1, 0, 1, 3, 2, 4, 0, 4, 0, 1, 1, 0, 4, 0, 4, 0, 4, 0, 0, 0, 1, 4, 0, 1, 1, 0, 4, 0, 4, 0, 4, 0, 4, 0, 1, 2133000, 3, 0, 3, 0, 0, 0, 4, 2, 3, 4, 2, 4, 4, 4, 0, 3, 2, 3, 3, 3, 3, 1, 2, 1, 0, 0, 0, 1, 3, 3, 1, 0, 0, 3, 3, 3, 0, 0, 0, 0, 2, 1, 1,
};
unsigned int vdd_int_sud_lut_params[] = {
	1, 1, 2, 1, 1, 1, 5, 1, 1, 1, 6, 2, 1, 1, 2, 1, 2, 1, 3, 8, 1, 1, 1, 3, 4, 2, 1, 2, 1, 4, 4, 2, 2, 1, 2, 1, 2, 1, 1, 1, 3, 2, 1, 0, 3, 1, 4, 1, 4, 1, 6, 1, 2, 1, 4, 1716000, 1, 1, 1, 1, 0, 1, 2, 1, 0, 1, 0, 0, 1, 1, 0, 3, 2, 3, 3, 3, 3, 1, 2, 1, 0, 0, 0, 1, 3, 3, 1, 0, 0, 3, 3, 3, 0, 0, 0, 0, 2, 1, 1,
};
unsigned int vdd_int_uud_lut_params[] = {
	3, 3, 5, 3, 3, 5, 5, 1, 1, 3, 4, 1, 5, 5, 1, 3, 2, 2, 5, 7, 0, 3, 2, 5, 4, 1, 5, 1, 5, 3, 3, 5, 1, 5, 1, 5, 1, 5, 11, 19, 2, 1, 5, 0, 7, 3, 6, 1, 4, 3, 6, 3, 1, 5, 3, 842000, 1, 3, 1, 3, 0, 5, 1, 2, 3, 1, 0, 0, 1, 1, 11, 0, 3, 7, 7, 7, 7, 0, 0, 0, 1, 1, 1, 0, 0, 7, 3, 1, 1, 7, 7, 7, 2, 1, 1, 1, 1, 0, 0,
};
unsigned int vdd_mif_od_lut_params[] = {
	7500000, 7500000, 1160000, 2,
};
unsigned int vdd_mif_nm_lut_params[] = {
	4266000, 4266000, 1066000, 2,
};
unsigned int vdd_mif_ud_lut_params[] = {
	2984000, 2984000, 711000, 2,
};
unsigned int vdd_mif_sud_lut_params[] = {
	1716000, 1716000, 355000, 2,
};
unsigned int vdd_mif_uud_lut_params[] = {
	842000, 842000, 214000, 1,
};
unsigned int vdd_g3d_nm_lut_params[] = {
	900000, 900000, 2,
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
	1066000, 3, 3,
};
unsigned int vdd_cam_ud_lut_params[] = {
	711000, 3, 3,
};
unsigned int vdd_cam_sud_lut_params[] = {
	355000, 3, 3,
};
unsigned int vdd_cam_uud_lut_params[] = {
	108000, 1, 1,
};
unsigned int vdd_cpucl0_sod_lut_params[] = {
	2115000, 2250000, 1828000, 1,
};
unsigned int vdd_cpucl0_od_lut_params[] = {
	1818000, 1950000, 1010000, 0,
};
unsigned int vdd_cpucl0_nm_lut_params[] = {
	1470000, 1550000, 950000, 0,
};
unsigned int vdd_cpucl0_ud_lut_params[] = {
	975000, 1000000, 818000, 0,
};
unsigned int vdd_cpucl0_sud_lut_params[] = {
	610000, 650000, 551000, 0,
};
unsigned int vdd_cpucl0_uud_lut_params[] = {
	325000, 350000, 325000, 0,
};
unsigned int vdd_cpucl1_sod_lut_params[] = {
	2600000,
};
unsigned int vdd_cpucl1_od_lut_params[] = {
	2250000,
};
unsigned int vdd_cpucl1_nm_lut_params[] = {
	1700000,
};
unsigned int vdd_cpucl1_ud_lut_params[] = {
	1225000,
};
unsigned int vdd_cpucl1_sud_lut_params[] = {
	800000,
};
unsigned int vdd_cpucl1_uud_lut_params[] = {
	475000,
};
unsigned int vdd_tpu_nm_lut_params[] = {
	1120000, 2,
};
unsigned int vdd_tpu_ud_lut_params[] = {
	845000, 3,
};
unsigned int vdd_tpu_sud_lut_params[] = {
	625000, 2,
};
unsigned int vdd_tpu_uud_lut_params[] = {
	225000, 2,
};
unsigned int vdd_cpucl2_sod_lut_params[] = {
	3300000,
};
unsigned int vdd_cpucl2_od_lut_params[] = {
	2700000,
};
unsigned int vdd_cpucl2_nm_lut_params[] = {
	2150000,
};
unsigned int vdd_cpucl2_ud_lut_params[] = {
	1750000,
};
unsigned int vdd_cpucl2_sud_lut_params[] = {
	1125000,
};
unsigned int vdd_cpucl2_uud_lut_params[] = {
	725000,
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
unsigned int mux_cpucl0_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_cpucl1_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_cpucl2_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_clk_hsi0_usb20_ref_nm_lut_params[] = {
	2, 31, 650000, 2, 1, 1,
};
unsigned int clkcmu_hsi0_usb32drd_uud_lut_params[] = {
	9, 1,
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
unsigned int mux_nocl2aa_cmuref_uud_lut_params[] = {
	1,
};
unsigned int mux_nocl2ab_cmuref_uud_lut_params[] = {
	1,
};
unsigned int clkcmu_dpub_dsim_uud_lut_params[] = {
	6, 0,
};
unsigned int clkcmu_hsi0_dpgtc_uud_lut_params[] = {
	3, 1, 9, 1,
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
unsigned int div_clk_cpucl0_add_ch_clk_uud_lut_params[] = {
	11,
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
unsigned int div_clk_hsi0_usix_nm_lut_params[] = {
	0, 0,
};
unsigned int div_clk_hsi0_usix_200mhz_lut_params[] = {
	1, 0,
};
unsigned int div_clk_hsi0_usix_100mhz_lut_params[] = {
	3, 0,
};
unsigned int div_clk_hsi0_usix_80mhz_lut_params[] = {
	4, 0,
};
unsigned int div_clk_hsi0_usix_50mhz_lut_params[] = {
	7, 0,
};
unsigned int div_clk_hsi0_usix_40mhz_lut_params[] = {
	9, 0,
};
unsigned int div_clk_hsi0_usix_24mhz_lut_params[] = {
	0, 1,
};
unsigned int div_clk_hsi0_usix_12mhz_lut_params[] = {
	1, 1,
};
unsigned int div_clk_hsi0_usix_8mhz_lut_params[] = {
	2, 1,
};
unsigned int div_clk_hsi0_usix_4mhz_lut_params[] = {
	5, 1,
};
unsigned int clkcmu_hsi0_peri_uud_lut_params[] = {
	0,
};
unsigned int clkcmu_hsi0_peri_nm_lut_params[] = {
	0,
};
unsigned int div_clk_slc_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc1_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc2_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_od_lut_params[] = {
	1,
};
unsigned int div_clk_slc3_dclk_uud_lut_params[] = {
	1,
};
unsigned int div_clk_peric0_usi6_usi_uud_lut_params[] = {
	0,
};
unsigned int mux_clkcmu_peric0_ip_uud_lut_params[] = {
	1,
};
unsigned int div_clk_pericx_usix_usi_nm_lut_params[] = {
	0, 1,
};
unsigned int div_clk_pericx_usix_usi_200mhz_lut_params[] = {
	1, 1,
};
unsigned int div_clk_pericx_usix_usi_100mhz_lut_params[] = {
	3, 1,
};
unsigned int div_clk_pericx_usix_usi_50mhz_lut_params[] = {
	7, 1,
};
unsigned int div_clk_pericx_usix_usi_40mhz_lut_params[] = {
	9, 1,
};
unsigned int div_clk_pericx_usix_usi_24mhz_lut_params[] = {
	0, 0,
};
unsigned int div_clk_pericx_usix_usi_12mhz_lut_params[] = {
	1, 0,
};
unsigned int div_clk_pericx_usix_usi_8mhz_lut_params[] = {
	2, 0,
};
unsigned int div_clk_pericx_usix_usi_4mhz_lut_params[] = {
	5, 0,
};
unsigned int div_clk_peric0_usi0_uart_uud_lut_params[] = {
	1,
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
unsigned int clk_tpu_add_ch_clk_uud_lut_params[] = {
	11,
};

/* COMMON VCLK -> LUT Parameter List */
unsigned int blk_cmu_nm_lut_params[] = {
	2133000, 1866000, 800000, 667000, 2400000, 0, 0, 0, 1, 2, 0, 2, 2, 7, 7, 0, 1, 4, 0, 0,
};
unsigned int blk_cmu_ud_lut_params[] = {
	2133000, 1866000, 800000, 666000, 2400000, 0, 0, 0, 1, 2, 0, 2, 2, 7, 7, 0, 1, 4, 0, 0,
};
unsigned int blk_cmu_uud_lut_params[] = {
	2133000, 1865000, 800000, 666000, 2400000, 0, 0, 0, 1, 2, 0, 2, 2, 7, 7, 0, 1, 4, 0, 0,
};
unsigned int blk_gsactrl_nm_lut_params[] = {
	1200000, 1, 1, 1, 1, 1, 1, 0,
};
unsigned int blk_s2d_nm_lut_params[] = {
	400000, 1, 0,
};
unsigned int blk_apm_nm_lut_params[] = {
	1, 0, 1, 0, 0,
};
unsigned int blk_cpucl0_sod_lut_params[] = {
	0, 0, 0, 1, 3, 3, 2, 3, 7, 1, 0, 0, 0, 0, 2, 1, 1, 2,
};
unsigned int blk_cpucl0_od_lut_params[] = {
	0, 0, 0, 1, 3, 3, 2, 3, 7, 1, 0, 0, 0, 0, 2, 1, 1, 2,
};
unsigned int blk_cpucl0_nm_lut_params[] = {
	0, 0, 0, 1, 3, 3, 2, 3, 7, 1, 0, 0, 0, 0, 2, 1, 1, 2,
};
unsigned int blk_cpucl0_ud_lut_params[] = {
	0, 0, 0, 1, 3, 3, 2, 3, 7, 1, 0, 0, 0, 0, 2, 1, 1, 2,
};
unsigned int blk_cpucl0_uud_lut_params[] = {
	0, 0, 0, 1, 3, 3, 2, 3, 7, 1, 0, 0, 0, 0, 2, 1, 1, 2,
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
unsigned int blk_gsacore_nm_lut_params[] = {
	0, 3, 1, 0, 0, 2,
};
unsigned int blk_hsi0_nm_lut_params[] = {
	0, 0, 0, 1, 1,
};
unsigned int blk_hsi1_ud_lut_params[] = {
	0, 1, 0, 0,
};
unsigned int blk_hsi1_uud_lut_params[] = {
	0, 0, 0, 0,
};
unsigned int blk_nocl0_od_lut_params[] = {
	0, 0, 0,
};
unsigned int blk_nocl0_uud_lut_params[] = {
	0, 0, 0,
};
unsigned int blk_nocl1b_uud_lut_params[] = {
	0, 2, 0, 0,
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
unsigned int blk_bw_uud_lut_params[] = {
	1,
};
unsigned int blk_dpub_uud_lut_params[] = {
	1,
};
unsigned int blk_dpuf0_uud_lut_params[] = {
	1,
};
unsigned int blk_dpuf1_uud_lut_params[] = {
	1,
};
unsigned int blk_eh_uud_lut_params[] = {
	0,
};
unsigned int blk_eh_ud_lut_params[] = {
	0,
};
unsigned int blk_g2d_uud_lut_params[] = {
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
unsigned int blk_gse_uud_lut_params[] = {
	1,
};
unsigned int blk_hsi2_uud_lut_params[] = {
	0,
};
unsigned int blk_ispfe_uud_lut_params[] = {
	1,
};
unsigned int blk_mcsc_uud_lut_params[] = {
	1,
};
unsigned int blk_mif_uud_lut_params[] = {
	0,
};
unsigned int blk_misc_uud_lut_params[] = {
	1, 1, 0, 0,
};
unsigned int blk_nocl1a_uud_lut_params[] = {
	2, 0, 0,
};
unsigned int blk_nocl2aa_uud_lut_params[] = {
	2, 0, 0,
};
unsigned int blk_nocl2ab_uud_lut_params[] = {
	0, 0, 2,
};
unsigned int blk_peric0_uud_lut_params[] = {
	1, 0,
};
unsigned int blk_peric1_uud_lut_params[] = {
	0,
};
unsigned int blk_rgbp_uud_lut_params[] = {
	1,
};
unsigned int blk_tnr_uud_lut_params[] = {
	1,
};
unsigned int blk_tpu_uud_lut_params[] = {
	3, 0,
};
unsigned int blk_yuvp_uud_lut_params[] = {
	1,
};
