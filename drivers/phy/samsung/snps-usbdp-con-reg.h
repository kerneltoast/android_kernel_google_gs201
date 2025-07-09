/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung EXYNOS SoC series USB PHY driver
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef _USB_USBPHY_CAL_SNPS_USBDP_CON_REG_H_
#define _USB_USBPHY_CAL_SNPS_USBDP_CON_REG_H_

#define SNPS_USBDPPHY_REG_PHY_RST_CTRL 0x0000
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned phy_reset : 1;
		/* bit[1] */
		unsigned phy_reset_ovrd_en : 1;
		/* bit[3:2] */
		unsigned RSVD3_2 : 2;
		/* bit[4] */
		unsigned pipe_lane0_reset_n : 1;
		/* bit[5] */
		unsigned pipe_lane0_reset_n_ovrd_en : 1;
		/* bit[31:6] */
		unsigned RSVD31_6 : 26;
	} b;
} SNPS_USBDPPHY_REG_PHY_RST_CTRL_o, *SNPS_USBDPPHY_REG_PHY_RST_CTRL_p;

#define SNPS_USBDPPHY_REG_PHY_CR_PARA_CON0 0x0004
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned phy0_cr_para_sel : 1;
		/* bit[3:1] */
		unsigned RSVD3_1 : 3;
		/* bit[4] */
		unsigned phy0_cr_para_ack : 1;
		/* bit[7:5] */
		unsigned RSVD7_5 : 3;
		/* bit[8] */
		unsigned phy0_cr_para_clk : 1;
		/* bit[15:9] */
		unsigned RSVD15_9 : 7;
		/* bit[31:16] */
		unsigned phy0_cr_para_addr : 16;
	} b;
} SNPS_USBDPPHY_REG_PHY_CR_PARA_CON0_o, *SNPS_USBDPPHY_REG_PHY_CR_PARA_CON0_p;

#define SNPS_USBDPPHY_REG_PHY_CR_PARA_CON1 0x0008
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned phy0_cr_para_rd_en : 1;
		/* bit[15:1] */
		unsigned RSVD15_1 : 15;
		/* bit[31:16] */
		unsigned phy0_cr_para_rd_data : 16;
	} b;
} SNPS_USBDPPHY_REG_PHY_CR_PARA_CON1_o, *SNPS_USBDPPHY_REG_PHY_CR_PARA_CON1_p;

#define SNPS_USBDPPHY_REG_PHY_CR_PARA_CON2 0x000c
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned phy0_cr_para_wr_en : 1;
		/* bit[15:1] */
		unsigned RSVD15_1 : 15;
		/* bit[31:16] */
		unsigned phy0_cr_para_wr_data : 16;
	} b;
} SNPS_USBDPPHY_REG_PHY_CR_PARA_CON2_o, *SNPS_USBDPPHY_REG_PHY_CR_PARA_CON2_p;

#define SNPS_USBDPPHY_REG_PHY_CONFIG0 0x0100
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned pg_mode_en : 1;
		/* bit[1] */
		unsigned phy0_ana_pwr_en : 1;
		/* bit[2] */
		unsigned phy0_ana_pwr_stable : 1;
		/* bit[7:3] */
		unsigned phy0_cmn_in_generic_bus : 5;
		/* bit[12:8] */
		unsigned phy0_cmn_out_generic_bus : 5;
		/* bit[13] */
		unsigned phy0_hdmimode_enable : 1;
		/* bit[14] */
		unsigned phy0_pcs_pwr_stable : 1;
		/* bit[15] */
		unsigned phy0_pma_pwr_stable : 1;
		/* bit[16] */
		unsigned RSVD16 : 1;
		/* bit[17] */
		unsigned phy0_ref_alt_clk_lp_sel : 1;
		/* bit[18] */
		unsigned phy0_ref_repeat_clk_en : 1;
		/* bit[19] */
		unsigned phy0_ref_use_pad : 1;
		/* bit[20] */
		unsigned phy0_ss_mplla_force_en : 1;
		/* bit[21] */
		unsigned phy0_ss_mplla_ssc_en : 1;
		/* bit[22] */
		unsigned phy0_sup_pre_hp : 1;
		/* bit[23] */
		unsigned phy_rtune_req : 1;
		/* bit[24] */
		unsigned phy_rtune_ack : 1;
		/* bit[25] */
		unsigned phy_ss_lane0_rx2tx_par_lb_en : 1;
		/* bit[26] */
		unsigned phy_ss_rx0_term_acdc : 1;
		/* bit[27] */
		unsigned phy_ss_tx0_vregdrv_byp : 1;
		/* bit[28] */
		unsigned phy_ss_tx0_bypass_eq_calc : 1;
		/* bit[31:29] */
		unsigned RSVD31_29 : 3;
	} b;
} SNPS_USBDPPHY_REG_PHY_CONFIG0_o, *SNPS_USBDPPHY_REG_PHY_CONFIG0_p;

#define SNPS_USBDPPHY_REG_PHY_CONFIG1 0x0104
typedef union {
	u32 data;
	struct {
		/* bit[3:0] */
		unsigned pipe_lane0_link_num : 4;
		/* bit[5:4] */
		unsigned pipe_lane0_phy_src_sel : 2;
		/* bit[6] */
		unsigned pipe_lane0_tx2rx_loopbk : 1;
		/* bit[7] */
		unsigned pipe_rx_cdr_legacy_en : 1;
		/* bit[8] */
		unsigned pipe_rx_recal_cont_en : 1;
		/* bit[14:9] */
		unsigned pipe_rx0_idle_los_cnt : 6;
		/* bit[23:15] */
		unsigned pipe_rx0_ebuff_location : 9;
		/* bit[24] */
		unsigned ext_pclk_req : 1;
		/* bit[27:25] */
		unsigned hdmi_mpllb_hdmi_div : 3;
		/* bit[28] */
		unsigned hdmi_mpllb_hdmi_pixel_clk : 1;
		/* bit[30:29] */
		unsigned hdmi_mpllb_hdmi_pixel_clk_div : 2;
		/* bit[31] */
		unsigned RSVD31 : 1;
	} b;
} SNPS_USBDPPHY_REG_PHY_CONFIG1_o, *SNPS_USBDPPHY_REG_PHY_CONFIG1_p;

#define SNPS_USBDPPHY_REG_PHY_CONFIG2 0x0108
typedef union {
	u32 data;
	struct {
		/* bit[15:0] */
		unsigned upcs_pipe_config : 16;
		/* bit[16] */
		unsigned upcs_pwr_stable : 1;
		/* bit[17] */
		unsigned upcs_pwr_en : 1;
		/* bit[18] */
		unsigned phy0_pcs_pwr_en : 1;
		/* bit[19] */
		unsigned phy0_pma_pwr_en : 1;
		/* bit[20] */
		unsigned phy0_test_flyover_en : 1;
		/* bit[21] */
		unsigned phy0_test_stop_clk_en : 1;
		/* bit[22] */
		unsigned phy0_test_tx_ref_clk_en : 1;
		/* bit[23] */
		unsigned phy_test_burnin : 1;
		/* bit[24] */
		unsigned phy_test_powerdown : 1;
		/* bit[25] */
		unsigned ss_lane0_active : 1;
		/* bit[26] */
		unsigned ss_lane1_active : 1;
		/* bit[31:27] */
		unsigned RSVD31_27 : 5;
	} b;
} SNPS_USBDPPHY_REG_PHY_CONFIG2_o, *SNPS_USBDPPHY_REG_PHY_CONFIG2_p;

#define SNPS_USBDPPHY_REG_PHY_CONFIG3 0x010c
typedef union {
	u32 data;
	struct {
		/* bit[5:0] */
		unsigned phy0_rext_ctrl : 6;
		/* bit[7:6] */
		unsigned RSVD7_6 : 2;
		/* bit[8] */
		unsigned phy0_rext_en : 1;
		/* bit[31:9] */
		unsigned RSVD31_9 : 23;
	} b;
} SNPS_USBDPPHY_REG_PHY_CONFIG3_o, *SNPS_USBDPPHY_REG_PHY_CONFIG3_p;

#define SNPS_USBDPPHY_REG_PHY_SRAM_CON 0x0110
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned phy0_sram_bypass : 1;
		/* bit[1] */
		unsigned phy0_sram_ext_ld_done : 1;
		/* bit[2] */
		unsigned phy0_sram_init_done : 1;
		/* bit[31:3] */
		unsigned RSVD31_3 : 29;
	} b;
} SNPS_USBDPPHY_REG_PHY_SRAM_CON_o, *SNPS_USBDPPHY_REG_PHY_SRAM_CON_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG0 0x0114
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned phy_ext_ctrl_sel : 1;
		/* bit[3:1] */
		unsigned RSVD3_1 : 3;
		/* bit[4] */
		unsigned phy_ext_bs_rx_bigswing : 1;
		/* bit[7:5] */
		unsigned phy_ext_bs_rx_level : 3;
		/* bit[8] */
		unsigned phy_ext_bs_tx_lowswing : 1;
		/* bit[14:9] */
		unsigned phy_ext_dco_finetune : 6;
		/* bit[16:15] */
		unsigned phy_ext_dco_range : 2;
		/* bit[31:17] */
		unsigned RSVD31_17 : 15;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG0_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG0_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG1 0x0118
typedef union {
	u32 data;
	struct {
		/* bit[6:0] */
		unsigned phy_ext_mplla_cp_int : 7;
		/* bit[7] */
		unsigned RSVD7 : 1;
		/* bit[14:8] */
		unsigned phy_ext_mplla_cp_int_gs : 7;
		/* bit[15] */
		unsigned RSVD15 : 1;
		/* bit[22:16] */
		unsigned phy_ext_mplla_cp_prop : 7;
		/* bit[23] */
		unsigned RSVD23 : 1;
		/* bit[30:24] */
		unsigned phy_ext_mplla_cp_prop_gs : 7;
		/* bit[31] */
		unsigned RSVD31 : 1;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG1_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG1_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG2 0x011c
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned phy_ext_mplla_div5_clk_en : 1;
		/* bit[1] */
		unsigned phy_ext_mplla_div_clk_en : 1;
		/* bit[3:2] */
		unsigned RSVD3_2 : 2;
		/* bit[11:4] */
		unsigned phy_ext_mplla_div_multiplier : 8;
		/* bit[31:12] */
		unsigned RSVD31_12 : 20;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG2_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG2_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG3 0x0120
typedef union {
	u32 data;
	struct {
		/* bit[15:0] */
		unsigned phy_ext_mplla_fracn_den : 16;
		/* bit[16] */
		unsigned phy_ext_mplla_fracn_en : 1;
		/* bit[31:17] */
		unsigned RSVD31_17 : 15;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG3_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG3_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG4 0x0124
typedef union {
	u32 data;
	struct {
		/* bit[15:0] */
		unsigned phy_ext_mplla_fracn_quot : 16;
		/* bit[31:16] */
		unsigned phy_ext_mplla_fracn_rem : 16;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG4_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG4_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG5 0x0128
typedef union {
	u32 data;
	struct {
		/* bit[1:0] */
		unsigned phy_ext_mplla_freq_vco : 2;
		/* bit[3:2] */
		unsigned RSVD3_2 : 2;
		/* bit[15:4] */
		unsigned phy_ext_mplla_multiplier : 12;
		/* bit[31:16] */
		unsigned RSVD31_16 : 16;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG5_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG5_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG6 0x012c
typedef union {
	u32 data;
	struct {
		/* bit[19:0] */
		unsigned phy_ext_mplla_ssc_peak : 20;
		/* bit[31:20] */
		unsigned RSVD31_20 : 12;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG6_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG6_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG7 0x0130
typedef union {
	u32 data;
	struct {
		/* bit[20:0] */
		unsigned phy_ext_mplla_ssc_stepsize : 21;
		/* bit[31:21] */
		unsigned RSVD31_21 : 11;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG7_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG7_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG8 0x0134
typedef union {
	u32 data;
	struct {
		/* bit[0] SSC profile control. */
		unsigned ss_mplla_ssc_up_spread : 1;
		/* bit[3:1] MPLLA Output Clock Divider.
		   mplla_tx_clk_div[2:0] - Divide Ratio:
				¦ 3'b000 - 1
				¦ 3'b001 - 2
				¦ 3'b010 - 4
				¦ 3'b011 - 8
				¦ 3'b100 - 16
				¦ 3'b101 - 32
				¦ 3'b110 - 32
				¦ 3'b111 - 32 */
		unsigned ss_mplla_tx_clk_div : 3;
		/* bit[5:4] V2I operating range. */
		unsigned ss_mplla_v2i : 2;
		/* bit[6] MPLLA word clock divide by 2 enable. */
		unsigned ss_mplla_word_div2_en : 1;
		/* bit[7] */
		unsigned RSVD7 : 1;
		/* bit[10:8] MPLLA reference clock divider control. */
		unsigned ss_ref_clk_mplla_div : 3;
		/* bit[11] */
		unsigned RSVD11 : 1;
		/* bit[14:12] Input reference clock frequency range.
		   Specifies the frequency range of the input reference clock */
		unsigned ref_range : 3;
		/* bit[19:15] */
		unsigned RSVD19_15 : 5;
		/* bit[21:20] RX adaptation enable.
			Enables the RX adaptation circuitry and applies the input receiver
			AFE(analog front end) equalization settings */
		unsigned ss_rx_adapt_afe_en_g1 : 2;
		/* bit[23:22] RX adaptation enable.
			Enables the RX adaptation circuitry and applies the input receiver
			AFE(analog front end) equalization settings */
		unsigned ss_rx_adapt_afe_en_g2 : 2;
		/* bit[25:24] Enables the RX adaptation and decision feedback equalization */
		unsigned ss_rx_adapt_dfe_en_g1 : 2;
		/* bit[27:26] Enables the RX adaptation and decision feedback equalization */
		unsigned ss_rx_adapt_dfe_en_g2 : 2;
		/* bit[29:28] Controls the frequency of the RX VCO to a lower-frequency operating band. */
		unsigned ss_rx_cdr_vco_lowfreq_g1 : 2;
		/* bit[31:30] Controls the frequency of the RX VCO to a lower-frequency operating band. */
		unsigned ss_rx_cdr_vco_lowfreq_g2 : 2;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG8_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG8_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG9 0x0138
typedef union {
	u32 data;
	struct {
		/* bit[7:0] Controls the gain of the AFE. */
		unsigned ss_rx_eq_afe_gain_g1 : 8;
		/* bit[15:8] Controls the gain of the AFE. */
		unsigned ss_rx_eq_afe_gain_g2 : 8;
		/* bit[21:16] RX equalization attenuation level. */
		unsigned ss_rx_eq_att_lvl_g1 : 6;
		/* bit[23:22] */
		unsigned RSVD23_22 : 2;
		/* bit[29:24] RX equalization attenuation level. */
		unsigned ss_rx_eq_att_lvl_g2 : 6;
		/* bit[31:30] */
		unsigned RSVD31_30 : 2;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG9_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG9_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG10 0x013c
typedef union {
	u32 data;
	struct {
		/* bit[9:0] Controls the CTLE boost level */
		unsigned ss_rx_eq_ctle_boost_g1 : 10;
		/* bit[11:10] */
		unsigned RSVD11_10 : 2;
		/* bit[21:12] Controls the CTLE boost level */
		unsigned ss_rx_eq_ctle_boost_g2 : 10;
		/* bit[31:22] */
		unsigned RSVD31_22 : 10;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG10_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG10_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG11 0x0140
typedef union {
	u32 data;
	struct {
		/* bit[7:0] Controls the value of the offset applied to the IQ calibration result. */
		unsigned phy_ext_rx_eq_delta_iq_g1 : 8;
		/* bit[15:8] Controls the value of the offset applied to the IQ calibration result. */
		unsigned phy_ext_rx_eq_delta_iq_g2 : 8;
		/* bit[31:16] */
		unsigned RSVD31_16 : 16;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG11_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG11_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG12 0x0144
typedef union {
	u32 data;
	struct {
		/* bit[15:0] Controls the value of DFE data Tap1. */
		unsigned ss_rx_eq_dfe_tap1_g1 : 16;
		/* bit[31:16] Controls the value of DFE data Tap1. */
		unsigned ss_rx_eq_dfe_tap1_g2 : 16;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG12_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG12_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG13 0x0148
typedef union {
	u32 data;
	struct {
		/* bit[1:0] This signal allows to bypass the duty cycle corrector AC coupling capacitor.*/
		unsigned ss_rx_dcc_byp_ac_cap : 2;
		/* bit[3:2] */
		unsigned RSVD3_2 : 2;
		/* bit[10:4] RX VCO calibration reference load value. */
		unsigned ss_rx_ref_ld_val_g1 : 7;
		/* bit[11] */
		unsigned RSVD11 : 1;
		/* bit[18:12] RX VCO calibration reference load value */
		unsigned ss_rx_ref_ld_val_g2 : 7;
		/* bit[31:19] */
		unsigned RSVD31_19 : 13;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG13_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG13_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG14 0x014c
typedef union {
	u32 data;
	struct {
		/* bit[0] High Frequency Signal Detection. */
		unsigned ss_rx_sigdet_hf_en : 1;
		/* bit[1] PCS_RAW sigdef_hf_out disable filtering */
		unsigned ss_rx_sigdet_hf_filt_dis : 1;
		/* bit[3:2] */
		unsigned RSVD3_2 : 2;
		/* bit[9:4] Threshold for signal detection in high speed path */
		unsigned ss_rx_sigdet_hf_threshold_g1 : 6;
		/* bit[11:10] Threshold for signal detection in high speed path */
		unsigned RSVD11_10 : 2;
		/* bit[17:12] */
		unsigned ss_rx_sigdet_hf_threshold_g2 : 6;
		/* bit[31:18] */
		unsigned RSVD31_18 : 14;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG14_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG14_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG15 0x0150
typedef union {
	u32 data;
	struct {
		/* bit[0] Low Frequency Signal Detection. */
		unsigned ss_rx_sigdet_lf_en : 1;
		/* bit[1] Low Frequency Filter Enable */
		unsigned ss_rx_sigdet_lf_filter_en : 1;
		/* bit[3:2] */
		unsigned RSVD3_2 : 2;
		/* bit[9:4] Threshold for signal detection in low speed path. */
		unsigned ss_rx_sigdet_lf_threshold_g1 : 6;
		/* bit[11:10] */
		unsigned RSVD11_10 : 2;
		/* bit[17:12] Threshold for signal detection in low speed path. */
		unsigned ss_rx_sigdet_lf_threshold_g2 : 6;
		/* bit[19:18] */
		unsigned RSVD19_18 : 2;
		/* bit[22:20] Set the desired termination value for the receiver */
		unsigned ss_rx_term_ctrl : 3;
		/* bit[31:23] */
		unsigned RSVD31_23 : 9;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG15_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG15_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG16 0x0154
typedef union {
	u32 data;
	struct {
		/* bit[12:0] SS RX VCO calibration load value */
		unsigned ss_rx_vco_ld_val_g1 : 13;
		/* bit[15:13] */
		unsigned RSVD15_13 : 3;
		/* bit[28:16] SS RX VCO calibration load value */
		unsigned ss_rx_vco_ld_val_g2 : 13;
		/* bit[31:29] */
		unsigned RSVD31_29 : 3;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG16_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG16_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG17 0x0158
typedef union {
	u32 data;
	struct {
		/* bit[2:0] RX biasing current control.*/
		unsigned rx_vref_ctrl : 3;
		/* bit[3] */
		unsigned RSVD3 : 1;
		/* bit[6:4] Controls VCO regulator reference. */
		unsigned sup_rx_vco_vref_sel : 3;
		/* bit[7] */
		unsigned RSVD7 : 1;
		/* bit[8] */
		unsigned ss_tx_ana_iboost_en : 1;
		/* bit[9] */
		unsigned dp_tx_ana_iboost_en : 1;
		/* bit[11:10] */
		unsigned RSVD11_10 : 2;
		/* bit[13:12] */
		unsigned ss_tx_ana_rboost_en : 4;
		/* bit[15:14] */
		unsigned dp_tx_ana_rboost_en : 4;
		/* bit[17] */
		unsigned ss_tx_dcc_byp_ac_cap : 1;
		/* bit[16] */
		unsigned dp_tx_dcc_byp_ac_cap : 1;
		/* bit[31:18] */
		unsigned RSVD31_18 : 14;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG17_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG17_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG18 0x015c
typedef union {
	u32 data;
	struct {
		/* bit[5:0] */
		unsigned ss_tx_eq_main_g1 : 6;
		/* bit[6:0] */
		unsigned dp_tx_eq_main_g1 : 6;
		/* bit[17:12] */
		unsigned ss_tx_eq_main_g2 : 6;
		/* bit[23:18] */
		unsigned dp_tx_eq_main_g2 : 6;
		/* bit[24] */
		unsigned ss_tx_eq_ovrd_g1 : 1;
		/* bit[25] */
		unsigned dp_tx_eq_ovrd_g1 : 1;
		/* bit[26] */
		unsigned ss_tx_eq_ovrd_g2 : 1;
		/* bit[27] */
		unsigned dp_tx_eq_ovrd_g2 : 1;
		/* bit[31:28] */
		unsigned RSVD31_28 : 4;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG18_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG18_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG19 0x0160
typedef union {
	u32 data;
	struct {
		/* bit[5:0] */
		unsigned ss_tx_eq_post_g1 : 6;
		/* bit[11:6] */
		unsigned dp_tx_eq_post_g1 : 6;
		/* bit[17:12] */
		unsigned ss_tx_eq_post_g2 : 6;
		/* bit[23:18] */
		unsigned dp_tx_eq_post_g2 : 6;
		/* bit[31:24] */
		unsigned RSVD31_24 : 8;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG19_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG19_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG20 0x0164
typedef union {
	u32 data;
	struct {
		/* bit[5:0] */
		unsigned ss_tx_eq_pre_g1 : 6;
		/* bit[11:6] */
		unsigned dp_tx_eq_pre_g1 : 6;
		/* bit[17:12] */
		unsigned ss_tx_eq_pre_g2 : 6;
		/* bit[23:18] */
		unsigned dp_tx_eq_pre_g2 : 6;
		/* bit[31:24] */
		unsigned RSVD31_24 : 8;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG20_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG20_p;

#define SNPS_USBDPPHY_REG_PHY_EXT_CONFIG21 0x0168
typedef union {
	u32 data;
	struct {
		/* bit[2:0] */
		unsigned ss_tx_term_ctrl : 3;
		/* bit[3] */
		unsigned RSVD3 : 1;
		/* bit[6:4] */
		unsigned ss_tx_vswing_lvl : 3;
		/* bit[31:7] */
		unsigned RSVD31_7 : 25;
	} b;
} SNPS_USBDPPHY_REG_PHY_EXT_CONFIG21_o, *SNPS_USBDPPHY_REG_PHY_EXT_CONFIG21_p;

#define SNPS_USBDPPHY_REG_TCA_CONFIG 0x016c
typedef union {
	u32 data;
	struct {
		/* bit[1:0] */
		unsigned RSVD1_0 : 2;
		/* bit[2] Flip Invert to ComboPHYType-C
		 * Allows SS[Tx/Rx]2 pairs on Type-C connector to be the primary connector.
		 * May be used to preferable choose SS[Tx/Rx]2 based on routing considerations
		 * over the PCB.*/
		unsigned typec_flip_invert : 1;
		/* bit[3] PoR as USB Device
		 * Changes the TCA PoR for a USB device. As a device TCA relies on
		 * VBUS valid interface instead of USB Controller interface
		 * to achieve controller and PHY synchronization to reach P3.
		 * System should drive this as 1'b1 during TCA PoR for a USB Device PoR.*/
		unsigned tca_usb_dev_por : 1;
		/* bit[4] Type-C Host Drive VBUS from TCA
		 * Indication to the USB Host Subsystem that the VBUS Valid Drive
		 * needs to be held till this signal is asserted.
		 * Prevents the SS+ Device RxDetect timeout and the Device falling
		 * back to USB 2.0 only mode by disallowing the VBUS enable until the
		 * Host PHY is ready to respond to the SS+ Device Receiver Detection
		 * request.*/
		unsigned tca_drv_host_vbus : 1;
		/* bit[5] Type-C IDDIG indication to USB Subsystem from TCA.(Output)
		 * Implements the DR_SWAP functionality as specified in [TYPEC].
		 * The TCPM may access TCA to manipulate this signal and do the
		 * OTG like role swap for a DRP.*/
		unsigned tca_iddig : 1;
		/* bit[6] */
		unsigned tca_powerpresent : 1;
		/* bit[7] */
		unsigned tca_vbusvalid : 1;
		/* bit[13:8] Type-C Miscellaneous control to USB Subsystem from TCA.
		 * Implements the generic control and override functionality to be used
		 * within the USB Subsystems migrating to Type-C.*/
		unsigned tca_misc_ctrl : 6;
		/* bit[31:14] */
		unsigned RSVD31_14 : 18;
	} b;
} SNPS_USBDPPHY_REG_TCA_CONFIG_o, *SNPS_USBDPPHY_REG_TCA_CONFIG_p;

#define SNPS_USBDPPHY_REG_PHY_POWER_DOWN 0x0170
typedef union {
	u32 data;
	struct {
		/* bit[0] (?) */
		unsigned power_down_mode_en : 1;
		/* bit[1] */
		unsigned ref_clk_en_pd : 1;
		/* bit[3:2] */
		unsigned RSVD3_2 : 2;
		/* bit[5:4] */
		unsigned tx0_pstate_pd : 2;
		/* bit[7:6] */
		unsigned tx1_pstate_pd : 2;
		/* bit[9:8] */
		unsigned tx2_pstate_pd : 2;
		/* bit[11:10] */
		unsigned tx3_pstate_pd : 2;
		/* bit[13:12] */
		unsigned rx1_pstate_pd : 2;
		/* bit[15:14] */
		unsigned rx2_pstate_pd : 2;
		/* bit[16] */
		unsigned tx0_reset_pd : 1;
		/* bit[17] */
		unsigned tx1_reset_pd : 1;
		/* bit[18] */
		unsigned tx2_reset_pd : 1;
		/* bit[19] */
		unsigned tx3_reset_pd : 1;
		/* bit[20] */
		unsigned rx1_reset_pd : 1;
		/* bit[21] */
		unsigned rx2_reset_pd : 1;
		/* bit[23:22] */
		unsigned RSVD23_22 : 2;
		/* bit[24] */
		unsigned tx0_disable_pd : 1;
		/* bit[25] */
		unsigned tx1_disable_pd : 1;
		/* bit[26] */
		unsigned tx2_disable_pd : 1;
		/* bit[27] */
		unsigned tx3_disable_pd : 1;
		/* bit[28] */
		unsigned rx1_disable_pd : 1;
		/* bit[29] */
		unsigned rx2_disable_pd : 1;
		/* bit[31:30] */
		unsigned RSVD31_30 : 2;
	} b;
} SNPS_USBDPPHY_REG_PHY_POWER_DOWN_o, *SNPS_USBDPPHY_REG_PHY_POWER_DOWN_p;

#define SNPS_USBDPPHY_REG_TCA_STATE_CTRL 0x0174
typedef union {
	u32 data;
	struct {
		/* bit[0] SuperSpeed+ RxDetect Disable/Enable Acknowledge.
		 * When asserted, acknowledge from SuperSpeed+ state machine that
		 * Receiver has been disabled and it has put the PHY in P3 state.
		 * When de-asserted, acknowledge from the controller that it has
		 * received Rx Detection Enable request and it has proceeded to
		 * enable USB SS+ operation.
		 * After de-assertion, the USB controller may optionally assert
		 * ss_pipe_resetn*/
		unsigned ss_rxdet_disable_ack : 1;
		/* bit[1] Select ss_rxdet_disable_ack signal from link or by register */
		unsigned ss_rxdet_disable_ack_ovrd_en : 1;
		/* bit[3:2] */
		unsigned RSVD3_2 : 2;
		/* bit[7:4] */
		unsigned pipe_lane0_powerdown : 4;
		/* bit[8] */
		unsigned pipe_lane0_powerdown_ovrd_en : 1;
		/* bit[15:9] */
		unsigned RSVD15_9 : 7;
		/* bit[16] */
		unsigned ss_rxdet_disable : 1;
		/* bit[17] */
		unsigned pipe_lane0_phystatus : 1;
		/* bit[31:18] */
		unsigned RSVD31_18 : 14;
	} b;
} SNPS_USBDPPHY_REG_TCA_STATE_CTRL_o, *SNPS_USBDPPHY_REG_TCA_STATE_CTRL_p;

#define SNPS_USBDPPHY_REG_PHY_LANE1_CTRL 0x0178
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned phy_lane1_power_present : 1;
		/* bit[1] */
		unsigned phy_ss_rx1_term_acdc : 1;
		/* bit[2] */
		unsigned phy_ss_tx1_bypass_eq_calc : 1;
		/* bit[3] */
		unsigned phy_ss_tx1_vregdrv_byp : 1;
		/* bit[7:4] */
		unsigned pipe_lane1_link_num : 4;
		/* bit[11:8] */
		unsigned pipe_lane1_powerdown : 4;
		/* bit[13:12] */
		unsigned pipe_lane1_rate : 2;
		/* bit[14] */
		unsigned pipe_tx1_elecidle : 1;
		/* bit[15] */
		unsigned RSVD15 : 1;
		/* bit[16] */
		unsigned ss_rxdet_disable_1 : 1;
		/* bit[17] */
		unsigned ss_rxdet_disable_ack_1 : 1;
		/* bit[31:18] */
		unsigned RSVD31_18 : 14;
	} b;
} SNPS_USBDPPHY_REG_PHY_LANE1_CTRL_o, *SNPS_USBDPPHY_REG_PHY_LANE1_CTRL_p;

#define SNPS_USBDPPHY_REG_DP_AUX_CONFIG0 0x0200
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned aux_dp_dn_swap : 1;
		/* bit[3:1] */
		unsigned RSVD3_1 : 3;
		/* bit[5:4] */
		unsigned aux_hys_tune : 2;
		/* bit[7:6] */
		unsigned RSVD7_6 : 2;
		/* bit[8] */
		unsigned aux_vod_tune : 1;
		/* bit[9] */
		unsigned aux_pwdnb : 1;
		/* bit[11:10] */
		unsigned RSVD11_10 : 2;
		/* bit[15:12] */
		unsigned aux_ctrl : 4;
		/* bit[31:16] */
		unsigned RSVD31_16 : 16;
	} b;
} SNPS_USBDPPHY_REG_DP_AUX_CONFIG0_o, *SNPS_USBDPPHY_REG_DP_AUX_CONFIG0_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG0 0x0204
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned dp_lane0_rx2tx_par_lb_en : 1;
		/* bit[1] */
		unsigned dp_lane1_rx2tx_par_lb_en : 1;
		/* bit[2] */
		unsigned dp_lane2_rx2tx_par_lb_en : 1;
		/* bit[3] */
		unsigned dp_lane3_rx2tx_par_lb_en : 1;
		/* bit[4] */
		unsigned dp_lane0_tx2rx_ser_lb_en : 1;
		/* bit[5] */
		unsigned dp_lane1_tx2rx_ser_lb_en : 1;
		/* bit[6] */
		unsigned dp_lane2_tx2rx_ser_lb_en : 1;
		/* bit[7] */
		unsigned dp_lane3_tx2rx_ser_lb_en : 1;
		/* bit[31:8] */
		unsigned RSVD31_8 : 24;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG0_o, *SNPS_USBDPPHY_REG_DP_CONFIG0_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG1 0x0208
typedef union {
	u32 data;
	struct {
		/* bit[6:0] */
		unsigned dp_mpllb_cp_int : 7;
		/* bit[7] */
		unsigned RSVD7 : 1;
		/* bit[14:8] */
		unsigned dp_mpllb_cp_int_gs : 7;
		/* bit[15] */
		unsigned RSVD15 : 1;
		/* bit[22:16] */
		unsigned dp_mpllb_cp_prop : 7;
		/* bit[23] */
		unsigned RSVD23 : 1;
		/* bit[30:24] */
		unsigned dp_mpllb_cp_prop_gs : 7;
		/* bit[31] */
		unsigned RSVD31 : 1;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG1_o, *SNPS_USBDPPHY_REG_DP_CONFIG1_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG2 0x020c
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned dp_mpllb_div5_clk_en : 1;
		/* bit[1] */
		unsigned dp_mpllb_div_clk_en : 1;
		/* bit[3:2] */
		unsigned RSVD3_2 : 2;
		/* bit[11:4] */
		unsigned dp_mpllb_div_multiplier : 8;
		/* bit[12] */
		unsigned dp_mpllb_force_en : 1;
		/* bit[13] */
		unsigned dp_mpllb_force_ack : 1;
		/* bit[14] */
		unsigned dp_mpllb_fracn_cfg_update_en : 1;
		/* bit[15] */
		unsigned dp_mpllb_fracn_en : 1;
		/* bit[31:16] */
		unsigned dp_mpllb_fracn_den : 16;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG2_o, *SNPS_USBDPPHY_REG_DP_CONFIG2_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG3 0x0210
typedef union {
	u32 data;
	struct {
		/* bit[15:0] */
		unsigned dp_mpllb_fracn_quot : 16;
		/* bit[31:16] */
		unsigned dp_mpllb_fracn_rem : 16;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG3_o, *SNPS_USBDPPHY_REG_DP_CONFIG3_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG4 0x0214
typedef union {
	u32 data;
	struct {
		/* bit[1:0] */
		unsigned dp_mpllb_freq_vco : 2;
		/* bit[2] */
		unsigned dp_mpllb_init_cal_disable : 1;
		/* bit[3] */
		unsigned RSVD3 : 1;
		/* bit[15:4] */
		unsigned dp_mpllb_multiplier : 12;
		/* bit[16] */
		unsigned dp_mpllb_pmix_en : 1;
		/* bit[17] */
		unsigned dp_mpllb_ssc_en : 1;
		/* bit[31:18] */
		unsigned RSVD31_18 : 14;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG4_o, *SNPS_USBDPPHY_REG_DP_CONFIG4_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG5 0x0218
typedef union {
	u32 data;
	struct {
		/* bit[19:0] */
		unsigned dp_mpllb_ssc_peak : 20;
		/* bit[31:20] */
		unsigned RSVD31_20 : 12;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG5_o, *SNPS_USBDPPHY_REG_DP_CONFIG5_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG6 0x021c
typedef union {
	u32 data;
	struct {
		/* bit[20:0] */
		unsigned dp_mpllb_ssc_stepsize : 21;
		/* bit[31:21] */
		unsigned RSVD31_21 : 11;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG6_o, *SNPS_USBDPPHY_REG_DP_CONFIG6_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG7 0x0220
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned dp_mpllb_ssc_up_spread : 1;
		/* bit[3:1] */
		unsigned dp_mpllb_tx_clk_div : 3;
		/* bit[5:4] */
		unsigned dp_mpllb_v2i : 2;
		/* bit[6] */
		unsigned dp_mpllb_word_div2_en : 1;
		/* bit[7] */
		unsigned dp_pg_reset : 1;
		/* bit[8] */
		unsigned dp_ref_clk_en : 1;
		/* bit[11:9] */
		unsigned dp_ref_clk_mpllb_div : 3;
		/* bit[12] */
		unsigned dp_ref_clk_req : 1;
		/* bit[31:13] */
		unsigned RSVD31_13 : 19;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG7_o, *SNPS_USBDPPHY_REG_DP_CONFIG7_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG8 0x0224
typedef union {
	u32 data;
	struct {
		/* bit[5:0] */
		unsigned dp_tx0_eq_main : 6;
		/* bit[7:6] */
		unsigned RSVD7_6 : 2;
		/* bit[13:8] */
		unsigned dp_tx1_eq_main : 6;
		/* bit[15:14] */
		unsigned RSVD15_14 : 2;
		/* bit[21:16] */
		unsigned dp_tx2_eq_main : 6;
		/* bit[23:22] */
		unsigned RSVD23_22 : 2;
		/* bit[29:24] */
		unsigned dp_tx3_eq_main : 6;
		/* bit[31:30] */
		unsigned RSVD31_30 : 2;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG8_o, *SNPS_USBDPPHY_REG_DP_CONFIG8_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG9 0x0228
typedef union {
	u32 data;
	struct {
		/* bit[5:0] */
		unsigned dp_tx0_eq_post : 6;
		/* bit[7:6] */
		unsigned RSVD7_6 : 2;
		/* bit[13:8] */
		unsigned dp_tx1_eq_post : 6;
		/* bit[15:14] */
		unsigned RSVD15_14 : 2;
		/* bit[21:16] */
		unsigned dp_tx2_eq_post : 6;
		/* bit[23:22] */
		unsigned RSVD23_22 : 2;
		/* bit[29:24] */
		unsigned dp_tx3_eq_post : 6;
		/* bit[31:30] */
		unsigned RSVD31_30 : 2;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG9_o, *SNPS_USBDPPHY_REG_DP_CONFIG9_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG10 0x022c
typedef union {
	u32 data;
	struct {
		/* bit[5:0] */
		unsigned dp_tx0_eq_pre : 6;
		/* bit[7:6] */
		unsigned RSVD7_6 : 2;
		/* bit[13:8] */
		unsigned dp_tx1_eq_pre : 6;
		/* bit[15:14] */
		unsigned RSVD15_14 : 2;
		/* bit[21:16] */
		unsigned dp_tx2_eq_pre : 6;
		/* bit[23:22] */
		unsigned RSVD23_22 : 2;
		/* bit[29:24] */
		unsigned dp_tx3_eq_pre : 6;
		/* bit[31:30] */
		unsigned RSVD31_30 : 2;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG10_o, *SNPS_USBDPPHY_REG_DP_CONFIG10_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG11 0x0230
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned dp_tx0_bypass_eq_calc : 1;
		/* bit[1] */
		unsigned dp_tx1_bypass_eq_calc : 1;
		/* bit[2] */
		unsigned dp_tx2_bypass_eq_calc : 1;
		/* bit[3] */
		unsigned dp_tx3_bypass_eq_calc : 1;
		/* bit[5:4] */
		unsigned dp_tx0_pstate : 2;
		/* bit[7:6] */
		unsigned dp_tx1_pstate : 2;
		/* bit[9:8] */
		unsigned dp_tx2_pstate : 2;
		/* bit[11:10] */
		unsigned dp_tx3_pstate : 2;
		/* bit[12] */
		unsigned dp_tx0_lpd : 1;
		/* bit[13] */
		unsigned dp_tx1_lpd : 1;
		/* bit[14] */
		unsigned dp_tx2_lpd : 1;
		/* bit[15] */
		unsigned dp_tx3_lpd : 1;
		/* bit[18:16] */
		unsigned dp_tx0_rate : 3;
		/* bit[19] */
		unsigned RSVD19 : 1;
		/* bit[22:20] */
		unsigned dp_tx1_rate : 3;
		/* bit[23] */
		unsigned RSVD23 : 1;
		/* bit[26:24] */
		unsigned dp_tx2_rate : 3;
		/* bit[27] */
		unsigned RSVD27 : 1;
		/* bit[30:28] */
		unsigned dp_tx3_rate : 3;
		/* bit[31] */
		unsigned RSVD31 : 1;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG11_o, *SNPS_USBDPPHY_REG_DP_CONFIG11_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG12 0x0234
typedef union {
	u32 data;
	struct {
		/* bit[1:0] */
		unsigned dp_tx0_width : 2;
		/* bit[3:2] */
		unsigned dp_tx1_width : 2;
		/* bit[5:4] */
		unsigned dp_tx2_width : 2;
		/* bit[7:6] */
		unsigned dp_tx3_width : 2;
		/* bit[8] */
		unsigned dp_tx0_mpll_en : 1;
		/* bit[9] */
		unsigned dp_tx1_mpll_en : 1;
		/* bit[10] */
		unsigned dp_tx2_mpll_en : 1;
		/* bit[11] */
		unsigned dp_tx3_mpll_en : 1;
		/* bit[12] */
		unsigned dp_tx0_req_status : 1;
		/* bit[13] */
		unsigned dp_tx1_req_status : 1;
		/* bit[14] */
		unsigned dp_tx2_req_status : 1;
		/* bit[15] Write 1 : Indicates */
		unsigned dp_tx3_req_status : 1;
		/* bit[16] */
		unsigned dp_tx0_ack : 1;
		/* bit[17] */
		unsigned dp_tx1_ack : 1;
		/* bit[18] */
		unsigned dp_tx2_ack : 1;
		/* bit[19] */
		unsigned dp_tx3_ack : 1;
		/* bit[31:20] */
		unsigned RSVD31_20 : 12;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG12_o, *SNPS_USBDPPHY_REG_DP_CONFIG12_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG13 0x0238
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned dp_tx0_disable : 1;
		/* bit[1] */
		unsigned dp_tx1_disable : 1;
		/* bit[2] */
		unsigned dp_tx2_disable : 1;
		/* bit[3] */
		unsigned dp_tx3_disable : 1;
		/* bit[4] */
		unsigned dp_tx0_reset : 1;
		/* bit[5] */
		unsigned dp_tx1_reset : 1;
		/* bit[6] */
		unsigned dp_tx2_reset : 1;
		/* bit[7] */
		unsigned dp_tx3_reset : 1;
		/* bit[8] */
		unsigned dp_tx0_detrx_req_status : 1;
		/* bit[9] */
		unsigned dp_tx1_detrx_req_status : 1;
		/* bit[10] */
		unsigned dp_tx2_detrx_req_status : 1;
		/* bit[11] */
		unsigned dp_tx3_detrx_req_status : 1;
		/* bit[12] */
		unsigned dp_tx0_detrx_result : 1;
		/* bit[13] */
		unsigned dp_tx1_detrx_result : 1;
		/* bit[14] */
		unsigned dp_tx2_detrx_result : 1;
		/* bit[15] */
		unsigned dp_tx3_detrx_result : 1;
		/* bit[16] */
		unsigned dp_tx0_flyover_data_m : 1;
		/* bit[17] */
		unsigned dp_tx1_flyover_data_m : 1;
		/* bit[18] */
		unsigned dp_tx2_flyover_data_m : 1;
		/* bit[19] */
		unsigned dp_tx3_flyover_data_m : 1;
		/* bit[20] */
		unsigned dp_tx0_flyover_data_p : 1;
		/* bit[21] */
		unsigned dp_tx1_flyover_data_p : 1;
		/* bit[22] */
		unsigned dp_tx2_flyover_data_p : 1;
		/* bit[23] */
		unsigned dp_tx3_flyover_data_p : 1;
		/* bit[24] */
		unsigned dp_tx0_hp_prot_en : 1;
		/* bit[25] */
		unsigned dp_tx1_hp_prot_en : 1;
		/* bit[26] */
		unsigned dp_tx2_hp_prot_en : 1;
		/* bit[27] */
		unsigned dp_tx3_hp_prot_en : 1;
		/* bit[31:28] */
		unsigned RSVD31_28 : 4;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG13_o, *SNPS_USBDPPHY_REG_DP_CONFIG13_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG14 0x023c
typedef union {
	u32 data;
	struct {
		/* bit[4:0] */
		unsigned dp_tx0_in_generic_bus : 5;
		/* bit[7:5] */
		unsigned RSVD7_5 : 3;
		/* bit[12:8] */
		unsigned dp_tx1_in_generic_bus : 5;
		/* bit[15:13] */
		unsigned RSVD15_13 : 3;
		/* bit[20:16] */
		unsigned dp_tx2_in_generic_bus : 5;
		/* bit[23:21] */
		unsigned RSVD23_21 : 3;
		/* bit[28:24] */
		unsigned dp_tx3_in_generic_bus : 5;
		/* bit[31:29] */
		unsigned RSVD31_29 : 3;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG14_o, *SNPS_USBDPPHY_REG_DP_CONFIG14_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG15 0x0240
typedef union {
	u32 data;
	struct {
		/* bit[4:0] */
		unsigned dp_tx0_out_generic_bus : 5;
		/* bit[7:5] */
		unsigned RSVD7_5 : 3;
		/* bit[12:8] */
		unsigned dp_tx1_out_generic_bus : 5;
		/* bit[15:13] */
		unsigned RSVD15_13 : 3;
		/* bit[20:16] */
		unsigned dp_tx2_out_generic_bus : 5;
		/* bit[23:21] */
		unsigned RSVD23_21 : 3;
		/* bit[28:24] */
		unsigned dp_tx3_out_generic_bus : 5;
		/* bit[31:29] */
		unsigned RSVD31_29 : 3;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG15_o, *SNPS_USBDPPHY_REG_DP_CONFIG15_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG16 0x0244
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned dp_tx0_invert : 1;
		/* bit[1] */
		unsigned dp_tx1_invert : 1;
		/* bit[2] */
		unsigned dp_tx2_invert : 1;
		/* bit[3] */
		unsigned dp_tx3_invert : 1;
		/* bit[6:4] */
		unsigned dp_tx0_term_ctrl : 3;
		/* bit[7] */
		unsigned RSVD7 : 1;
		/* bit[10:8] */
		unsigned dp_tx1_term_ctrl : 3;
		/* bit[11] */
		unsigned RSVD11 : 1;
		/* bit[14:12] */
		unsigned dp_tx2_term_ctrl : 3;
		/* bit[15] */
		unsigned RSVD15 : 1;
		/* bit[18:16] */
		unsigned dp_tx3_term_ctrl : 3;
		/* bit[19] */
		unsigned RSVD19 : 1;
		/* bit[20] */
		unsigned dp_tx0_vregdrv_byp : 1;
		/* bit[21] */
		unsigned dp_tx1_vregdrv_byp : 1;
		/* bit[22] */
		unsigned dp_tx2_vregdrv_byp : 1;
		/* bit[23] */
		unsigned dp_tx3_vregdrv_byp : 1;
		/* bit[24] */
		unsigned dp_tx0_clk_rdy : 1;
		/* bit[25] */
		unsigned dp_tx1_clk_rdy : 1;
		/* bit[26] */
		unsigned dp_tx2_clk_rdy : 1;
		/* bit[27] */
		unsigned dp_tx3_clk_rdy : 1;
		/* bit[28] */
		unsigned dp_tx0_data_en : 1;
		/* bit[29] */
		unsigned dp_tx1_data_en : 1;
		/* bit[30] */
		unsigned dp_tx2_data_en : 1;
		/* bit[31] */
		unsigned dp_tx3_data_en : 1;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG16_o, *SNPS_USBDPPHY_REG_DP_CONFIG16_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG17 0x0248
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned dp_tx0_dcc_byp_ac_cap : 1;
		/* bit[1] */
		unsigned dp_tx1_dcc_byp_ac_cap : 1;
		/* bit[2] */
		unsigned dp_tx2_dcc_byp_ac_cap : 1;
		/* bit[3] */
		unsigned dp_tx3_dcc_byp_ac_cap : 1;
		/* bit[4] */
		unsigned dp_tx0_iboost_en : 1;
		/* bit[5] */
		unsigned dp_tx1_iboost_en : 1;
		/* bit[6] */
		unsigned dp_tx2_iboost_en : 1;
		/* bit[7] */
		unsigned dp_tx3_iboost_en : 1;
		/* bit[9:8] */
		unsigned dp_tx0_rboost_en : 2;
		/* bit[11:10] */
		unsigned dp_tx1_rboost_en : 2;
		/* bit[13:12] */
		unsigned dp_tx2_rboost_en : 2;
		/* bit[15:14] */
		unsigned dp_tx3_rboost_en : 2;
		/* bit[31:16] */
		unsigned RSVD31_16 : 16;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG17_o, *SNPS_USBDPPHY_REG_DP_CONFIG17_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG18 0x024c
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned dp_mpllb_div_clk : 1;
		/* bit[1] */
		unsigned dp_mpllb_oword_clk : 1;
		/* bit[2] */
		unsigned dp_mpllb_qword_clk : 1;
		/* bit[31:3] */
		unsigned RSVD31_3 : 29;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG18_o, *SNPS_USBDPPHY_REG_DP_CONFIG18_p;

#define SNPS_USBDPPHY_REG_DP_CONFIG19 0x0250
typedef union {
	u32 data;
	struct {
		/* bit[0] */
		unsigned dpalt_disable : 1;
		/* bit[1] */
		unsigned dpalt_disable_ack : 1;
		/* bit[2] */
		unsigned dpalt_dp4 : 1;
		/* bit[3] */
		unsigned tca_dp4_por : 1;
		/* bit[31:4] */
		unsigned RSVD31_4 : 28;
	} b;
} SNPS_USBDPPHY_REG_DP_CONFIG19_o, *SNPS_USBDPPHY_REG_DP_CONFIG19_p;

#define USB31DRD_LINK_LCSR_TX_DEEMPH    (0xD060)
#define USB31DRD_LINK_LCSR_TX_DEEMPH_1  (0xD064)
#define USB31DRD_LINK_LCSR_TX_DEEMPH_2  (0xD068)
#define USB31DRD_LINK_LCSR_TX_DEEMPH_3  (0xD06C)

#endif
