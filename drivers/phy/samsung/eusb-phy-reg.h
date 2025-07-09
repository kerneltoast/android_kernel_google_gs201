/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung EXYNOS SoC series USB PHY driver
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef _USB_USBPHY_CAL_EUSB_PHY_REG_H_
#define _USB_USBPHY_CAL_EUSB_PHY_REG_H_

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_IDCODE	 0x0000
typedef union {
	u16 data;
	struct {
		/* bit[15:0] IDCODE value */
		unsigned idcode:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_IDCODE_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_IDCODE_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_OVRD_IN_0	 0x0004
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for suspend_n */
		unsigned suspend_n_ovrd:1;
		/* bit[1] Enable of override value for suspend_n */
		unsigned suspend_n_ovrd_en:1;
		/* bit[2] Override value for sleep_n */
		unsigned sleep_n_ovrd:1;
		/* bit[3] Enable of override value for sleep_n */
		unsigned sleep_n_ovrd_en:1;
		/* bit[4] Override value for port_reset */
		unsigned port_reset_ovrd:1;
		/* bit[5] Enable of override value for port_reset */
		unsigned port_reset_ovrd_en:1;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_OVRD_IN_1	 0x0008
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for termselect */
		unsigned termselect_ovrd:1;
		/* bit[1] Enable of override value for termselect */
		unsigned termselect_ovrd_en:1;
		/* bit[3:2] Override value for xcvrselect */
		unsigned xcvrselect_ovrd:2;
		/* bit[4] Enable of override value for xcvrselect */
		unsigned xcvrselect_ovrd_en:1;
		/* bit[5] Override value for dppulldown_ovrd */
		unsigned dppulldown_ovrd:1;
		/* bit[6] Enable of override value for dppulldown_ovrd_en */
		unsigned dppulldown_ovrd_en:1;
		/* bit[7] Override value for dmpulldown_ovrd */
		unsigned dmpulldown_ovrd:1;
		/* bit[8] Enable of override value for dmpulldown_ovrd_en */
		unsigned dmpulldown_ovrd_en:1;
		/* bit[9] Override value for utmi_clk_force_en */
		unsigned utmi_clk_force_en_ovrd:1;
		/* bit[10] Enable of override value for utmi_clk_force_en */
		unsigned utmi_clk_force_en_ovrd_en:1;
		/* bit[11] Override value for vbus_valid_ext */
		unsigned vbus_valid_ext_ovrd:1;
		/* bit[12] Enable of override value for vbus_valid_ext */
		unsigned vbus_valid_ext_ovrd_en:1;
		/* bit[15:13] */
		unsigned RSVD15_13:3;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_OVRD_IN_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_OVRD_IN_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_OVRD_IN_0	 0x000c
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for txbitstuffen */
		unsigned txbitstuffen_ovrd:1;
		/* bit[1] Enable of override value for txbitstuffen */
		unsigned txbitstuffen_ovrd_en:1;
		/* bit[2] Override value for txvalid */
		unsigned txvalid_ovrd:1;
		/* bit[3] Enable of override value for txvalid */
		unsigned txvalid_ovrd_en:1;
		/* bit[11:4] Override value for txdata */
		unsigned txdata_ovrd:8;
		/* bit[12] Enable of override value for txdata */
		unsigned txdata_ovrd_en:1;
		/* bit[14:13] Override value for opmode */
		unsigned opmode_ovrd:2;
		/* bit[15] Enable of override value for opmode */
		unsigned opmode_ovrd_en:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_OVRD_OUT_0	 0x0010
typedef union {
	u16 data;
	struct {
		/* bit[1:0] Override value for linestate */
		unsigned linestate_ovrd:2;
		/* bit[2] Enable of override value for linestate */
		unsigned linestate_ovrd_en:1;
		/* bit[3] Override value for hostdisconnect */
		unsigned hostdisconnect_ovrd:1;
		/* bit[4] Enable of override value for hostdisconnect */
		unsigned hostdisconnect_ovrd_en:1;
		/* bit[5] Override value for ref_clk_req */
		unsigned ref_clk_req_ovrd:1;
		/* bit[6] Enable of override value for ref_clk_req */
		unsigned ref_clk_req_ovrd_en:1;
		/* bit[15:7] */
		unsigned RSVD15_7:9;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_OVRD_OUT_0	 0x0014
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for rxerror */
		unsigned rxerror_ovrd:1;
		/* bit[1] Enable of override value for rxerror */
		unsigned rxerror_ovrd_en:1;
		/* bit[2] Override value for rxactive */
		unsigned rxactive_ovrd:1;
		/* bit[3] Enable of override value for rxactive */
		unsigned rxactive_ovrd_en:1;
		/* bit[4] Override value for rxvalid */
		unsigned rxvalid_ovrd:1;
		/* bit[5] Enable of override value for rxvalid */
		unsigned rxvalid_ovrd_en:1;
		/* bit[6] Override value for txready */
		unsigned txready_ovrd:1;
		/* bit[7] Enable of override value for txready */
		unsigned txready_ovrd_en:1;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_DIG_OVRD_IN_0	 0x0018
typedef union {
	u16 data;
	struct {
		/* bit[2:0] Override value for ref_freq_sel */
		unsigned ref_freq_sel_ovrd:3;
		/* bit[3] Enable of override value for ref_freq_sel */
		unsigned ref_freq_sel_ovrd_en:1;
		/* bit[4] Override value for phy_cfg_rptr_mode */
		unsigned phy_cfg_rptr_mode_ovrd:1;
		/* bit[5] Enable of override value for phy_cfg_rptr_mode */
		unsigned phy_cfg_rptr_mode_ovrd_en:1;
		/* bit[6] Override value for phy_cfg_por_in_lx */
		unsigned phy_cfg_por_in_lx_ovrd:1;
		/* bit[7] Enable of override value for phy_cfg_por_in_lx */
		unsigned phy_cfg_por_in_lx_ovrd_en:1;
		/* bit[8] Override value for phy_enable */
		unsigned phy_enable_ovrd:1;
		/*
		 * bit[9] Enable of override value for phy_enable.
		 * This phy_enable feature is useful only if phy_enable pin is
		 * present at phy top through IPNAME_PHY_ENABLE_SUPPORT macro.
		 * User should consider this register override as redundant
		 * if the feature is not enabled.
		 */
		unsigned phy_enable_ovrd_en:1;
		/* bit[15:10] */
		unsigned RSVD15_10:6;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_DIG_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_DIG_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_0	 0x001c
typedef union {
	u16 data;
	struct {
		/* bit[2:0] Override value for phy_cfg_pll_phy_vco_cntrl */
		unsigned phy_cfg_pll_phy_vco_cntrl_ovrd:3;
		/* bit[3] Enable of override value for phy_cfg_pll_phy_vco_cntrl */
		unsigned phy_cfg_pll_phy_vco_cntrl_ovrd_en:1;
		/* bit[5:4] Override value for phy_cfg_pll_phy_gmp_cntrl */
		unsigned phy_cfg_pll_phy_gmp_cntrl_ovrd:2;
		/* bit[6] Enable of override value for phy_cfg_pll_phy_gmp_cntrl */
		unsigned phy_cfg_pll_phy_gmp_cntrl_ovrd_en:1;
		/* bit[13:7] Override value for phy_cfg_pll_phy_gmp_cntrl */
		unsigned phy_cfg_pll_phy_cpbias_cntrl_ovrd:7;
		/* bit[14] Enable of override value for phy_cfg_pll_phy_gmp_cntrl */
		unsigned phy_cfg_pll_phy_cpbias_cntrl_ovrd_en:1;
		/* bit[15] */
		unsigned RSVD15:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_1	 0x0020
typedef union {
	u16 data;
	struct {
		/* bit[5:0] Override value for */
		unsigned phy_cfg_pll_phy_int_cntrl_ovrd:6;
		/* bit[6] Enable of override value for */
		unsigned phy_cfg_pll_phy_int_cntrl_ovrd_en:1;
		/* bit[12:7] Override value for */
		unsigned phy_cfg_pll_phy_prop_cntrl_ovrd:6;
		/* bit[13] Enable of override value for */
		unsigned phy_cfg_pll_phy_prop_cntrl_ovrd_en:1;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_2	 0x0024
typedef union {
	u16 data;
	struct {
		/* bit[11:0] Override value for phy_cfg_pll_fb_div */
		unsigned phy_cfg_pll_fb_div_ovrd:12;
		/* bit[12] Enable of override value for phy_cfg_pll_fb_div */
		unsigned phy_cfg_pll_fb_div_ovrd_en:1;
		/* bit[15:13] */
		unsigned RSVD15_13:3;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_2_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_2_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_3	 0x0028
typedef union {
	u16 data;
	struct {
		/* bit[1:0] Override value for phy_cfg_pll_vref_tune */
		unsigned phy_cfg_pll_vref_tune_ovrd:2;
		/* bit[2] Enable of override value for phy_cfg_pll_vref_tune */
		unsigned phy_cfg_pll_vref_tune_ovrd_en:1;
		/* bit[6:3] Override value for phy_cfg_pll_ref_div */
		unsigned phy_cfg_pll_ref_div_ovrd:4;
		/* bit[7] Enable of override value for phy_cfg_pll_ref_div */
		unsigned phy_cfg_pll_ref_div_ovrd_en:1;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_3_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_OVRD_IN_3_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_OVRD_IN_0	 0x002c
typedef union {
	u16 data;
	struct {
		/* bit[1:0] Override value for phy_cfg_tx_rise_tune */
		unsigned phy_cfg_tx_rise_tune_ovrd:2;
		/* bit[2] Enable of override value for phy_cfg_tx_rise_tune */
		unsigned phy_cfg_tx_rise_tune_ovrd_en:1;
		/* bit[4:3] Override value for phy_cfg_tx_hs_xv_tune */
		unsigned phy_cfg_tx_hs_xv_tune_ovrd:2;
		/* bit[5] Enable of override value for phy_cfg_tx_hs_xv_tune */
		unsigned phy_cfg_tx_hs_xv_tune_ovrd_en:1;
		/* bit[8:6] Override value for phy_cfg_tx_hs_vref_tune */
		unsigned phy_cfg_tx_hs_vref_tune_ovrd:3;
		/* bit[9] Enable of override value for phy_cfg_tx_hs_vref_tune */
		unsigned phy_cfg_tx_hs_vref_tune_ovrd_en:1;
		/* bit[12:10] Override value for phy_cfg_tx_preemp_tune */
		unsigned phy_cfg_tx_preemp_tune_ovrd:3;
		/* bit[13] Enable of override value for phy_cfg_tx_preemp_tune */
		unsigned phy_cfg_tx_preemp_tune_ovrd_en:1;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_OVRD_IN_1	 0x0030
typedef union {
	u16 data;
	struct {
		/* bit[1:0] Override value for phy_cfg_tx_fsls_vref_tune */
		unsigned phy_cfg_tx_fsls_vref_tune_ovrd:2;
		/* bit[2] Enable of override value for phy_cfg_tx_fsls_vref_tune */
		unsigned phy_cfg_tx_fsls_vref_tune_ovrd_en:1;
		/* bit[4:3] Override value for phy_cfg_tx_res_tune */
		unsigned phy_cfg_tx_res_tune_ovrd:2;
		/* bit[5] Enable of override value for phy_cfg_tx_res_tune */
		unsigned phy_cfg_tx_res_tune_ovrd_en:1;
		/* bit[6] Override value for phy_cfg_tx_fsls_slew_rate_tune */
		unsigned phy_cfg_tx_fsls_slew_rate_tune_ovrd:1;
		/* bit[7] Enable of override value for phy_cfg_tx_fsls_slew_rate_tune */
		unsigned phy_cfg_tx_fsls_slew_rate_tune_ovrd_en:1;
		/* bit[8] Override value for phy_cfg_tx_fsls_vreg_bypass */
		unsigned phy_cfg_tx_fsls_vreg_bypass_ovrd:1;
		/* bit[9] Enable of override value for phy_cfg_tx_fsls_vreg_bypass */
		unsigned phy_cfg_tx_fsls_vreg_bypass_ovrd_en:1;
		/* bit[15:10] */
		unsigned RSVD15_10:6;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_OVRD_IN_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_OVRD_IN_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RX_OVRD_IN_0	 0x0034
typedef union {
	u16 data;
	struct {
		/* bit[2:0] Override value for phy_cfg_rx_hs_tune */
		unsigned phy_cfg_rx_hs_tune_ovrd:3;
		/* bit[3] Enable of override value for phy_cfg_rx_hs_tune */
		unsigned phy_cfg_rx_hs_tune_ovrd_en:1;
		/* bit[5:4] Override value for phy_cfg_rx_eq_ctle */
		unsigned phy_cfg_rx_eq_ctle_ovrd:2;
		/* bit[6] Enable of override value for phy_cfg_rx_eq_ctle */
		unsigned phy_cfg_rx_eq_ctle_ovrd_en:1;
		/* bit[7] Enable of override value for phy_cfg_rx_hs_term_en_ovrd_r */
		unsigned phy_cfg_rx_hs_term_en_ovrd_r:1;
		/* bit[8] Enable of override value for phy_cfg_rx_hs_term_en_ovrd_en */
		unsigned phy_cfg_rx_hs_term_en_ovrd_en:1;
		/* bit[15:9] */
		unsigned RSVD15_9:7;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RX_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RX_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_TEST_OVRD_IN_0	 0x0038
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for test_stop_clk_en */
		unsigned test_stop_clk_en_ovrd:1;
		/* bit[1] Enable of override value for test_stop_clk_en */
		unsigned test_stop_clk_en_ovrd_en:1;
		/* bit[2] Override value for test_loopback_en */
		unsigned test_loopback_en_ovrd:1;
		/* bit[3] Enable of override value for test_loopback_en */
		unsigned test_loopback_en_ovrd_en:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_TEST_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_TEST_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_RES_OVRD_0	 0x003c
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for res_req_in */
		unsigned res_req_in_ovrd:1;
		/* bit[1] Enable of override value for  res_req_in */
		unsigned res_req_in_ovrd_en:1;
		/* bit[2] Override value for res_ack_in */
		unsigned res_ack_in_ovrd:1;
		/* bit[3] Enable of override value for res_ack_in */
		unsigned res_ack_in_ovrd_en:1;
		/* bit[4] Override value for res_req_out */
		unsigned res_req_out_ovrd:1;
		/* bit[5] Enable of override value for  res_req_out */
		unsigned res_req_out_ovrd_en:1;
		/* bit[6] Override value for res_ack_out */
		unsigned res_ack_out_ovrd:1;
		/* bit[7] Enable of override value for res_ack_out */
		unsigned res_ack_out_ovrd_en:1;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_RES_OVRD_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_RES_OVRD_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RCAL_OVRD	 0x0040
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for phy_cfg_rcal_bypass */
		unsigned phy_cfg_rcal_bypass_ovrd:1;
		/* bit[1] Enable of override value for phy_cfg_rcal_bypass */
		unsigned phy_cfg_rcal_bypass_ovrd_en:1;
		/* bit[5:2] Override value for phy_cfg_rcal_code */
		unsigned phy_cfg_rcal_code_ovrd:4;
		/* bit[6] Enable of override value for phy_cfg_rcal_code */
		unsigned phy_cfg_rcal_code_ovrd_en:1;
		/* bit[10:7] Override value for phy_cfg_rcal_offset */
		unsigned phy_cfg_rcal_offset_ovrd:4;
		/* bit[11] Enable of override value for phy_cfg_rcal_offset */
		unsigned phy_cfg_rcal_offset_ovrd_en:1;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RCAL_OVRD_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RCAL_OVRD_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UART_AUTORESUME_OVRD	 0x0044
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for phy_autoresume_enable */
		unsigned phy_autoresume_enable_ovrd:1;
		/* bit[1] Enable of override value for phy_autoresume_enable */
		unsigned phy_autoresume_enable_ovrd_en:1;
		/* bit[2] Override value for phy_tx_dig_bypass_sel */
		unsigned phy_tx_dig_bypass_sel_ovrd:1;
		/* bit[3] Enable of override value for phy_tx_dig_bypass_sel */
		unsigned phy_tx_dig_bypass_sel_ovrd_en:1;
		/* bit[4] Override value for phy_tx_se_dp_en */
		unsigned phy_tx_se_dp_en_ovrd:1;
		/* bit[5] Enable of override value for phy_tx_se_dp_en */
		unsigned phy_tx_se_dp_en_ovrd_en:1;
		/* bit[6] Override value for phy_tx_se_dm_en */
		unsigned phy_tx_se_dm_en_ovrd:1;
		/* bit[7] Enable of override value for phy_tx_se_dm_en */
		unsigned phy_tx_se_dm_en_ovrd_en:1;
		/* bit[8] Override value for phy_tx_se_dp */
		unsigned phy_tx_se_dp_ovrd:1;
		/* bit[9] Enable of override value for phy_tx_se_dp */
		unsigned phy_tx_se_dp_ovrd_en:1;
		/* bit[10] Override value for phy_tx_se_dm */
		unsigned phy_tx_se_dm_ovrd:1;
		/* bit[11] Enable of override value for phy_tx_se_dm */
		unsigned phy_tx_se_dm_ovrd_en:1;
		/* bit[12] Override value for phy_rx_se_dp */
		unsigned phy_rx_se_dp_ovrd:1;
		/* bit[13] Enable of override value for phy_rx_se_dp */
		unsigned phy_rx_se_dp_ovrd_en:1;
		/* bit[14] Override value for phy_rx_se_dm */
		unsigned phy_rx_se_dm_ovrd:1;
		/* bit[15] Enable of override value for phy_rx_se_dm */
		unsigned phy_rx_se_dm_ovrd_en:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UART_AUTORESUME_OVRD_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UART_AUTORESUME_OVRD_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_ASIC_IN_0	 0x0080
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ASIC for suspend_n */
		unsigned suspend_n:1;
		/* bit[1] Value from ASIC for sleep_n */
		unsigned sleep_n:1;
		/* bit[2] Value from ASIC for utmi_port_reset */
		unsigned utmi_port_reset:1;
		/* bit[15:3] */
		unsigned RSVD15_3:13;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_ASIC_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_ASIC_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_ASIC_IN_1	 0x0084
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ASIC for termselect */
		unsigned termselect:1;
		/* bit[2:1] Value from ASIC for xcvrselect */
		unsigned xcvrselect:2;
		/* bit[3] Value from ASIC for dppulldown */
		unsigned dppulldown:1;
		/* bit[4] Value from ASIC for dmpulldown */
		unsigned dmpulldown:1;
		/* bit[5] Value from ASIC for utmi_clk_force_en */
		unsigned utmi_clk_force_en:1;
		/* bit[6] Value from ASIC for vbus_valid_ext */
		unsigned vbus_valid_ext:1;
		/* bit[15:7] */
		unsigned RSVD15_7:9;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_ASIC_IN_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_ASIC_IN_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_ASIC_IN_0	 0x0088
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ASIC for txbitstuffen */
		unsigned txbitstuffen:1;
		/* bit[1] Value from ASIC for txvalid */
		unsigned txvalid:1;
		/* bit[9:2] Value from ASIC for txdata */
		unsigned txdata:8;
		/* bit[11:10] Value from ASIC for opmode */
		unsigned opmode:2;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_ASIC_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_ASIC_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_ASIC_OUT_0	 0x008c
typedef union {
	u16 data;
	struct {
		/* bit[1:0] Value from ASIC for linestate */
		unsigned linestate:2;
		/* bit[2] Value from ASIC for hostdisconnect */
		unsigned hostdisconnect:1;
		/* bit[3] Value from ASIC for ref_clk_req */
		unsigned ref_clk_req:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_ASIC_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_CTRL_ASIC_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_ASIC_OUT_0	 0x0090
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ASIC for rxerror */
		unsigned rxerror:1;
		/* bit[1] Value from ASIC for rxactive */
		unsigned rxactive:1;
		/* bit[2] Value from ASIC for rxvalid */
		unsigned rxvalid:1;
		/* bit[3] Value from ASIC for txready */
		unsigned txready:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_ASIC_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UTMI_TXRX_ASIC_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_DIG_ASIC_IN_0	 0x0094
typedef union {
	u16 data;
	struct {
		/* bit[2:0] Value from ASIC for ref_freq_sel */
		unsigned ref_freq_sel:3;
		/* bit[3] Value from ASIC for phy_cfg_rptr_mode */
		unsigned phy_cfg_rptr_mode:1;
		/* bit[4] Value from ASIC for phy_cfg_por_in_lx */
		unsigned phy_cfg_por_in_lx:1;
		/* bit[5] Value from ASIC for phy_enable This phy_enable feature is useful only if
		 * phy_enable pin is present at phy top through IPNAME_PHY_ENABLE_SUPPORT macro.
		 * User should consider this register override as redundant if the feature is not
		 * enabled.
		 */
		unsigned phy_enable:1;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_DIG_ASIC_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_DIG_ASIC_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_0	 0x0098
typedef union {
	u16 data;
	struct {
		/* bit[2:0] Value from ASIC for phy_cfg_pll_phy_vco_cntrl */
		unsigned phy_cfg_pll_phy_vco_cntrl:3;
		/* bit[4:3] Value from ASIC for phy_cfg_pll_phy_gmp_cntrl */
		unsigned phy_cfg_pll_phy_gmp_cntrl:2;
		/* bit[11:5] Value from ASIC for phy_cfg_pll_phy_cpbias_cntrl */
		unsigned phy_cfg_pll_phy_cpbias_cntrl:7;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_1	 0x009c
typedef union {
	u16 data;
	struct {
		/* bit[5:0] Value from ASIC for phy_cfg_pll_phy_int_cntrl */
		unsigned phy_cfg_pll_phy_int_cntrl:6;
		/* bit[11:6] Value from ASIC for phy_cfg_pll_phy_prop_cntrl */
		unsigned phy_cfg_pll_phy_prop_cntrl:6;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_2	 0x00a0
typedef union {
	u16 data;
	struct {
		/* bit[11:0] Value from ASIC for phy_cfg_pll_fb_div */
		unsigned phy_cfg_pll_fb_div:12;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_2_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_2_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_3	 0x00a4
typedef union {
	u16 data;
	struct {
		/* bit[1:0] Value from ASIC for phy_cfg_pll_vref_tune */
		unsigned phy_cfg_pll_vref_tune:2;
		/* bit[5:2] Value from ASIC for phy_cfg_pll_ref_div */
		unsigned phy_cfg_pll_ref_div:4;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_3_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_PLL_ASIC_IN_3_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_ASIC_IN_0	 0x00a8
typedef union {
	u16 data;
	struct {
		/* bit[1:0] Value from ASIC for phy_cfg_tx_rise_tune */
		unsigned phy_cfg_tx_rise_tune:2;
		/* bit[3:2] Value from ASIC for phy_cfg_tx_hs_xv_tune */
		unsigned phy_cfg_tx_hs_xv_tune:2;
		/* bit[6:4] Value from ASIC for phy_cfg_tx_hs_vref_tune */
		unsigned phy_cfg_tx_hs_vref_tune:3;
		/* bit[9:7] Value from ASIC for phy_cfg_tx_preemp_tune */
		unsigned phy_cfg_tx_preemp_tune:3;
		/* bit[15:10] */
		unsigned RSVD15_10:6;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_ASIC_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_ASIC_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_ASIC_IN_1	 0x00ac
typedef union {
	u16 data;
	struct {
		/* bit[1:0] Value from ASIC for phy_cfg_tx_fsls_vref_tune */
		unsigned phy_cfg_tx_fsls_vref_tune:2;
		/* bit[3:2] Value from ASIC for phy_cfg_tx_res_tune */
		unsigned phy_cfg_tx_res_tune:2;
		/* bit[4] Value from ASIC for phy_cfg_tx_fsls_slew_rate_tune */
		unsigned phy_cfg_tx_fsls_slew_rate_tune:1;
		/* bit[5] Value from ASIC for phy_cfg_tx_fsls_vreg_bypass */
		unsigned phy_cfg_tx_fsls_vreg_bypass:1;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_ASIC_IN_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_TX_ASIC_IN_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RX_ASIC_IN_0	 0x00b0
typedef union {
	u16 data;
	struct {
		/* bit[2:0] Value from ASIC for phy_cfg_rx_hs_tune */
		unsigned phy_cfg_rx_hs_tune:3;
		/* bit[4:3] Value from ASIC for phy_cfg_rx_eq_ctle */
		unsigned phy_cfg_rx_eq_ctle:2;
		/* bit[5] Value from ASIC for phy_cfg_rx_hs_term_en */
		unsigned phy_cfg_rx_hs_term_en:1;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RX_ASIC_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RX_ASIC_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_TEST_ASIC_IN_0	 0x00b4
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ASIC for test_stop_clk_en */
		unsigned test_stop_clk_en:1;
		/* bit[1] Value from ASIC for test_loopback_en */
		unsigned test_loopback_en:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_TEST_ASIC_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_TEST_ASIC_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_RES_ASIC_0	 0x00b8
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ASIC for res_req_in */
		unsigned res_req_in:1;
		/* bit[1] Value from ASIC for res_ack_in */
		unsigned res_ack_in:1;
		/* bit[2] Value from ASIC for res_req_out */
		unsigned res_req_out:1;
		/* bit[3] Value from ASIC for res_ack_out */
		unsigned res_ack_out:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_RES_ASIC_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_RES_ASIC_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RCAL_ASIC_IN	 0x00bc
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ASIC for phy_cfg_rcal_bypass */
		unsigned phy_cfg_rcal_bypass:1;
		/* bit[4:1] Value from ASIC for phy_cfg_rcal_code */
		unsigned phy_cfg_rcal_code:4;
		/* bit[8:5] Value from ASIC for phy_cfg_rcal_offset */
		unsigned phy_cfg_rcal_offset:4;
		/* bit[15:9] */
		unsigned RSVD15_9:7;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RCAL_ASIC_IN_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_PHY_CFG_RCAL_ASIC_IN_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UART_AUTORESUME_ASIC	 0x00c0
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ASIC for phy_autoresume_enable */
		unsigned phy_autoresume_enable:1;
		/* bit[1] Value from ASIC for phy_tx_dig_bypass_sel */
		unsigned phy_tx_dig_bypass_sel:1;
		/* bit[2] Value from ASIC for phy_tx_se_dp_en */
		unsigned phy_tx_se_dp_en:1;
		/* bit[3] Value from ASIC for phy_tx_se_dm_en */
		unsigned phy_tx_se_dm_en:1;
		/* bit[4] Value from ASIC for phy_tx_se_dp */
		unsigned phy_tx_se_dp:1;
		/* bit[5] Value from ASIC for phy_tx_se_dm */
		unsigned phy_tx_se_dm:1;
		/* bit[6] Value from ASIC for phy_rx_se_dp */
		unsigned phy_rx_se_dp:1;
		/* bit[7] Value from ASIC for phy_rx_se_dm */
		unsigned phy_rx_se_dm:1;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UART_AUTORESUME_ASIC_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ASIC_UART_AUTORESUME_ASIC_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SCM_L0_TIME	 0x0100
typedef union {
	u16 data;
	struct {
		/* bit[7:0] SCM SE1 TX duration in L0 */
		unsigned scm_l0_time:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SCM_L0_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SCM_L0_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SCM_LX_TIME	 0x0104
typedef union {
	u16 data;
	struct {
		/* bit[9:0] SCM SE1 TX duration on Lx exit */
		unsigned scm_lx_time:10;
		/* bit[15:10] */
		unsigned RSVD15_10:6;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SCM_LX_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SCM_LX_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CM_L0_UI_TIME	 0x0108
typedef union {
	u16 data;
	struct {
		/* bit[7:0] CM UI duration in L0 */
		unsigned cm_l0_ui_time:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CM_L0_UI_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CM_L0_UI_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CM_LX_UI_TIME	 0x010c
typedef union {
	u16 data;
	struct {
		/* bit[9:0] CM UI duration in Lx */
		unsigned cm_lx_ui_time:10;
		/* bit[15:10] */
		unsigned RSVD15_10:6;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CM_LX_UI_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CM_LX_UI_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_LP_MODE_TIME_LO	 0x0110
typedef union {
	u16 data;
	struct {
		/* bit[15:0] lower 16bit of 20bit 3ms timer value */
		unsigned lp_mode_time_lo:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_LP_MODE_TIME_LO_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_LP_MODE_TIME_LO_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_LP_MODE_TIME_HI	 0x0114
typedef union {
	u16 data;
	struct {
		/* bit[3:0] upper 4bit of 20bit 3ms timer value */
		unsigned lp_mode_time_hi:4;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_LP_MODE_TIME_HI_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_LP_MODE_TIME_HI_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SCM_B2B_WAIT_TIME	 0x0118
typedef union {
	u16 data;
	struct {
		/* bit[9:0] Wait time between back-to-back CM messages */
		unsigned scm_b2b_wait_time:10;
		/* bit[15:10] */
		unsigned RSVD15_10:6;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SCM_B2B_WAIT_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SCM_B2B_WAIT_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME	 0x011c
typedef union {
	u16 data;
	struct {
		/* bit[15:0] ESE1 Rx minimum detection time */
		unsigned ese1_rx_time:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_HOST	 0x0120
typedef union {
	u16 data;
	struct {
		/* bit[11:0] HS SOF disconnect ESE1 detect time for eDSPr */
		unsigned ese1_rx_time_host:12;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_HOST_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_HOST_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_NATIVE_HOST_8LSUI	 0x0124
typedef union {
	u16 data;
	struct {
		/* bit[11:0] 8LS UI SE1 detect time for eDSPn to declare ESE1 */
		unsigned ese1_rx_time_native_host_8lsui:12;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_NATIVE_HOST_8LSUI_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_NATIVE_HOST_8LSUI_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_TX_TIME_LO	 0x0128
typedef union {
	u16 data;
	struct {
		/* bit[15:0] ESE1 TX time; lower 16 bits */
		unsigned ese1_tx_time_lo:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_TX_TIME_LO_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_TX_TIME_LO_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_TX_TIME_HI	 0x012c
typedef union {
	u16 data;
	struct {
		/* bit[3:0] EXTSE1 TX time; upper 4 bits */
		unsigned ese1_tx_time_max:4;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_TX_TIME_HI_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_TX_TIME_HI_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_HS_RST_TO_FS_TIME	 0x0130
typedef union {
	u16 data;
	struct {
		/* bit[15:0] Wait time before switching to enumeration state after HS to FS transition */
		unsigned hs_rst_to_fs_time:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_HS_RST_TO_FS_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_HS_RST_TO_FS_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CONNECT_HOST_ACK_TIME	 0x0134
typedef union {
	u16 data;
	struct {
		/* bit[7:0] Native Host connect debounce time */
		unsigned connect_host_ack_time:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CONNECT_HOST_ACK_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CONNECT_HOST_ACK_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SE_SIG_WAIT_TIME	 0x0138
typedef union {
	u16 data;
	struct {
		/* bit[7:0] Wait time between two consecutive SE signals (Port Reset, Config, Connect) */
		unsigned se_sig2sig_wait_time:8;
		/* bit[15:8] Timer to qualify SE1 is received by eUSPn to declare bus reset in HS mode */
		unsigned se1_fltrd_rx_time:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SE_SIG_WAIT_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SE_SIG_WAIT_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CHK_EDP_PULSE_WAIT_TIME	 0x013c
typedef union {
	u16 data;
	struct {
		/* bit[7:0] Timer to ensure the pulse (in response to CM.FS) on eDP is over before moving the state machine */
		unsigned check_edp_pulse_wait_time:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CHK_EDP_PULSE_WAIT_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CHK_EDP_PULSE_WAIT_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_FAST_SIM_CTRL	 0x0140
typedef union {
	u16 data;
	struct {
		/* bit[0] Enable fast BIAS power up (simulation only) */
		unsigned cfg_fast_bias_pu_en:1;
		/* bit[1] Enable fast PLL power up (simulation only) */
		unsigned cfg_fast_pll_pu_en:1;
		/* bit[2] Enable fast TX vref vreg power up (simulation only) */
		unsigned cfg_fast_tx_vref_vreg_pu_en:1;
		/* bit[3] disconnect timer enable for fast mode (simulation only) */
		unsigned cfg_fast_disc_timer_en:1;
		/* bit[4] suspend 3ms timer enable for fast mode (simulation only) */
		unsigned cfg_fast_lp_mode_timer_en:1;
		/* bit[5] ese1 rx timer enable for fast mode (simulation only) */
		unsigned cfg_fast_ese1_rx_timer_en:1;
		/* bit[6] ese1 timer enable for fast mode (simulation only) */
		unsigned cfg_fast_ese1_timer_en:1;
		/* bit[7] hs_rst_to_fs timer enable for fast mode (simulation only) */
		unsigned cfg_fast_hs_rst_to_fs_en:1;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_FAST_SIM_CTRL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_FAST_SIM_CTRL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_DEBUG_CTRL	 0x0144
typedef union {
	u16 data;
	struct {
		/* bit[0] Enumeration bypass configuration */
		unsigned cfg_enum_bypass:1;
		/* bit[1] Disable the bus reset detection while suspend entry in HS mode */
		unsigned cfg_disable_rst_in_susp_entry:1;
		/* bit[2] CM ack check bypass */
		unsigned cfg_skip_cm_ack_check:1;
		/* bit[3] CM ack polarity. 0: value 1 is cm_ack,  1: value 0 is cm_ack */
		unsigned cfg_cm_ack_polarity:1;
		/* bit[4] Enables RX serial shift register in RX IDLE state */
		unsigned fs_rx_kjk_fix:1;
		/* bit[5] FS EOR UI type. 0: LS UI, 1: FS UI. */
		unsigned cfg_fs_eor_ui_type:1;
		/* bit[6] Configuration to skip both ping generation(eUSPn) and checking(eDSPn) */
		unsigned cfg_skip_ping:1;
		/* bit[15:7] */
		unsigned RSVD15_7:9;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_DEBUG_CTRL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_DEBUG_CTRL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_DTB_DEBUG_SEL	 0x0148
typedef union {
	u16 data;
	struct {
		/* bit[7:0] dtb debug selection */
		unsigned cfg_dtb_debug_sel:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_DTB_DEBUG_SEL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_DTB_DEBUG_SEL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_OCLA_DEBUG_SEL	 0x014c
typedef union {
	u16 data;
	struct {
		/* bit[0] ocla debug enable */
		unsigned cfg_ocla_debug_en:1;
		/* bit[4:1] ocla debug selection */
		unsigned cfg_ocla_debug_sel:4;
		/* bit[15:5] */
		unsigned RSVD15_5:11;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_OCLA_DEBUG_SEL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_OCLA_DEBUG_SEL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SUSP_DISC_TIMER_VAL_LO	 0x0150
typedef union {
	u16 data;
	struct {
		/* bit[15:0] susp_disc_timer_val lower 16bit */
		unsigned susp_disc_timer_val_lo:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SUSP_DISC_TIMER_VAL_LO_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SUSP_DISC_TIMER_VAL_LO_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SUSP_DISC_TIMER_VAL_HI	 0x0154
typedef union {
	u16 data;
	struct {
		/* bit[2:0] susp_disc_timer_val upper 3 bits */
		unsigned susp_disc_timer_val_hi:3;
		/* bit[15:3] */
		unsigned RSVD15_3:13;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SUSP_DISC_TIMER_VAL_HI_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SUSP_DISC_TIMER_VAL_HI_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_EUSB_STATE_JMP_DBG_IN	 0x0158
typedef union {
	u16 data;
	struct {
		/* bit[4:0] eusb_state to be jumped */
		unsigned eusb_state_jmp_dbg_in:5;
		/* bit[5] eusb_state jump enable for debug */
		unsigned eusb_state_jmp_dbg_en:1;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_EUSB_STATE_JMP_DBG_IN_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_EUSB_STATE_JMP_DBG_IN_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_EUSB_STATE_JMP_DBG_OUT	 0x015c
typedef union {
	u16 data;
	struct {
		/* bit[4:0] current eusb_state */
		unsigned eusb_state_jmp_dbg_out:5;
		/* bit[15:5] */
		unsigned RSVD15_5:11;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_EUSB_STATE_JMP_DBG_OUT_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_EUSB_STATE_JMP_DBG_OUT_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_HS_DBG_IN	 0x0160
typedef union {
	u16 data;
	struct {
		/* bit[0] tap_shift_dir_tm */
		unsigned tap_shift_dir_tm:1;
		/* bit[1] tap_shift_enb_tm */
		unsigned tap_shift_enb_tm:1;
		/* bit[2] tap_sel_ovrd_tm */
		unsigned tap_sel_ovrd_tm:1;
		/* bit[6:3] Number of ser_clk cycles to filter rising edge of HS squelch */
		unsigned sq_fltr_val_tm:4;
		/* bit[15:7] */
		unsigned RSVD15_7:9;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_HS_DBG_IN_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_HS_DBG_IN_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_HS_DBG_OUT	 0x0164
typedef union {
	u16 data;
	struct {
		/* bit[3:0] tap_val information */
		unsigned tap_val:4;
		/* bit[4] ldTransInd_debug  information */
		unsigned ldtransind_debug:1;
		/* bit[5] changeEn_debug  information */
		unsigned changeen_debug:1;
		/* bit[6] dirToChange_debug  information */
		unsigned dirtochange_debug:1;
		/* bit[15:7] */
		unsigned RSVD15_7:9;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_HS_DBG_OUT_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_HS_DBG_OUT_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_CFG_FSLS_CONN_TIME	 0x0168
typedef union {
	u16 data;
	struct {
		/* bit[7:0] FS/LS connection configuration time */
		unsigned cfg_fsls_conn_time:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_CFG_FSLS_CONN_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_CFG_FSLS_CONN_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_CFG_FSLS_DISC_TIME	 0x016c
typedef union {
	u16 data;
	struct {
		/* bit[7:0] FS/LS disconnection configuration time */
		unsigned cfg_fsls_disc_time:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_CFG_FSLS_DISC_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_CFG_FSLS_DISC_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SE1_CHK_TIME	 0x0170
typedef union {
	u16 data;
	struct {
		/* bit[7:0] SE1 check time before blocking Tx path when ese1 Rx detection is enabled. Valid for Native mode only. */
		unsigned cfg_se1_chk_time:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SE1_CHK_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_SE1_CHK_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CM_TEST_OVRD_IN	 0x0174
typedef union {
	u16 data;
	struct {
		/* bit[0] CM.TEST override value */
		unsigned cfg_cm_test:1;
		/* bit[1] CM.TEST override enable */
		unsigned cfg_cm_test_ovrd_en:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CM_TEST_OVRD_IN_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_CM_TEST_OVRD_IN_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RSM_WKUP_PKT_ON_EXT_TIME	 0x0178
typedef union {
	u16 data;
	struct {
		/* bit[7:0] packet on extension time during receiving resume */
		unsigned resume_pkt_on_ext_time:8;
		/* bit[15:8] packet on extension time during transmitting wakeup */
		unsigned eow_norsm_pkt_ext_time:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RSM_WKUP_PKT_ON_EXT_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RSM_WKUP_PKT_ON_EXT_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_HS_FSLS_SE0_DRV	 0x017c
typedef union {
	u16 data;
	struct {
		/* bit[2:0] TX HS SE0 drive value in HS UI */
		unsigned tx_hs_se0_drv_r:3;
		/* bit[7:3] TX_FS/LS SE0 drive value in HS UI */
		unsigned tx_fsls_se0_drv_r:5;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_HS_FSLS_SE0_DRV_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_HS_FSLS_SE0_DRV_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_CFG_EOP_FLTRD_CNT_VAL	 0x0180
typedef union {
	u16 data;
	struct {
		/* bit[1:0] FS eop filter count value */
		unsigned cfg_fs_eop_fltr_cnt_val:2;
		/* bit[6:2] LS eop filter count value */
		unsigned cfg_ls_eop_fltr_cnt_val:5;
		/* bit[15:7] */
		unsigned RSVD15_7:9;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_CFG_EOP_FLTRD_CNT_VAL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_CFG_EOP_FLTRD_CNT_VAL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TXRX_PING_GEN_CHK_CNT_VAL_0	 0x0184
typedef union {
	u16 data;
	struct {
		/* bit[0] If set, digital ping in resume EOP is generated if resume is in HS or FS/LS. */
		unsigned cfg_rsm_end_ping_gen:1;
		/* bit[5:1] Delay to start digital ping generation to differentiate resume EOP or EOP at HS end of reset */
		unsigned cfg_ping_start_dly:5;
		/* bit[11:6] Controls the duration of the analog ping check window after SOF */
		unsigned cfg_analog_ping_window_cnt_val:6;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TXRX_PING_GEN_CHK_CNT_VAL_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TXRX_PING_GEN_CHK_CNT_VAL_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TXRX_PING_GEN_CHK_CNT_VAL_1	 0x0188
typedef union {
	u16 data;
	struct {
		/* bit[5:0] Analog ping generation start count value */
		unsigned cfg_analog_ping_start_marker:6;
		/* bit[11:6] Analog ping generation end count value */
		unsigned cfg_analog_ping_end_marker:6;
		/* bit[13:12] Number of missing analog pings (> 1) for disconnect */
		unsigned cfg_analog_ping_miss_prog_val:2;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TXRX_PING_GEN_CHK_CNT_VAL_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TXRX_PING_GEN_CHK_CNT_VAL_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TXRX_PING_GEN_CHK_CNT_VAL_2	 0x018c
typedef union {
	u16 data;
	struct {
		/* bit[5:0] programming value on which analog ping checking window starts */
		unsigned chk_ana_ping_window_start_val:6;
		/* bit[11:6] programming value for start count for anlaog ping checking */
		unsigned chk_ana_ping_start_marker_val:6;
		/* bit[14:12] programming value for ping pulse gen */
		unsigned chk_ana_ping_pulse_prog_val:3;
		/* bit[15] */
		unsigned RSVD15:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TXRX_PING_GEN_CHK_CNT_VAL_2_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TXRX_PING_GEN_CHK_CNT_VAL_2_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_NATIVE_HOST_2FSUI	 0x0190
typedef union {
	u16 data;
	struct {
		/* bit[11:0] 2FS UI SE1 detect time for native host to assert ESE1 */
		unsigned ese1_rx_time_native_host_2fsui:12;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_NATIVE_HOST_2FSUI_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_ESE1_RX_TIME_NATIVE_HOST_2FSUI_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_DTB_CREG_DATA	 0x0194
typedef union {
	u16 data;
	struct {
		/* bit[15:0] 16bit DTB data selected by dtb_debug_sel */
		unsigned dtb_creg_data:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_DTB_CREG_DATA_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_DTB_CREG_DATA_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_DIG_CTRL	 0x0198
typedef union {
	u16 data;
	struct {
		/* bit[2:0] Controls the back to back TX packet handling */
		unsigned cfg_utmi_tx_ctrl:3;
		/* bit[3] Controls the TX start delay for the LS keep alive */
		unsigned cfg_ls_kpalive_ctrl:1;
		/* bit[8:4] Duration of SOR/SOW after L2 with a resolution of 4 FS bit times */
		unsigned cfg_sor_l2_len:5;
		/* bit[13:9] Duration of SOR/SOW after L1 with a resolution of 2 FS bit times */
		unsigned cfg_sor_l1_len:5;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_DIG_CTRL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_DIG_CTRL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_DIG_CTRL	 0x019c
typedef union {
	u16 data;
	struct {
		/* bit[0] Swaps the SE driver output from M to P */
		unsigned cfg_rx_se_m_swap:1;
		/* bit[1] Swaps the SE driver output from P to M */
		unsigned cfg_rx_se_p_swap:1;
		/* bit[2] Inverts the data line in FS/LS data recovery block */
		unsigned cfg_fsls_data_invert:1;
		/* bit[8:3] Receiver timeout value */
		unsigned cfg_rcv_timeout_val:6;
		/* bit[11:9] Squelch delay for linestate generation (SCLK cycle) */
		unsigned cfg_squelch_ls_dly:3;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_DIG_CTRL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_RX_DIG_CTRL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_HS_SE0_OFST_CTRL_0	 0x01a0
typedef union {
	u16 data;
	struct {
		/* bit[0] TX HS Driver offset disable */
		unsigned tx_hs_ofst_drv_dis_r:1;
		/* bit[3:1] TX HS Driver offset duration for tx_fsls_vref_tune = 01 */
		unsigned tx_hs_ofst_drv_1_r:3;
		/* bit[6:4] TX HS Driver offset duration for tx_fsls_vref_tune = 10 */
		unsigned tx_hs_ofst_drv_2_r:3;
		/* bit[9:7] TX HS Driver offset duration for tx_fsls_vref_tune = 11 */
		unsigned tx_hs_ofst_drv_3_r:3;
		/* bit[10] TX HS Driver offset level for tx_fsls_vref_tune = 01 */
		unsigned tx_hs_ofst_lvl_1_r:1;
		/* bit[11] TX HS Driver offset level for tx_fsls_vref_tune = 10 */
		unsigned tx_hs_ofst_lvl_2_r:1;
		/* bit[12] TX HS Driver offset level for tx_fsls_vref_tune = 11 */
		unsigned tx_hs_ofst_lvl_3_r:1;
		/* bit[15:13] */
		unsigned RSVD15_13:3;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_HS_SE0_OFST_CTRL_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_HS_SE0_OFST_CTRL_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_HS_SE0_OFST_CTRL_1	 0x01a4
typedef union {
	u16 data;
	struct {
		/* bit[2:0] TX HS Driver SE0 duration for tx_fsls_vref_tune = 01 */
		unsigned tx_hs_se0_drv_1_r:3;
		/* bit[5:3] TX HS Driver SE0 duration for tx_fsls_vref_tune = 10 */
		unsigned tx_hs_se0_drv_2_r:3;
		/* bit[8:6] TX HS Driver SE0 duration for tx_fsls_vref_tune = 11 */
		unsigned tx_hs_se0_drv_3_r:3;
		/* bit[15:9] */
		unsigned RSVD15_9:7;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_HS_SE0_OFST_CTRL_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_TX_HS_SE0_OFST_CTRL_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_PHY_CFG	 0x01a8
typedef union {
	u16 data;
	struct {
		/* bit[7:0] PHY CONFIGURATION values */
		unsigned phy_cfg:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_PHY_CFG_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_PHY_CFG_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_BIAS_PWRUP_TIME	 0x0200
typedef union {
	u16 data;
	struct {
		/* bit[6:0] Time (in ref_dig_clk cycles) for BIAS block to power-up after chop en is asserted (spec 1us) */
		unsigned bias_pu_time:7;
		/* bit[13:7] Time (in ref_dig_clk cycles) for BIAS block after which the chop en can be asserted after bias_pu (spec 2us) */
		unsigned bias_chop_en_time:7;
		/* bit[14] Enable BIAS_CHOP_EN_TIME and BIAS_PU_TIME register settings */
		unsigned bias_time_en:1;
		/* bit[15] */
		unsigned RSVD15:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_BIAS_PWRUP_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_BIAS_PWRUP_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRUP_TIME_0	 0x0204
typedef union {
	u16 data;
	struct {
		/* bit[9:0] PLL reset pulse width (in ref_dig_clk cycles) (spec 4us) */
		unsigned pll_reset_time:10;
		/* bit[10] Enable PLL_RESET_TIME register setting */
		unsigned pll_reset_time_en:1;
		/* bit[15:11] */
		unsigned RSVD15_11:5;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRUP_TIME_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRUP_TIME_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRUP_TIME_1	 0x0208
typedef union {
	u16 data;
	struct {
		/* bit[9:0] PLL Gear Shift pulse width (in ref_dig_clk cycles) (spec 5us) */
		unsigned pll_gear_shift_time:10;
		/* bit[10] Enable PLL_GEAR_SHIFT_TIME register setting */
		unsigned pll_gear_shift_time_en:1;
		/* bit[15:11] */
		unsigned RSVD15_11:5;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRUP_TIME_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRUP_TIME_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRUP_TIME_2	 0x020c
typedef union {
	u16 data;
	struct {
		/* bit[9:0] PLL lock time duration after gear-shift (spec 5us) */
		unsigned pll_lock_time:10;
		/* bit[10] Enable PLL_LOCK_TIME register setting */
		unsigned pll_lock_time_en:1;
		/* bit[15:11] */
		unsigned RSVD15_11:5;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRUP_TIME_2_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRUP_TIME_2_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_TX_PWRUP_TIME	 0x0210
typedef union {
	u16 data;
	struct {
		/* bit[6:0] Time (in ref_dig_clk cycles) required for TX reference voltage regulator to power up (spec 2us) */
		unsigned tx_vref_vreg_pu_time:7;
		/* bit[13:7] Time (in ref_dig_clk cycles) required for waiting after PLL lock to enable the TX reference voltage regulator (spec 1us) */
		unsigned tx_vref_vreg_pu_wait_time:7;
		/* bit[14] Enable TX_VREF_VREG_PU_WAIT_TIME and TX_VREF_VREG_PU_TIME register setting */
		unsigned tx_vref_vreg_pu_en:1;
		/* bit[15] */
		unsigned RSVD15:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_TX_PWRUP_TIME_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_TX_PWRUP_TIME_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRDN_TIME_0	 0x0214
typedef union {
	u16 data;
	struct {
		/* bit[4:0] PLL power-down time (spec 0us) */
		unsigned pll_pwrdn_time:5;
		/* bit[5] Enable PLL power-down time */
		unsigned pll_pwrdn_time_en:1;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRDN_TIME_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_PLL_PWRDN_TIME_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_RCAL_COMP_TEST	 0x0218
typedef union {
	u16 data;
	struct {
		/* bit[0] RTUNE comp test enable */
		unsigned comp_test:1;
		/* bit[1] RTUNE comp result flip bit */
		unsigned comp_result_flip:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_RCAL_COMP_TEST_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_RCAL_COMP_TEST_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_RCAL_COMP_RESULT	 0x021c
typedef union {
	u16 data;
	struct {
		/* bit[0] RTUNE comp result latch enable */
		unsigned comp_result_latch_enb:1;
		/* bit[1] RTUNE comp result */
		unsigned comp_result_latch:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_RCAL_COMP_RESULT_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_RCAL_COMP_RESULT_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_CTL_STATUS	 0x0220
typedef union {
	u16 data;
	struct {
		/* bit[3:0] phy ana control state */
		unsigned pwr_state:4;
		/* bit[4] power up request */
		unsigned pwr_up_req:1;
		/* bit[5] power up done */
		unsigned pwr_up_done:1;
		/* bit[6] power down request */
		unsigned pwr_dn_req:1;
		/* bit[7] power down done */
		unsigned pwr_dn_done:1;
		/* bit[8] Change to PLL clock request */
		unsigned change_to_pll_clk:1;
		/* bit[9] PLL clock selected */
		unsigned pll_clk_selected:1;
		/* bit[10] synchornized version of pll_lock on pclk */
		unsigned pll_lock_sync:1;
		/* bit[11] RTUNE request */
		unsigned tune_req:1;
		/* bit[12] RTUNE done */
		unsigned tune_done:1;
		/* bit[13] Generic Timer completion */
		unsigned wait_cnt_ne_0:1;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_CTL_STATUS_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_CTL_STATUS_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_MP_CTL_OVRD_0	 0x0224
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value of mstr_xcvr input */
		unsigned mstr_xcvr_ovrd:1;
		/* bit[1] Override enable of mstr_xcvr input */
		unsigned mstr_xcvr_ovrd_en:1;
		/* bit[2] Override value of mstr_xcvr_pwr_up_req input */
		unsigned mstr_xcvr_pwr_up_req_ovrd:1;
		/* bit[3] Override enable of mstr_xcvr_pwr_up_req input */
		unsigned mstr_xcvr_pwr_up_req_ovrd_en:1;
		/* bit[4] Override value of xcvr_pwr_up_req output */
		unsigned xcvr_pwr_up_req_ovrd:1;
		/* bit[5] Override enable of xcvr_pwr_up_req output */
		unsigned xcvr_pwr_up_req_ovrd_en:1;
		/* bit[6] Override value of xcvr_pll_on_out output */
		unsigned xcvr_pll_on_out_ovrd:1;
		/* bit[7] Override enable of xcvr_pll_on_out output */
		unsigned xcvr_pll_on_out_ovrd_en:1;
		/* bit[8] Override value of xcvr_pll_on_in input */
		unsigned xcvr_pll_on_in_ovrd:1;
		/* bit[9] Override enable of xcvr_pll_on_in input */
		unsigned xcvr_pll_on_in_ovrd_en:1;
		/* bit[10] Override value of xcvr_pwr_dn_done output */
		unsigned xcvr_pwr_dn_done_ovrd:1;
		/* bit[11] Override enable of xcvr_pwr_dn_done output */
		unsigned xcvr_pwr_dn_done_ovrd_en:1;
		/* bit[12] Override value of mstr_xcvr_pwr_dn_req input */
		unsigned mstr_xcvr_pwr_dn_req_ovrd:1;
		/* bit[13] Override enable of mstr_xcvr_pwr_dn_req input */
		unsigned mstr_xcvr_pwr_dn_req_ovrd_en:1;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_MP_CTL_OVRD_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_MP_CTL_OVRD_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_MP_CTL_OVRD_1	 0x0228
typedef union {
	u16 data;
	struct {
		/* bit[3:0] Override value of  input xcvr_rtune_code_in */
		unsigned xcvr_rtune_code_in_ovrd:4;
		/* bit[4] Override enable of  input xcvr_rtune_code_in */
		unsigned xcvr_rtune_code_in_ovrd_en:1;
		/* bit[5] Override value of  input xcvr_rtune_code_rdy_in */
		unsigned xcvr_rtune_code_rdy_in_ovrd:1;
		/* bit[6] Override enable of  input xcvr_rtune_code_rdy_in */
		unsigned xcvr_rtune_code_rdy_in_ovrd_en:1;
		/* bit[10:7] Override value of  input xcvr_rtune_code_out */
		unsigned xcvr_rtune_code_out_ovrd:4;
		/* bit[11] Override enable of  input xcvr_rtune_code_out */
		unsigned xcvr_rtune_code_out_ovrd_en:1;
		/* bit[12] Override value of  input xcvr_rtune_code_rdy_out */
		unsigned xcvr_rtune_code_rdy_out_ovrd:1;
		/* bit[13] Override enable of  input xcvr_rtune_code_rdy_out */
		unsigned xcvr_rtune_code_rdy_out_ovrd_en:1;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_MP_CTL_OVRD_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_MP_CTL_OVRD_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_MP_CFG_VAL	 0x022c
typedef union {
	u16 data;
	struct {
		/* bit[9:0] Bypass path wait time in idle state */
		unsigned bp_path_wait_time:10;
		/* bit[15:10] */
		unsigned RSVD15_10:6;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_MP_CFG_VAL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_MP_CFG_VAL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_RES_ACK_IN_WAIT_DLY	 0x0230
typedef union {
	u16 data;
	struct {
		/* bit[7:0] Programmable wait delay for res_ack_in */
		unsigned res_ack_in_wait_dly:8;
		/* bit[15:8] */
		unsigned RSVD15_8:8;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_RES_ACK_IN_WAIT_DLY_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_RES_ACK_IN_WAIT_DLY_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_QUOT	 0x0240
typedef union {
	u16 data;
	struct {
		/* bit[15:0] frac_quot value (default value for 26MHz) */
		unsigned frac_quot:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_QUOT_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_QUOT_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_DEN	 0x0244
typedef union {
	u16 data;
	struct {
		/* bit[15:0] frac_den value (default value for 26MHz) */
		unsigned frac_den:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_DEN_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_DEN_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_REM	 0x0248
typedef union {
	u16 data;
	struct {
		/* bit[15:0] frac_rem value (default value for 26MHz) */
		unsigned frac_rem:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_REM_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_REM_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_OUT_OVRD	 0x024c
typedef union {
	u16 data;
	struct {
		/* bit[14:0] frac_out[19:5] override value This register will override the MSB's <19:5> of the frac_out
		 *  register.  When overridden, the LSBs <4:0> will be set to 5'd0. */
		unsigned frac_out_ovrd_val:15;
		/* bit[15] frac_out override enable */
		unsigned frac_out_ovrd_en:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_OUT_OVRD_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_OUT_OVRD_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_SSC_RAMP_OVRD	 0x0250
typedef union {
	u16 data;
	struct {
		/* bit[14:0] ssc_ramp[19:5] override value This register will override the MSB's <19:5> of the ssc_ramp
		 *  register.  When overridden, the LSBs <4:0> will be set to 5'd0. */
		unsigned ssc_ramp_ovrd_val:15;
		/* bit[15] ssc_ramp override enable */
		unsigned ssc_ramp_ovrd_en:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_SSC_RAMP_OVRD_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_SSC_RAMP_OVRD_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_CONFIG	 0x0254
typedef union {
	u16 data;
	struct {
		/* bit[0] Status registers reading update This is self clear field. */
		unsigned rd_update:1;
		/* bit[1] Bypass the mint_i[11:0] override logic. */
		unsigned bypass_pll_logic:1;
		/* bit[2] Fractional enable over-ride value */
		unsigned frac_en_ovrd_val:1;
		/* bit[3] Fractional enable over-ride enable */
		unsigned frac_en_ovrd_en:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_CONFIG_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_CONFIG_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_OUT_STATUS	 0x0258
typedef union {
	u16 data;
	struct {
		/* bit[14:0] Current value of frac_out_pre[19:5] */
		unsigned frac_out_stats_val:15;
		/* bit[15] */
		unsigned RSVD15:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_OUT_STATUS_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_FRAC_OUT_STATUS_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_SSC_RAMP_STATUS	 0x025c
typedef union {
	u16 data;
	struct {
		/* bit[14:0] Current value of ssc_ramp_pre_sat[20:6] */
		unsigned ssc_ramp_stats_val:15;
		/* bit[15] */
		unsigned RSVD15:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_SSC_RAMP_STATUS_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_PWR_FSM_SSGEN_SSC_RAMP_STATUS_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_BIST_CTRL	 0x0280
typedef union {
	u16 data;
	struct {
		/* bit[0] Bist Enable signal */
		unsigned bist_en:1;
		/* bit[1] Bist Mode;   1'b0 transmits a 00 and FF pattern with NRZI; 1'b1 transmits a USB2.0 test-packet bist; should be set before other bist enable signals. */
		unsigned bist_mode:1;
		/* bit[2] Repeat mode; 1'b0 transmits a single test-packet mode; 1'b1 transmits in infinite test-packet mode;  should be set before other bist enable signals. */
		unsigned test_pkt_rpt:1;
		/* bit[3] HS Bist mode */
		unsigned hs_bist:1;
		/* bit[4] FS Bist mode */
		unsigned fs_bist:1;
		/* bit[5] LS Bist mode */
		unsigned ls_bist:1;
		/* bit[6] Enables injecting intentional bit error during both test-packet and all 0's/1's bist pattern. */
		unsigned inst_bist_err:1;
		/* bit[7] Disable bist error generation during false HS or FS/LS eop detection. */
		unsigned bist_err_feop_det_disable:1;
		/* bit[8] Enable generation of Port Reset (ESE1) TX prior each bist pattern generation. */
		unsigned bist_ese1_gen_en:1;
		/* bit[15:9] */
		unsigned RSVD15_9:7;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_BIST_CTRL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_BIST_CTRL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_BIST_STATUS	 0x0284
typedef union {
	u16 data;
	struct {
		/* bit[0] Bist done signal */
		unsigned bist_done:1;
		/* bit[1] Bist Error indication */
		unsigned bist_error:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_BIST_STATUS_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_BIST_STATUS_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_HS_ANA_OVRD_OUT_0	 0x0300
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for tx_hs_data_post */
		unsigned tx_hs_data_post_ovrd:1;
		/* bit[1] Enable of override value for tx_hs_data_post */
		unsigned tx_hs_data_post_ovrd_en:1;
		/* bit[2] Override value for tx_hs_data */
		unsigned tx_hs_data_ovrd:1;
		/* bit[3] Enable of override value for tx_hs_data */
		unsigned tx_hs_data_ovrd_en:1;
		/* bit[4] Override value for tx_hs_k_ofst */
		unsigned tx_hs_k_ofst_ovrd:1;
		/* bit[5] Enable of override value for tx_hs_k_ofst */
		unsigned tx_hs_k_ofst_ovrd_en:1;
		/* bit[6] Override value for tx_hs_j_ofst */
		unsigned tx_hs_j_ofst_ovrd:1;
		/* bit[7] Enable of override value for tx_hs_j_ofst */
		unsigned tx_hs_j_ofst_ovrd_en:1;
		/* bit[8] Override value for tx_hs_se0 */
		unsigned tx_hs_se0_ovrd:1;
		/* bit[9] Enable of override value for tx_hs_se0 */
		unsigned tx_hs_se0_ovrd_en:1;
		/* bit[10] Override value for tx_hs_en */
		unsigned tx_hs_en_ovrd:1;
		/* bit[11] Enable of override value for tx_hs_en */
		unsigned tx_hs_en_ovrd_en:1;
		/* bit[12] Override value for tx_hs_ofst_lvl */
		unsigned tx_hs_ofst_lvl_ovrd:1;
		/* bit[13] Enable of override value for tx_hs_ofst_lvl */
		unsigned tx_hs_ofst_lvl_ovrd_en:1;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_HS_ANA_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_HS_ANA_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_OVRD_OUT_0	 0x0304
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for rx_hs_en */
		unsigned rx_hs_en_ovrd:1;
		/* bit[1] Enable of override value for rx_hs_en */
		unsigned rx_hs_en_ovrd_en:1;
		/* bit[2] Override value for rx_hs_squelch_en */
		unsigned rx_hs_squelch_en_ovrd:1;
		/* bit[3] Enable of override value for rx_hs_squelch_en */
		unsigned rx_hs_squelch_en_ovrd_en:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_OVRD_IN_0	 0x0308
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for rx_hs_data */
		unsigned rx_hs_data_ovrd:1;
		/* bit[1] Enable of override value for rx_hs_data */
		unsigned rx_hs_data_ovrd_en:1;
		/* bit[2] Override value for rx_hs_squelch */
		unsigned rx_hs_squelch_ovrd:1;
		/* bit[3] Enable of override value for rx_hs_squelch */
		unsigned rx_hs_squelch_ovrd_en:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_SE_ANA_OVRD_OUT_0	 0x030c
typedef union {
	u16 data;
	struct {
		/* bit[4:0] Override value for tx_se_data_m */
		unsigned tx_se_data_m_ovrd:5;
		/* bit[5] Enable of override value for tx_se_data_m */
		unsigned tx_se_data_m_ovrd_en:1;
		/* bit[10:6] Override value for tx_se_data_p */
		unsigned tx_se_data_p_ovrd:5;
		/* bit[11] Enable of override value for tx_se_data_p */
		unsigned tx_se_data_p_ovrd_en:1;
		/* bit[12] Override value for tx_se_en_m */
		unsigned tx_se_en_m_ovrd:1;
		/* bit[13] Enable of override value for tx_se_en_m */
		unsigned tx_se_en_m_ovrd_en:1;
		/* bit[14] Override value for tx_se_en_p */
		unsigned tx_se_en_p_ovrd:1;
		/* bit[15] Enable of override value for tx_se_en_p */
		unsigned tx_se_en_p_ovrd_en:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_SE_ANA_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_SE_ANA_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_EN_ANA_OVRD_OUT_0	 0x0310
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for rx_se_en_m */
		unsigned rx_se_en_m_ovrd:1;
		/* bit[1] Enable of override value for rx_se_en_m */
		unsigned rx_se_en_m_ovrd_en:1;
		/* bit[2] Override value for rx_se_en_p */
		unsigned rx_se_en_p_ovrd:1;
		/* bit[3] Enable of override value for rx_se_en_p */
		unsigned rx_se_en_p_ovrd_en:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_EN_ANA_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_EN_ANA_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_DATA_ANA_OVRD_IN_0	 0x0314
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for rx_se_data_m */
		unsigned rx_se_data_m_ovrd:1;
		/* bit[1] Enable of override value for rx_se_data_m */
		unsigned rx_se_data_m_ovrd_en:1;
		/* bit[2] Override value for rx_se_data_p */
		unsigned rx_se_data_p_ovrd:1;
		/* bit[3] Enable of override value for rx_se_data_p */
		unsigned rx_se_data_p_ovrd_en:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_DATA_ANA_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_DATA_ANA_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PULLDOWN_OVRD_OUT_0	 0x0318
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for pulldown_en_m */
		unsigned pulldown_en_m_ovrd:1;
		/* bit[1] Enable of override value for pulldown_en_m */
		unsigned pulldown_en_m_ovrd_en:1;
		/* bit[2] Override value for pulldown_en_p */
		unsigned pulldown_en_p_ovrd:1;
		/* bit[3] Enable of override value for pulldown_en_p */
		unsigned pulldown_en_p_ovrd_en:1;
		/* bit[15:4] */
		unsigned RSVD15_4:12;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PULLDOWN_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PULLDOWN_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_OUT_0	 0x031c
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for */
		unsigned bias_pu_ovrd:1;
		/* bit[1] Enable of override value for */
		unsigned bias_pu_ovrd_en:1;
		/* bit[2] Override value for */
		unsigned bias_chop_en_ovrd:1;
		/* bit[3] Enable of override value for */
		unsigned bias_chop_en_ovrd_en:1;
		/* bit[4] Override value for pll_pu */
		unsigned pll_pu_ovrd:1;
		/* bit[5] Enable of override value for pll_pu */
		unsigned pll_pu_ovrd_en:1;
		/* bit[6] Override value for pll_reset */
		unsigned pll_reset_ovrd:1;
		/* bit[7] Enable of override value for pll_reset */
		unsigned pll_reset_ovrd_en:1;
		/* bit[8] Override value for pll_gear_shift */
		unsigned pll_gear_shift_ovrd:1;
		/* bit[9] Enable of override value for pll_gear_shift */
		unsigned pll_gear_shift_ovrd_en:1;
		/* bit[10] Override value for pll_force_lock */
		unsigned pll_force_lock_ovrd:1;
		/* bit[11] Enable of override value for pll_force_lock */
		unsigned pll_force_lock_ovrd_en:1;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_OUT_1	 0x0320
typedef union {
	u16 data;
	struct {
		/* bit[3:0] Override value for pll_phy_n */
		unsigned pll_phy_n_ovrd:4;
		/* bit[4] Enable of override value for pll_phy_n */
		unsigned pll_phy_n_ovrd_en:1;
		/* bit[10:5] Override value for pll_pmix_frac */
		unsigned pll_pmix_frac_ovrd:6;
		/* bit[11] Enable of override value for pll_pmix_frac */
		unsigned pll_pmix_frac_ovrd_en:1;
		/* bit[13:12] Override value for pll_pmix_int */
		unsigned pll_pmix_int_ovrd:2;
		/* bit[14] Enable of override value for pll_pmix_int */
		unsigned pll_pmix_int_ovrd_en:1;
		/* bit[15] */
		unsigned RSVD15:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_OUT_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_OUT_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_OUT_2	 0x0324
typedef union {
	u16 data;
	struct {
		/* bit[11:0] Override value for pll_phy_m */
		unsigned pll_phy_m_ovrd:12;
		/* bit[12] Enable of override value for pll_phy_m */
		unsigned pll_phy_m_ovrd_en:1;
		/* bit[13] Override value for pll_pmix_frac_en */
		unsigned pll_pmix_frac_en_ovrd:1;
		/* bit[14] Enable of override value for pll_pmix_frac_en */
		unsigned pll_pmix_frac_en_ovrd_en:1;
		/* bit[15] */
		unsigned RSVD15:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_OUT_2_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_OUT_2_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_IN_0	 0x0328
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for pll_phy_m */
		unsigned pll_lock_ovrd:1;
		/* bit[1] Enable of override value for pll_lock */
		unsigned pll_lock_ovrd_en:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_OVRD_OUT_0	 0x032c
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for tx_vref_vreg_pu */
		unsigned tx_vref_vreg_pu_ovrd:1;
		/* bit[1] Enable of override value for tx_vref_vreg_pu */
		unsigned tx_vref_vreg_pu_ovrd_en:1;
		/* bit[2] Override value for tx_vreg1v_bypass */
		unsigned tx_vreg1v_bypass_ovrd:1;
		/* bit[3] Enable of override value for tx_vreg1v_bypass */
		unsigned tx_vreg1v_bypass_ovrd_en:1;
		/* bit[4] Override value for tx_fsls_slew_rate_ctrl */
		unsigned tx_fsls_slew_rate_ctrl_ovrd:1;
		/* bit[5] Enable of override value for tx_fsls_slew_rate_ctrl */
		unsigned tx_fsls_slew_rate_ctrl_ovrd_en:1;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_OVRD_OUT_0	 0x0330
typedef union {
	u16 data;
	struct {
		/* bit[3:0] Override value for tx_sliver_en */
		unsigned tx_sliver_en_ovrd:4;
		/* bit[4] Enable of override value for tx_sliver_en */
		unsigned tx_sliver_en_ovrd_en:1;
		/* bit[5] Override value for tx_rtune_en */
		unsigned tx_rtune_en_ovrd:1;
		/* bit[6] Enable of override value for tx_rtune_en */
		unsigned tx_rtune_en_ovrd_en:1;
		/* bit[7] Override value for comp_clk */
		unsigned comp_clk_ovrd:1;
		/* bit[8] Enable of override value for comp_clk */
		unsigned comp_clk_ovrd_en:1;
		/* bit[12:9] Override value for rtune_code */
		unsigned rtune_code_ovrd:4;
		/* bit[13] Enable of override value for rtune_code */
		unsigned rtune_code_ovrd_en:1;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_OVRD_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_OVRD_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_OVRD_IN_0	 0x0334
typedef union {
	u16 data;
	struct {
		/* bit[0] Override value for comp_result */
		unsigned comp_result_ovrd:1;
		/* bit[1] Enable of override value for comp_result */
		unsigned comp_result_ovrd_en:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_OVRD_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_OVRD_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_MPLL_PRG_VAL	 0x0338
typedef union {
	u16 data;
	struct {
		/* bit[15:0] pll_mpll_prg value */
		unsigned pll_mpll_prg:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_MPLL_PRG_VAL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_MPLL_PRG_VAL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_CMN_SEL	 0x033c
typedef union {
	u16 data;
	struct {
		/* bit[5:0] ATB test select for common module */
		unsigned atb_cmn_sel:6;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_CMN_SEL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_CMN_SEL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_XCVR_SEL	 0x0340
typedef union {
	u16 data;
	struct {
		/* bit[5:0] ATB test select for XCVR analog module */
		unsigned atb_xcvr_sel:6;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_XCVR_SEL_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_XCVR_SEL_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_TERM_EN_OVRD	 0x0344
typedef union {
	u16 data;
	struct {
		/* bit[0] Rx HS termination enable override register value */
		unsigned rx_hs_term_en_ovrd:1;
		/* bit[1] Rx HS termination enable override register enable */
		unsigned rx_hs_term_en_ovrd_en:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_TERM_EN_OVRD_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_TERM_EN_OVRD_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_HS_ANA_OUT_0	 0x0380
typedef union {
	u16 data;
	struct {
		/* bit[0] Value to ANA for tx_hs_data_post */
		unsigned tx_hs_data_post:1;
		/* bit[1] Value to ANA for tx_hs_data */
		unsigned tx_hs_data:1;
		/* bit[2] Value to ANA for tx_hs_k_ofst */
		unsigned tx_hs_k_ofst:1;
		/* bit[3] Value to ANA for tx_hs_j_ofst */
		unsigned tx_hs_j_ofst:1;
		/* bit[4] Value to ANA for tx_hs_se0 */
		unsigned tx_hs_se0:1;
		/* bit[5] Value to ANA for tx_hs_en */
		unsigned tx_hs_en:1;
		/* bit[6] Value to ANA for tx_hs_ofst_lvl */
		unsigned tx_hs_ofst_lvl:1;
		/* bit[15:7] */
		unsigned RSVD15_7:9;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_HS_ANA_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_HS_ANA_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_OUT_0	 0x0384
typedef union {
	u16 data;
	struct {
		/* bit[0] Value to ANA for rx_hs_en */
		unsigned rx_hs_en:1;
		/* bit[1] Value to ANA for rx_hs_squelch_en */
		unsigned rx_hs_squelch_en:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_IN_0	 0x0388
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ANA for rx_hs_data */
		unsigned rx_hs_data:1;
		/* bit[1] Value from ANA for rx_hs_squelch */
		unsigned rx_hs_squelch:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_ANA_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_SE_ANA_OUT_0	 0x038c
typedef union {
	u16 data;
	struct {
		/* bit[4:0] Value to ANA for tx_se_data_m */
		unsigned tx_se_data_m:5;
		/* bit[9:5] Value to ANA for tx_se_data_p */
		unsigned tx_se_data_p:5;
		/* bit[10] Value to ANA for tx_se_en_m */
		unsigned tx_se_en_m:1;
		/* bit[11] Value to ANA for tx_se_en_p */
		unsigned tx_se_en_p:1;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_SE_ANA_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_SE_ANA_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_ANA_OUT_0	 0x0390
typedef union {
	u16 data;
	struct {
		/* bit[0] Value to ANA for rx_se_en_m */
		unsigned rx_se_en_m:1;
		/* bit[1] Value to ANA for rx_se_en_p */
		unsigned rx_se_en_p:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_ANA_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_ANA_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_ANA_IN_0	 0x0394
typedef union {
	u16 data;
	struct {
		/* bit[0] Value from ANA for rx_se_data_m */
		unsigned rx_se_data_m:1;
		/* bit[1] Value from ANA for rx_se_data_p */
		unsigned rx_se_data_p:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_ANA_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_SE_ANA_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PULLDOWN_ANA_OUT_0	 0x0398
typedef union {
	u16 data;
	struct {
		/* bit[0] Value to ANA for pulldown_en_m */
		unsigned pulldown_en_m:1;
		/* bit[1] Value to ANA for pulldown_en_p */
		unsigned pulldown_en_p:1;
		/* bit[15:2] */
		unsigned RSVD15_2:14;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PULLDOWN_ANA_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PULLDOWN_ANA_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_OUT_0	 0x039c
typedef union {
	u16 data;
	struct {
		/* bit[0] Value to ANA for bias_pu */
		unsigned bias_pu:1;
		/* bit[1] Value to ANA for bias_chop_en */
		unsigned bias_chop_en:1;
		/* bit[2] Value to ANA for pll_pu */
		unsigned pll_pu:1;
		/* bit[3] Value to ANA for pll_reset */
		unsigned pll_reset:1;
		/* bit[4] Value to ANA for pll_gear_shift */
		unsigned pll_gear_shift:1;
		/* bit[5] Value to ANA for pll_force_lock */
		unsigned pll_force_lock:1;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_OUT_1	 0x03a0
typedef union {
	u16 data;
	struct {
		/* bit[3:0] Value to ANA for pll_phy_n */
		unsigned pll_phy_n:4;
		/* bit[9:4] Value to ANA for pll_pmix_frac */
		unsigned pll_pmix_frac:6;
		/* bit[11:10] Value to ANA for pll_pmix_int */
		unsigned pll_pmix_int:2;
		/* bit[12] Value to ANA for pll_pmix_frac_en */
		unsigned pll_pmix_frac_en:1;
		/* bit[15:13] */
		unsigned RSVD15_13:3;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_OUT_1_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_OUT_1_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_OUT_2	 0x03a4
typedef union {
	u16 data;
	struct {
		/* bit[11:0] Value to ANA for pll_phy_m */
		unsigned pll_phy_m:12;
		/* bit[15:12] */
		unsigned RSVD15_12:4;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_OUT_2_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_OUT_2_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_IN_0	 0x03a8
typedef union {
	u16 data;
	struct {
		/* bit[0] Value to ANA for pll_lock */
		unsigned pll_lock:1;
		/* bit[15:1] */
		unsigned RSVD15_1:15;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_ANA_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_ANA_OUT_0	 0x03ac
typedef union {
	u16 data;
	struct {
		/* bit[0] Value to ANA for tx_vref_vreg_pu */
		unsigned tx_vref_vreg_pu:1;
		/* bit[1] Value to ANA for tx_vreg1v_bypass */
		unsigned tx_vreg1v_bypass:1;
		/* bit[2] Value to ANA for tx_fsls_slew_rate_ctrl */
		unsigned tx_fsls_slew_rate_ctrl:1;
		/* bit[15:3] */
		unsigned RSVD15_3:13;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_ANA_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_TX_ANA_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_ANA_OUT_0	 0x03b0
typedef union {
	u16 data;
	struct {
		/* bit[3:0] Value to ANA for tx_sliver_en */
		unsigned tx_sliver_en:4;
		/* bit[4] Value to ANA for tx_rtune_en */
		unsigned tx_rtune_en:1;
		/* bit[5] Value to ANA for comp_clk */
		unsigned comp_clk:1;
		/* bit[9:6] Value to ANA for rtune_code */
		unsigned rtune_code:4;
		/* bit[15:10] */
		unsigned RSVD15_10:6;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_ANA_OUT_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_ANA_OUT_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_ANA_IN_0	 0x03b4
typedef union {
	u16 data;
	struct {
		/* bit[0] Value to ANA for comp_result */
		unsigned comp_result:1;
		/* bit[15:1] */
		unsigned RSVD15_1:15;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_ANA_IN_0_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RCAL_ANA_IN_0_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_MPLL_PRG_VAL_STATUS	 0x03b8
typedef union {
	u16 data;
	struct {
		/* bit[15:0] Value to ANA for pll_mpll_prg */
		unsigned pll_mpll_prg:16;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_MPLL_PRG_VAL_STATUS_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_PLL_MPLL_PRG_VAL_STATUS_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_CMN_SEL_VAL_STATUS	 0x03bc
typedef union {
	u16 data;
	struct {
		/* bit[5:0] Value to ANA for atb_cmn_sel */
		unsigned atb_cmn_sel:6;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_CMN_SEL_VAL_STATUS_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_CMN_SEL_VAL_STATUS_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_XCVR_SEL_VAL_STATUS	 0x03c0
typedef union {
	u16 data;
	struct {
		/* bit[5:0] Value to ANA for atb_xcvr_sel */
		unsigned atb_xcvr_sel:6;
		/* bit[15:6] */
		unsigned RSVD15_6:10;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_XCVR_SEL_VAL_STATUS_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_ATB_XCVR_SEL_VAL_STATUS_p;

#define EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_TERM_EN_STATUS	 0x03c4
typedef union {
	u16 data;
	struct {
		/* bit[0] Value to ANA for rx_hs_term_en */
		unsigned rx_hs_term_en:1;
	} b;
} EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_TERM_EN_STATUS_o, *EUSBPHY_REG_PHY_CR_XCVR0_DIG_ANA_RX_HS_TERM_EN_STATUS_p;

#endif
