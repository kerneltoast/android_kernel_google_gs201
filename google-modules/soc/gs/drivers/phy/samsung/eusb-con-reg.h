/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung EXYNOS SoC series USB PHY driver
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef _USB_USBPHY_CAL_EUSB_CON_REG_H_
#define _USB_USBPHY_CAL_EUSB_CON_REG_H_

#define EUSBCON_REG_RST_CTRL	0x0000

/* Offset : 0x0000
 * Description: Control eUSB PHY Reset pins
 */
typedef union {
	u32 data;
	struct {
		// bit[0] :
		unsigned phy_reset : 1;
		// bit[1] :
		unsigned phy_reset_ovrd_en : 1;
		// bit[3:2]
		unsigned RSVD3_2 : 2;
		// bit[4] :
		unsigned utmi_port_reset : 1;
		// bit[5] :
		unsigned utmi_port_reset_ovrd_en : 1;
		// bit[31:6]
		unsigned RSVD31_6 : 26;
	} b;
} EUSBCON_REG_RST_CTRL_o, *EUSBPHY_REG_RST_CTRL_p;

#define EUSBCON_REG_CMN_CTRL	0x0004

/* Offset : 0x0004
 * Description: common block control signals
 */
typedef union {
	u32 data;
	struct {
		// bit[0] :
		unsigned phy_enable : 1;
		// bit[1] :
		unsigned retenable_n : 1;
		// bit[3:2]
		unsigned RSVD3_2 : 2;
		// bit[6:4] :
		unsigned ref_freq_sel : 3;
		// bit[7]
		unsigned RSVD7 : 1;
		// bit[8] :
		unsigned phy_cfg_cr_clk_sel : 1;
		// bit[9] :
		unsigned phy_cfg_port_in_lx : 1;
		// bit[10] :
		unsigned phy_cfg_rptr_mode : 1;
		// bit[31:11]
		unsigned RSVD31_11 : 21;
	} b;
} EUSBCON_REG_CMN_CTRL_o, *EUSBPHY_REG_CMN_CTRL_p;

#define EUSBCON_REG_PLLCFG0	0x0008

/* Offset : 0x0008
 * Description: configure phy pll
 */
typedef union {
	u32 data;
	struct {
		// bit[6:0] :
		unsigned pll_cpbias_cntrl : 7;
		// bit[7] :
		unsigned RSVD7 : 1;
		// bit[19:8] :
		unsigned pll_fb_div : 12;
		// bit[21:20]
		unsigned pll_gmp_cntrl : 2;
		// bit[23:22] :
		unsigned RSVD23_22 : 2;
		// bit[29:24] :
		unsigned pll_int_cntrl : 6;
		// bit[31:30]
		unsigned RSVD31_30 : 2;
	} b;
} EUSBCON_REG_PLLCFG0_o, *EUSBPHY_REG_PLLCFG0_p;

#define EUSBCON_REG_PLLCFG1	0x000C

/* Offset : 0x000c
 * Description: configure phy pll
 */
typedef union {
	u32 data;
	struct {
		// bit[5:0] :
		unsigned pll_prop_cntrl : 6;
		// bit[7:6] :
		unsigned RSVD7_6 : 2;
		// bit[11:8] :
		unsigned pll_ref_div : 4;
		// bit[14:12]
		unsigned pll_vco_cntrl : 3;
		// bit[15] :
		unsigned RSVD15 : 1;
		// bit[17:16] :
		unsigned pll_vref_tune : 2;
		// bit[31:18]
		unsigned RSVD31_30 : 14;
	} b;
} EUSBCON_REG_PLLCFG1_o, *EUSBPHY_REG_PLLCFG1_p;

#define EUSBCON_REG_RCAL	0x0010

/* Offset : 0x0010
 * Description: control external register calibration(rcal)
 */
typedef union {
	u32 data;
	struct {
		// bit[0] :
		unsigned rcal_bypass : 1;
		// bit[3:1] :
		unsigned RSVD3_1 : 3;
		// bit[7:4] :
		unsigned rcal_code : 4;
		// bit[11:8]
		unsigned rcal_offset : 4;
		// bit[31:12]
		unsigned RSVD31_12 : 20;
	} b;
} EUSBCON_REG_RCAL_o, *EUSBPHY_REG_RCAL_p;

#define EUSBCON_REG_TXTUNE	0x0014

/* Offset : 0x0014
 * Description: tune register of tx driver
 */
typedef union {
	u32 data;
	struct {
		// bit[0] :
		unsigned fsls_slew_rate : 1;
		// bit[2:1] :
		unsigned fsls_vref_tune : 2;
		// bit[3] :
		unsigned fsls_vreg_bypass : 1;
		// bit[6:4]
		unsigned hs_vref_tune : 3;
		// bit [8:7]
		unsigned hs_xv : 2;
		// bit [11:9]
		unsigned preemp : 3;
		// bit [13:12]
		unsigned res : 2;
		// bit [15:14]
		unsigned rise : 2;
		// bit[31:16]
		unsigned RSVD31_16 : 16;
	} b;
} EUSBCON_REG_TXTUNE_o, *EUSBPHY_REG_TXTUNE_p;

#define EUSBCON_REG_RXTUNE	0x0018

/* Offset : 0x0018
 * Description: tune register of rx driver
 */
typedef union {
	u32 data;
	struct {
		// bit[1:0] :
		unsigned eq_ctle : 2;
		// bit[3:2] :
		unsigned RSVD3_2 : 2;
		// bit[3] :
		unsigned hs_term_en : 1;
		// bit[7:5]
		unsigned RSVD7_5 : 3;
		// bit [10:8]
		unsigned hs_tune : 3;
		// bit[31:11]
		unsigned RSVD31_16 : 21;
	} b;
} EUSBCON_REG_RXTUNE_o, *EUSBPHY_REG_RXTUNE_p;

#define EUSBCON_REG_UTMI	0x001C

/* Offset : 0x001C
 * Description: control utmi interface signal
 */
typedef union {
	u32 data;
	struct {
		// bit[0] :
		unsigned clk_force_en : 1;
		// bit[1] :
		unsigned dm_pulldown : 1;
		// bit[2] :
		unsigned dm_pulldown_ovrd_en : 1;
		// bit[3] :
		unsigned dp_pulldown : 1;
		// bit[4] :
		unsigned dp_pulldown_ovrd_en : 1;
		// bit[5] :
		unsigned sleep_n : 1;
		// bit[6] :
		unsigned sleep_n_ovrd_en : 1;
		// bit[7] :
		unsigned suspend_n : 1;
		// bit[8] :
		unsigned suspend_n_ovrd_en : 1;
		// bit[9] :
		unsigned txbitstuffen : 1;
		// bit[10] :
		unsigned txbitstuffen_ovrd_en : 1;
		// bit[11] :
		unsigned vbus_valid_ext : 1;
		// bit[12] :
		unsigned vbus_valid_ext_ovrd_en : 1;
		// bit[31:13]
		unsigned RSVD31_13 : 17;
	} b;
} EUSBCON_REG_UTMI_o, *EUSBPHY_REG_UTMI_p;

#define EUSBCON_REG_TESTSE	0x0020

/* Offset : 0x0020
 * Description: control test pin
 */
typedef union {
	u32 data;
	struct {
		// bit[0] :
		unsigned test_loopback_en : 1;
		// bit[1] :
		unsigned test_stop_clk_en : 1;
		// bit[2] :
		unsigned tx_se_dp_en : 1;
		// bit[3] :
		unsigned tx_se_dm_en : 1;
		// bit[4] :
		unsigned tx_dig_bypass_sel : 1;
		// bit[5] :
		unsigned tx_se_test_en : 1;
		// bit[6] :
		unsigned test_iddq : 1;
		// bit[7] :
		unsigned mon_phy_rx_se_dp : 1;
		// bit[8] :
		unsigned mon_phy_rx_se_dm : 1;
		// bit[31:9]
		unsigned RSVD31_13 : 23;
	} b;
} EUSBCON_REG_TESTSE_o, *EUSBPHY_REG_TESTSE_p;

#endif
