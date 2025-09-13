/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#ifndef __DW3000_COMPAT_REG_H
#define __DW3000_COMPAT_REG_H

#define MAX_CHIP_VERSIONS 3

#ifdef DEFINE_COMPAT_REGISTERS
#define REG_TYPE unsigned int
#define REG_ADDR(n, ...) REG_TYPE n[MAX_CHIP_VERSIONS] = { __VA_ARGS__ }
#else
#define REG_TYPE extern unsigned int
#define REG_ADDR(n, ...) REG_TYPE n[MAX_CHIP_VERSIONS]
#endif
#define REG(n) n[__dw3000_chip_version]

REG_TYPE __dw3000_chip_version;

REG_ADDR(__tx_fctrl_id, 0x24, 0x20, 0x20);
REG_ADDR(__tx_fctrl_hi_id, 0x28, 0x24, 0x24);
REG_ADDR(__dx_time_id, 0x2c, 0x28, 0x28);
REG_ADDR(__rx_ttcko_lo_id, 0x5c, 0x58, 0x58);
REG_ADDR(__rx_ttcko_hi_id, 0x60, 0x5c, 0x5c);
REG_ADDR(__rx_time_0_id, 0x64, 0x60, 0x60);
REG_ADDR(__rx_time_1_id, 0x68, 0x64, 0x64);
REG_ADDR(__rx_time_2_id, 0x6c, 0x68, 0x68);
REG_ADDR(__rx_time_3_id, 0x70, 0x6c, 0x6c);
REG_ADDR(__tx_time_lo_id, 0x74, 0x70, 0x70);
REG_ADDR(__tx_time_hi_id, 0x78, 0x74, 0x74);
REG_ADDR(__tx_time_2_id, 0x10000, 0x78, 0x78);
REG_ADDR(__tx_antd_id, 0x10004, 0x7c, 0x7c);
REG_ADDR(__ack_resp_id, 0x10008, 0x10000, 0x10000);
REG_ADDR(__tx_power_id, 0x1000c, 0x10004, 0x10004);
REG_ADDR(__chan_ctrl_id, 0x10014, 0x10008, 0x10008);
REG_ADDR(__le_pend_01_id, 0x10018, 0x1000C, 0x1000C);
REG_ADDR(__le_pend_23_id, 0x1001c, 0x10010, 0x10010);
REG_ADDR(__spi_collision_status_id, 0x10020, 0x10014, 0x10014);
REG_ADDR(__rdb_status_id, 0x10024, 0x10018, 0x10018);
REG_ADDR(__rdb_diag_mode_id, 0x10028, 0x10020, 0x10020);
REG_ADDR(__regmap_ver_id, 0x1002c, 0x10024, 0x10024);
REG_ADDR(__sar_ctrl_sar_force_sel_bit_len, 3U, 4U, 4U);
REG_ADDR(__sar_ctrl_sar_force_sel_bit_mask, 0x7000U, 0xf000U, 0xf000U);
REG_ADDR(__nvm_cfg_gear_id_bit_offset, 11U, 12U, 12U);
REG_ADDR(__nvm_cfg_gear_id_bit_mask, 0x1800U, 0x3000U, 0x3000U);
REG_ADDR(__nvm_cfg_gear_kick_bit_offset, 10U, 0x11U, 0x11U);
REG_ADDR(__nvm_cfg_gear_kick_bit_mask, 0x400U, 0x800U, 0x800U);
REG_ADDR(__nvm_cfg_bias_kick_bit_offset, 8U, 10U, 10U);
REG_ADDR(__nvm_cfg_bias_kick_bit_mask, 0x100U, 0x400U, 0x400U);
REG_ADDR(__nvm_cfg_ldo_kick_bit_offset, 7U, 9U, 9U);
REG_ADDR(__nvm_cfg_ldo_kick_bit_mask, 0x80U, 0x200U, 0x200U);
REG_ADDR(__nvm_cfg_dgc_kick_bit_offset, 6U, 8U, 8U);
REG_ADDR(__nvm_cfg_dgc_kick_bit_mask, 0x40U, 0x100U, 0x100U);
REG_ADDR(__nvm_cfg_dgc_sel_bit_offset, 13U, 7U, 7U);
REG_ADDR(__nvm_cfg_dgc_sel_bit_mask, 0x2000U, 0x80U, 0x80U);
REG_ADDR(__nvm_cfg_nvm_pd_bit_offset, 9U, 6U, 6U);
REG_ADDR(__nvm_cfg_nvm_pd_bit_mask, 0x200U, 0x40U, 0x40U);
REG_ADDR(__ip_diag_1_ipchannelarea_bit_len, 17U, 19U, 19U);
REG_ADDR(__ip_diag_1_ipchannelarea_bit_mask, 0x1ffffUL, 0x7ffffUL, 0x7ffffUL);
REG_ADDR(__cy0_diag_1_cy0channelarea_bit_len, 16U, 19U, 19U);
REG_ADDR(__cy0_diag_1_cy0channelarea_bit_mask, 0xffffU, 0x7ffffU, 0x7ffffU);
REG_ADDR(__cy1_diag_1_cy1channelarea_bit_len, 16U, 19U, 19U);
REG_ADDR(__cy1_diag_1_cy1channelarea_bit_mask, 0xffffU, 0x7ffffU, 0x7ffffU);
REG_ADDR(__ip_config_hi_id, 0xe000e, 0xe0010, 0xe0010);
REG_ADDR(__cy_config_lo_id, 0xe0012, 0xe0014, 0xe0014);
REG_ADDR(__cy_config_hi_id, 0xe0016, 0xe0018, 0xe0018);
REG_ADDR(__cia_coefficient_adjust_id, 0xe001a, 0xe001c, 0xe001c);
REG_ADDR(__pgf_delay_comp_lo_id, 0xe001e, 0xe0020, 0xe0020);
REG_ADDR(__pgf_delay_comp_hi_id, 0xe0022, 0xe0024, 0xe0024);
REG_ADDR(__adc_mem_ptr_id, 0xf0020, 0xf0024, 0xf0024);
REG_ADDR(__test_ctrl0_id, 0xf0024, 0xf0028, 0xf0028);
REG_ADDR(__fcmd_status_id, 0xf003c, 0xf0040, 0xf0040);
REG_ADDR(__test_logging_id, 0xf0040, 0xf0044, 0xf0044);
REG_ADDR(__status_logging_id, 0xf0044, 0xf0048, 0xf0048);
REG_ADDR(__ctr_dbg_id, 0xf0048, 0xf004C, 0xf004C);
REG_ADDR(__led_ctrl_id, 0x110016, 0x110018, 0x110018);
REG_ADDR(__rx_ppm_id, 0x11001a, 0x11001c, 0x11001c);
REG_ADDR(__fosc_ctrl_id, 0x11001e, 0x110020, 0x110020);
REG_ADDR(__bias_ctrl_id, 0x11001f, 0x110030, 0x110030);
REG_ADDR(__bias_ctrl_dig_bias_ctrl_tc_r3_ulv_bit_offset, 12U, 9U, 9U);
REG_ADDR(__bias_ctrl_dig_bias_ctrl_tc_r3_ulv_bit_mask, 0x3000U, 0x600U, 0x600U);
/* LDO and BIAS tune kick */
/* C0 : Writing to bit 7 and 8 */
/* D0 OTP errata : only kick LDO not BIAS */
/* E0 : only kick LDO bit 9 (Writing to bit 10 for BIAS and 9 for LDO) */
REG_ADDR(__ldo_bias_kick, 0x180, 0x200, 0x600);

/* E0 specific */
REG_ADDR(__dgc_lut_0_cfg_id, 0x30038, 0x30038, 0x3002c);
REG_ADDR(__dgc_lut_1_cfg_id, 0x3003c, 0x3003c, 0x30030);
REG_ADDR(__dgc_lut_2_cfg_id, 0x30040, 0x30040, 0x30034);
REG_ADDR(__dgc_lut_3_cfg_id, 0x30044, 0x30044, 0x30038);
REG_ADDR(__dgc_lut_4_cfg_id, 0x30048, 0x30048, 0x3003c);
REG_ADDR(__dgc_lut_5_cfg_id, 0x3004c, 0x3004c, 0x30040);
REG_ADDR(__dgc_lut_6_cfg_id, 0x30050, 0x30050, 0x30044);
REG_ADDR(__dgc_dbg_id, 0x30060, 0x30060, 0x30054);
REG_ADDR(__cia_tdoa_0_tdoa_bit_len, 32U, 32U, 16U);
REG_ADDR(__cia_tdoa_0_tdoa_bit_mask, 0xffffffffUL, 0xffffffffUL, 0xffffU);

#define DW3000_TX_FCTRL_ID REG(__tx_fctrl_id)
#define DW3000_TX_FCTRL_HI_ID REG(__tx_fctrl_hi_id)
#define DW3000_DX_TIME_ID REG(__dx_time_id)
#define DW3000_RX_TTCKO_LO_ID REG(__rx_ttcko_lo_id)
#define DW3000_RX_TTCKO_HI_ID REG(__rx_ttcko_hi_id)
#define DW3000_RX_TIME_0_ID REG(__rx_time_0_id)
#define DW3000_RX_TIME_1_ID REG(__rx_time_1_id)
#define DW3000_RX_TIME_2_ID REG(__rx_time_2_id)
#define DW3000_RX_TIME_3_ID REG(__rx_time_3_id)
#define DW3000_TX_TIME_LO_ID REG(__tx_time_lo_id)
#define DW3000_TX_TIME_HI_ID REG(__tx_time_hi_id)
#define DW3000_TX_TIME_2_ID REG(__tx_time_2_id)
#define DW3000_TX_ANTD_ID REG(__tx_antd_id)
#define DW3000_ACK_RESP_ID REG(__ack_resp_id)
#define DW3000_TX_POWER_ID REG(__tx_power_id)
#define DW3000_CHAN_CTRL_ID REG(__chan_ctrl_id)
#define DW3000_LE_PEND_01_ID REG(__le_pend_01_id)
#define DW3000_LE_PEND_23_ID REG(__le_pend_23_id)
#define DW3000_SPI_COLLISION_STATUS_ID REG(__spi_collision_status_id)
#define DW3000_RDB_STATUS_ID REG(__rdb_status_id)
#define DW3000_RDB_DIAG_MODE_ID REG(__rdb_diag_mode_id)
#define DW3000_REGMAP_VER_ID REG(__regmap_ver_id)
#define DW3000_SAR_CTRL_SAR_FORCE_SEL_BIT_LEN \
	REG(__sar_ctrl_sar_force_sel_bit_len)
#define DW3000_SAR_CTRL_SAR_FORCE_SEL_BIT_MASK \
	REG(__sar_ctrl_sar_force_sel_bit_mask)
#define DW3000_NVM_CFG_GEAR_ID_BIT_OFFSET REG(__nvm_cfg_gear_id_bit_offset)
#define DW3000_NVM_CFG_GEAR_ID_BIT_MASK REG(__nvm_cfg_gear_id_bit_mask)
#define DW3000_NVM_CFG_GEAR_KICK_BIT_OFFSET REG(__nvm_cfg_gear_kick_bit_offset)
#define DW3000_NVM_CFG_GEAR_KICK_BIT_MASK REG(__nvm_cfg_gear_kick_bit_mask)
#define DW3000_NVM_CFG_BIAS_KICK_BIT_OFFSET REG(__nvm_cfg_bias_kick_bit_offset)
#define DW3000_NVM_CFG_BIAS_KICK_BIT_MASK REG(__nvm_cfg_bias_kick_bit_mask)
#define DW3000_NVM_CFG_LDO_KICK_BIT_OFFSET REG(__nvm_cfg_ldo_kick_bit_offset)
#define DW3000_NVM_CFG_LDO_KICK_BIT_MASK REG(__nvm_cfg_ldo_kick_bit_mask)
#define DW3000_NVM_CFG_DGC_KICK_BIT_OFFSET REG(__nvm_cfg_dgc_kick_bit_offset)
#define DW3000_NVM_CFG_DGC_KICK_BIT_MASK REG(__nvm_cfg_dgc_kick_bit_mask)
#define DW3000_NVM_CFG_DGC_SEL_BIT_OFFSET REG(__nvm_cfg_dgc_sel_bit_offset)
#define DW3000_NVM_CFG_DGC_SEL_BIT_MASK REG(__nvm_cfg_dgc_sel_bit_mask)
#define DW3000_NVM_CFG_NVM_PD_BIT_OFFSET REG(__nvm_cfg_nvm_pd_bit_offset)
#define DW3000_NVM_CFG_NVM_PD_BIT_MASK REG(__nvm_cfg_nvm_pd_bit_mask)
#define DW3000_IP_DIAG_1_IPCHANNELAREA_BIT_LEN \
	REG(__ip_diag_1_ipchannelarea_bit_len)
#define DW3000_IP_DIAG_1_IPCHANNELAREA_BIT_MASK \
	REG(__ip_diag_1_ipchannelarea_bit_len)
#define DW3000_CY0_DIAG_1_CY0CHANNELAREA_BIT_LEN \
	REG(__cy0_diag_1_cy0channelarea_bit_len)
#define DW3000_CY0_DIAG_1_CY0CHANNELAREA_BIT_MASK \
	REG(__cy0_diag_1_cy0channelarea_bit_mask)
#define DW3000_CY1_DIAG_1_CY1CHANNELAREA_BIT_LEN \
	REG(__cy1_diag_1_cy1channelarea_bit_len)
#define DW3000_CY1_DIAG_1_CY1CHANNELAREA_BIT_MASK \
	REG(__cy1_diag_1_cy1channelarea_bit_mask)
#define DW3000_IP_CONFIG_HI_ID REG(__ip_config_hi_id)
#define DW3000_CY_CONFIG_LO_ID REG(__cy_config_lo_id)
#define DW3000_CY_CONFIG_HI_ID REG(__cy_config_hi_id)
#define DW3000_CIA_COEFFICIENT_ADJUST_ID REG(__cia_coefficient_adjust_id)
#define DW3000_PGF_DELAY_COMP_LO_ID REG(__pgf_delay_comp_lo_id)
#define DW3000_PGF_DELAY_COMP_HI_ID REG(__pgf_delay_comp_hi_id)
#define DW3000_ADC_MEM_PTR_ID REG(__adc_mem_ptr_id)
#define DW3000_TEST_CTRL0_ID REG(__test_ctrl0_id)
#define DW3000_FCMD_STATUS_ID REG(__fcmd_status_id)
#define DW3000_TEST_LOGGING_ID REG(__test_logging_id)
#define DW3000_STATUS_LOGGING_ID REG(__status_logging_id)
#define DW3000_CTR_DBG_ID REG(__ctr_dbg_id)
#define DW3000_LED_CTRL_ID REG(__led_ctrl_id)
#define DW3000_RX_PPM_ID REG(__rx_ppm_id)
#define DW3000_FOSC_CTRL_ID REG(__fosc_ctrl_id)
#define DW3000_BIAS_CTRL_ID REG(__bias_ctrl_id)
#define DW3000_BIAS_CTRL_DIG_BIAS_CTRL_TC_R3_ULV_BIT_OFFSET \
	REG(__bias_ctrl_dig_bias_ctrl_tc_r3_ulv_bit_offset)
#define DW3000_BIAS_CTRL_DIG_BIAS_CTRL_TC_R3_ULV_BIT_MASK \
	REG(__bias_ctrl_dig_bias_ctrl_tc_r3_ulv_bit_mask)
#define DW3000_LDO_BIAS_KICK REG(__ldo_bias_kick)

/* E0 specific */
#define DW3000_DGC_LUT_0_CFG_ID REG(__dgc_lut_0_cfg_id)
#define DW3000_DGC_LUT_1_CFG_ID REG(__dgc_lut_1_cfg_id)
#define DW3000_DGC_LUT_2_CFG_ID REG(__dgc_lut_2_cfg_id)
#define DW3000_DGC_LUT_3_CFG_ID REG(__dgc_lut_3_cfg_id)
#define DW3000_DGC_LUT_4_CFG_ID REG(__dgc_lut_4_cfg_id)
#define DW3000_DGC_LUT_5_CFG_ID REG(__dgc_lut_5_cfg_id)
#define DW3000_DGC_LUT_6_CFG_ID REG(__dgc_lut_6_cfg_id)
#define DW3000_DGC_DBG_ID REG(__dgc_dbg_id)
#define DW3000_CIA_TDOA_0_TDOA_BIT_LEN REG(__cia_tdoa_0_tdoa_bit_len)
#define DW3000_CIA_TDOA_0_TDOA_BIT_MASK REG(__cia_tdoa_0_tdoa_bit_mask)

#endif /* __DW3000_COMPAT_REG_H */
