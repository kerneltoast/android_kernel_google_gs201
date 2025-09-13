/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung EXYNOS SoC series USB PHY driver
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef _SNPS_USBDP_TCA_REG_H_
#define _SNPS_USBDP_TCA_REG_H_

#define SNPS_USBDPPHY_TCA_TCA_CLK_RST    0x0000
typedef union {
	u32 data;
	struct {
		/* bit[0] Enable for suspend_clk as the tca_clk.
		 *  - 1'b0: PHY ref_clk_sel decides the tca_clk. If ref_clk_sel is 1'b1,
		 * ref_clk from PHY is the tca_clk, else suspend_clk is the tca_clk.
		 *  - 1'b1: suspend_clk is used as the tca_clk. */
		unsigned suspend_clk_en:1;
		/* bit[1] tca_ref_clk_en to the DPAlt_X-Bar , enables ref_dig_clk generation
		 * even though [dp,ss]_ref_clk_en might be low.
		 *  - 1'b0: No request from TCA for ref_clk. ref_clk is is enabled
		 * based on request from DP or SS+ protocol layer.
		 *  - 1'b1: Request from TCA for ref_clk to be enabled.
		 * Note :- The final ref_clk_en to PMA is OR of tca_ref_clk_en,
		 * and dp_ref_clk_en, ss_ref_clk_en */
		unsigned tca_ref_clk_en:1;
		/* bit[2] Enable automatic or dynamic clock gating of tca_clk when
		 * TCPC requests are processed for power saving
		 *  - 1'b0: tca_clk is always enabled.
		 *  - 1'b1: tca_clk is dynamically gated when there is no TCPC request
		 * pending to be completed. tca_clk is enabled when a new TCPC request comes. */
		unsigned tca_clk_auto_gate_en:1;
		/* bit[7:3] */
		unsigned RSVD7_3:5;
		/* bit[8] phy_reset SW override.
		 *  - 1'b0: No phy_reset override
		 *  - 1'b1: phy_reset is asserted (override on phy_reset coming from
		 * ComboPHY top) to UPCS,PMA, and PCS_RAW. A Soft tca_phy_reset can
		 * be driven by asserting this Bit for the required duration. */
		unsigned phy_rst_sw:1;
		/* bit[9] XBar_Assist Soft Reset
		 *  - 1'b0: Soft reset is applied. Active low Soft Reset to X-bar Assist,
		 * pulse low to apply the reset.
		 *  - 1'b1: Soft reset is not applied */
		unsigned xa_rst_sw:1;
		/* bit[10] VBUS Assist Soft Reset
		 *  - 1'b0: Soft reset is applied. Active low Soft Reset to VBUS Assist,
		 * pulse low to apply the reset.
		 *  - 1'b1: Soft reset is not applied */
		unsigned vba_rst_sw:1;
		/* bit[31:11] */
		unsigned RSVD31_11:21;
	} b;
} SNPS_USBDPPHY_TCA_TCA_CLK_RST_o, *SNPS_USBDPPHY_TCA_TCA_CLK_RST_p;

#define SNPS_USBDPPHY_TCA_TCA_INTR_EN    0x0004
typedef union {
	u32 data;
	struct {
		/* bit[0] XBar_Assist ack event enable
		 *  - 1'b0: Ack event is disabled
		 *  - 1'b1: Ack event is enabled
		 * When enabled, tca_int interrupt is generated for XBar_Assist TCPC
		 * request completion requested through write to TCA_TCPC register
		 * for desired ComboPHY configuration. */
		unsigned xa_ack_evt_en:1;
		/* bit[1] XBar_Assist timeout event enable
		 *  - 1'b0: Timeout event is disabled
		 *  - 1'b1: Timeout event is enabled
		 * When enabled, tca_int interrupt is generated on timeout of the TCPC
		 * request which was requested through write to TCA_TCPC register. */
		unsigned xa_timeout_evt_en:1;
		/* bit[7:2] */
		unsigned RSVD7_2:6;
		/* bit[8] sys_vbusvalid change event enable
		 *  - 1'b0: Change event is disabled
		 *  - 1'b1: Change event is enabled
		 * When enabled, tca_int interrupt is generated when
		 * sys_vbusvalid input to TCA changes. */
		unsigned sys_vbusvalid_evt_en:1;
		/* bit[9] */
		unsigned RSVD9:1;
		/* bit[10] tca_vbusvalid change event enable
		 *  - 1'b0: Change event is disabled
		 *  - 1'b1: Change event is enabled
		 * When enabled, tca_int interrupt is generated when
		 * tca_vbusvalid output from TCA changes. */
		unsigned tca_vbusvalid_evt_en:1;
		/* bit[11] */
		unsigned RSVD11:1;
		/* bit[12] tca_drv_host_vbus change event enable
		 *  - 1'b0: Change event is disabled
		 *  - 1'b1: Change event is enabled
		 * When enabled, tca_int interrupt is generated when
		 * tca_drv_host_vbus output from TCA changes. */
		unsigned tca_drv_host_vbus_evt_en:1;
		/* bit[15:13] */
		unsigned RSVD15_13:3;
		/* bit[16] ss_lane1_active change event enable
		 *  - 1'b0: Change event is disabled
		 *  - 1'b1: Change event is enabled
		 * ss_lane1_active TCA input changes. */
		unsigned ss_lane1_active_evt_en:1;
		/* bit[31:17] */
		unsigned RSVD31_17:15;
	} b;
} SNPS_USBDPPHY_TCA_TCA_INTR_EN_o, *SNPS_USBDPPHY_TCA_TCA_INTR_EN_p;

#define SNPS_USBDPPHY_TCA_TCA_INTR_STS   0x0008
typedef union {
	u32 data;
	struct {
		/* bit[0] XBar_Assist ack event status
		 *  - 1'b0: Ack event has not occurred (or been cleared when 1'b1 is written)
		 *  - 1'b1: Ack event has occurred with tca_int interrupt asserted,
		 * indicating XBar_Assist TCPC request completion.
		 * Note: This bit is set by the hardware and can be cleared by writing 1'b1 on it */
		unsigned xa_ack_evt:1;
		/* bit[1] XBar_Assist timeout event status
		 *  - 1'b0: Timeout event has not occurred (or been cleared when 1'b1 is written)
		 *  - 1'b1: Timeout event has occurred with tca_int interrup asserted,
		 * indicating timeout of the TCPC request.
		 * Note: This bit is set by the hardware and can be cleared by writing 1'b1 on it */
		unsigned xa_timeout_evt:1;
		/* bit[7:2] */
		unsigned RSVD7_2:6;
		/* bit[8] sys_vbusvalid change event status
		 *  - 1'b0: Change event has not occurred (or been cleared when 1'b1 is written)
		 *  - 1'b1: Change event has occurred with tca_int interrupt asserted,
		 * indicating sys_vbusvalid change.
		 * Note: This bit is set by the hardware and can be cleared by writing 1'b1 on it */
		unsigned sys_vbusvalid_evt:1;
		/* bit[9] sys_vbusvalid status/value as seen on TCA input. */
		unsigned sys_vbusvalid:1;
		/* bit[10] tca_vbusvalid change event status
		 *  - 1'b0: Change event has not occurred (or been cleared when 1'b1 is written)
		 *  - 1'b1: Change event has occurred with tca_int interrupt asserted,
		 * indicating tca_vbusvalid change.
		 * Note: This bit is set by the hardware and can be cleared by writing 1'b1 on it */
		unsigned tca_vbusvalid_evt:1;
		/* bit[11] tca_vbusvalid status/value as seen on TCA output. */
		unsigned tca_vbusvalid:1;
		/* bit[12] tca_drv_host_vbus change event status
		 *  - 1'b0: Change event has not occurred (or been cleared when 1'b1 is written)
		 *  - 1'b1: Change event has occurred with tca_int interrupt asserted,
		 * indicating tca_drv_host_vbus change.
		 * Note: This bit is set by the hardware and can be cleared by writing 1'b1 on it */
		unsigned tca_drv_host_vbus_evt:1;
		/* bit[13] tca_ drv_host_vbus status/value as seen on TCA output. */
		unsigned tca_drv_host_vbus:1;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
		/* bit[16] ss_lane1_active input change event status
		 *  - 1'b0: Change event has not occurred (or been cleared when 1'b1 is written)
		 *  - 1'b1: Change event has occurred with tca_int interrupt asserted,
		 * indicating ss_lane1_active input change.
		 * Note: This bit is set by the hardware and can be cleared by writing 1'b1 on it */
		unsigned ss_lane1_active_evt:1;
		/* bit[17] ss_lane1_active status/value as seen on TCA input. */
		unsigned ss_lane1_active:1;
		/* bit[31:18] */
		unsigned RSVD31_18:14;
	} b;
} SNPS_USBDPPHY_TCA_TCA_INTR_STS_o, *SNPS_USBDPPHY_TCA_TCA_INTR_STS_p;

#define SNPS_USBDPPHY_TCA_TCA_GCFG   0x0010
typedef union {
	u32 data;
	struct {
		/* bit[1:0] DPAlt Crossbar Operational Mode
		 *  - 2'bx0: System Configuration Mode, use TCA_SYSMODE_CFG Register for
		 * DPAlt_Xbar direct control. The SW layers explicitly need to synchronize
		 * DP and USB controller to have all PHY lanes in P3 for
		 * flip/conn_mode[1:0]/disable operation.
		 *  - 2'bx1: Controller Synced Mode, use TCA_TCPC Register for DPAlt_Xbar
		 * control. The default operational mode where the XBar Assist synchronizes
		 * between DP and USB Controller to achieve desired P3 state
		 * requirement on all the lanes of the PHY and apply
		 * the changes as requested through the TCA_TCPC Register. */
		unsigned op_mode:2;
		/* bit[3:2] */
		unsigned RSVD3_2:2;
		/* bit[4] USB Host/Device Mode
		 *  - 1'b0: Default mode, USB Subsystem is in DRP/Host mode.
		 *  - 1'b1: Device mode operation, USB Subsystem is in Device mode
		 * This is needed as for the Device mode of operation, the Xbar_Assist
		 * relies on VBUS Valid (instead of ss_rxdetect_disable) for the
		 * flip/conn_mode switch operation.
		 * The system should set this to 1'b1 when USB Subsystem is in Device mode. */
		unsigned role_hstdev:1;
		/* bit[7:5] */
		unsigned RSVD7_5:3;
		/* bit[8] Reset enable for PHY safe state.
		 *  - 1'b0: Disables reset assertion during typec disabled state
		 * or PHY safestate. (Default value)
		 *  - 1'b1: Enables reset assertion during typec disabled state
		 * or PHY safestate. */
		unsigned physafe_reset_en:1;
		/* bit[31:9] */
		unsigned RSVD31_9:23;
	} b;
} SNPS_USBDPPHY_TCA_TCA_GCFG_o, *SNPS_USBDPPHY_TCA_TCA_GCFG_p;

#define SNPS_USBDPPHY_TCA_TCA_TCPC   0x0014
typedef union {
	u32 data;
	struct {
		/* bit[1:0] Mux control from TCPM controlling the behavior of the
		 * ComboPHY DPAlt_Xbar and TCA synchronization.
		 *  - 2'b00: No connection (default)
		 *  - 2'b01: USB3.1 Connected
		 *  - 2'b10: DP Alternate Mode - 4 lanes
		 *  - 2'b11: USB3.1 + Display Port Lanes 0 and 1 */
		unsigned tcpc_mux_control:2;
		/* bit[2] Connector orientation from TCPM
		 *  - 1'b0: Normal (CC1=A5, CC2=B5, TX1=A2/A3, RX1=B10/B11), default
		 *  - 1'b1: Flipped (CC2=A5, CC1=B5, TX1=B2/B3, RX1=A10/A11)
		 * Note: The selection is swapped when typec_flip_invert input to the ComboPHY is 1'b1. */
		unsigned tcpc_connector_orientation:1;
		/* bit[3] Control from TCPM to put the PHY in lowpower/Safe state.
		 *  - 1'b0: Standard Operation
		 *  - 1'b1: Disable PHY and use advanced low power strategy (P3.Disable
		 * on all PHY lanes with even lower power consumtion than P3).
		 * Also puts all the PHY lanes in USB Safe state. */
		unsigned tcpc_low_power_en:1;
		/* bit[4] When written as 1'b1, indicates to the XBar_Assist that other 3 fields
		 * in this register are valid and the operation requested needs to be
		 * performed by the XBar_Assist. This bit is auto cleared by the
		 * XBar_Assist, when XBar_Assist has read the required request. */
		unsigned tcpc_valid:1;
		/* bit[31:5] */
		unsigned RSVD31_5:27;
	} b;
} SNPS_USBDPPHY_TCA_TCA_TCPC_o, *SNPS_USBDPPHY_TCA_TCA_TCPC_p;

#define SNPS_USBDPPHY_TCA_TCA_SYSMODE_CFG    0x0018
typedef union {
	u32 data;
	struct {
		/* bit[1:0] Drives the typec_conn_mode[1:0] input of the XBAR. Selects the
		 * DPAlt_XBar config based on the pin configuration requested to
		 * support the desired DisplayPort Alternate Mode [DPALT].
		 *  - 2'b00: Reserved
		 *  - 2'b01: USB-Only Pin Configuration
		 *  - 2'b10: DPALT Pin configuration C,E
		 *  - 2'b11: DPALT Pin configuration D,F */
		unsigned typec_conn_mode:2;
		/* bit[2] Drives the typec_flip input of the XBAR. Selects the DPAlt_XBar
		 * config based on the flip requested [TYPE-C].
		 *  - 1'b0: Normal (CC1=A5, CC2=B5, TX1=A2/A3, RX1=B10/B11), default
		 *  - 1'b1: Flipped (CC2=A5, CC1=B5, TX1=B2/B3, RX1=A10/A11)
		 * Note: The selection is swapped when typec_flip_invert input to the ComboPHY is 1'b1. */
		unsigned typec_flip:1;
		/* bit[3] Drives the typec_disable (or typec_disconnect) input of the XBAR.
		 * This is driven high when all the lanes are in P3 to achieve LowPower
		 * operation P3.Disable USB Safe state of ComboPHY serial IOs
		 *  - 1'b0: Standard Operation
		 *  - 1'b1: lowpower USB Safe State operation */
		unsigned typec_disable:1;
		/* bit[4] Drives the typec_physafe input of the XBAR. This is driven high when
		 * all lanes are in P3 or P2 to achieve USB safe state of
		 * Type-C PHY serial IOs.
		 *  - 1'b0: Standard Operation
		 *  - 1'b1: USB Safe State operation */
		unsigned typec_physafe:1;
		/* bit[5] Drives the typec_reset input of the XBAR. This drives all the
		 * lane resets of the PHY lanes. */
		unsigned typec_reset:1;
		/* bit[31:6] */
		unsigned RSVD31_6:26;
	} b;
} SNPS_USBDPPHY_TCA_TCA_SYSMODE_CFG_o, *SNPS_USBDPPHY_TCA_TCA_SYSMODE_CFG_p;

#define SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_CFG0  0x0020
typedef union {
	u32 data;
	struct {
		/* bit[0] When 1'b1, Drives the block_ss_op input to DPAlt_XBar as 1'b1.
		 *  - 1'b0: Normal Mode, SS+ operation is not blocked
		 *  - 1'b1: SS+ operation is immediately blocked to emulate an offchip switch. */
		unsigned block_ss_op:1;
		/* bit[2:1] When transitioning out of USB modes, selects if SS+ operation
		 * has to be automatically blocked.
		 * It helps to emulate behavior similar to external switch by immediately
		 * killing the USB link and LTSSM reaching RxDetect
		 *  -  When bit [0] is 1'b1: Enable automatic assertion of block_ss_op when
		 * transitioning out of tcpc_mux_control[1:0] USB mode.
		 *  -  When bit [1] is 1'b1: Enable automatic assertion of block_ss_op when
		 * transitioning out of tcpc_mux_control[1:0] USB_DP2 mode. */
		unsigned auto_block_ss_op_en:2;
		/* bit[3] When transitioning out of USB mode, selects if sstx_detrx_req to PHY
		 * has to be blocked along with the ss_rxdet_disable request to controller.
		 *  - 1'b0: Does not block sstx_detrx_req to PHY. Relies on controller to
		 * disable RxDetect when ss_rxdet_disable is asserted.
		 *  - 1'b1: Block sstx_detrx_req to PHY when ss_rxdet_disable to
		 * controller is asserted. */
		unsigned sstx_detrx_req_block_en:1;
		/* bit[7:4] */
		unsigned RSVD7_4:4;
		/* bit[8] Handshake mechanism with USB Controller
		 *  - 1'b0: relies on USB Tx/Rx lanes reaching P3
		 *  - 1'b1: relies on ss_rxdet_disable_ack handshake along with
		 * USB Tx/Rx lanes reaching P3 */
		unsigned ss_hdshk_req:1;
		/* bit[9] Handshake mechanism with DP Controller
		 *  - 1'b0: relies on all DP Operational Tx lanes reaching P3
		 *  - 1'b1: relies on dpalt_disable_ack handshake along with all
		 * DP Operational Tx lanes reaching P3 */
		unsigned dp_hdshk_req:1;
		/* bit[10] Enable TypeC_MUX switching in P2 state for USB.
		 *  - 1'b0: Switching is allowed only when P3 state is achieved
		 *  - 1'b1: Switching is done when P2 state is achieved. */
		unsigned usb_p2_switch_en:1;
		/* bit[11] Enable TypeC_MUX switching in P2 state for DP.
		 *  - 1'b0: DP lane switching is allowed only when P3 state is achieved
		 *  - 1'b1: DP lane switching is done when P2 state is achieved. */
		unsigned dp_p2_switch_en:1;
		/* bit[12] Enable lane ack check bypass during TCA USB lane switching requests
		 *  - 1'b0: TCA checks for tx/rx_ack de-assertion
		 *  - 1'b1: TCA does not check for tx/rx_ack de-assertion */
		unsigned usb_ack_check_bypass_en:1;
		/* bit[13] Enable lane ack check bypass during TCA DP lane switching requests
		 *  - 1'b0: TCA checks for tx/rx_ack de-assertion
		 *  - 1'b1: TCA does not check for tx/rx_ack de-assertion */
		unsigned dp_ack_check_bypass_en:1;
		/* bit[15:14] */
		unsigned RSVD15_14:2;
		/* bit[16] Enable automatic transient safe mode (by assertion of typec_disable
		 * on DPAlt_XBar) when changing DPAlt_XBar configuration (flip/conn_mode[1:0])
		 *  - 1'b0: Transient Safe Mode is not used.
		 *  - 1'b1: Transient Safe Mode is used asserting typec_disable before
		 * the flip/conn_mode[1:0] change; typec_disable is deasserted
		 * after the configuration change.
		 * Note: This bit is only valid when during a TCPC request FLD_LOW_POWER_EN is 1'b0. */
		unsigned auto_safe_state:1;
		/* bit[31:17] */
		unsigned RSVD31_17:15;
	} b;
} SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_CFG0_o, *SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_CFG0_p;

#define SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_CFG1  0x0024
typedef union {
	u32 data;
	struct {
		/* bit[19:0] XBar_Assist TCPC Request Timeout value, Timeout value for the TCPC request.
		 * Calculated as (tca_clk_period*256* FLD_XA_TIMEOUT_VAL).
		 * The default timeout value is calculated with 19.2MHz tca_clk for value of 500ms */
		unsigned xa_timeout_val:20;
		/* bit[31:20] */
		unsigned RSVD31_20:12;
	} b;
} SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_CFG1_o, *SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_CFG1_p;

#define SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_DBG0  0x0028
typedef union {
	u32 data;
	struct {
		/* bit[0] dpalt_dp4 status/value provides the status of the signal
		 * as driven by TCA block */
		unsigned dpalt_dp4:1;
		/* bit[1] dpalt_disable status/value provides the status of the signal
		 * as driven by TCA block */
		unsigned dpalt_disable:1;
		/* bit[2] dpalt_disable_ack status/value Status/Value as seen on TCA input. */
		unsigned dpalt_disable_ack:1;
		/* bit[3] */
		unsigned RSVD3:1;
		/* bit[4] ss_rxdetect_disable status/value provides the status of the signal
		 * as driven by TCA block. */
		unsigned ss_rxdetect_disable:1;
		/* bit[5] ss_rxdetect_disable_ack status/value Status/Value as seen on TCA input. */
		unsigned ss_rxdetect_disable_ack:1;
		/* bit[6] ss_rxdet_disable_1 status/value provides the status of the signal
		 * as driven by TCA block. */
		unsigned ss_rxdet_disable_1:1;
		/* bit[7] ss_rxdet_disable_ack_1 status/value Status/Value as seen on TCA input. */
		unsigned ss_rxdet_disable_ack_1:1;
		/* bit[8] block_ss_op status/value provides the status of the signal
		 * as driven by TCA block. */
		unsigned block_ss_op:1;
		/* bit[9] ss_lane0_active status/value as seen on TCA input. */
		unsigned ss_lane0_active:1;
		/* bit[10] ss_lane1_active status/value as seen on TCA input. */
		unsigned ss_lane1_active:1;
		/* bit[11] */
		unsigned RSVD11:1;
		/* bit[12] xbar_ready status/value as driven by XBar_Assist to VBUS_Assist */
		unsigned xbar_ready:1;
		/* bit[13] tca_clk synchronized status in TCA_PSTATE Register.
		 *  - 1'b0: Status is not synchronized in tca_clk
		 *  - 1'b1: Status is synchronized in tca_clk
		 * There could be big difference in two values if tca_clk is
		 * very slow (derived from suspend_clk). */
		unsigned pstate_synced:1;
		/* bit[14] TCA_PSTATE Register values are captured at which specific moment
		 *  - 1'b0: Current values are shown.
		 *  - 1'b1: Values captured at the moment when TCPC request Timeout occurs. */
		unsigned pstate_timeout_val:1;
		/* bit[19:15] Multiplier value for xa_req_compl_cycles[15:0] status in
		 * TCA_CTRLSYNCMODE_DBG1 Register.
		 * Multiplier = 2 ^ (xa_req_compl_mult[4:0]) */
		unsigned xa_req_compl_mult:5;
		/* bit[24:20] Override enable bits for
		 * ctrl_if_ovvrd_en[0]:  dpalt_dp4
		 * ctrl_if_ovvrd_en[1]:  dpalt_disable
		 * ctrl_if_ovvrd_en[2]:  dpalt_disable_ack
		 * ctrl_if_ovvrd_en[3]:  ss_rxdet_disable
		 * ctrl_if_ovvrd_en[4]:  ss_rxdet_disable_ack
		 * Nth bit when 1'b1 enables the override of related controller
		 * handshake signal using corresponding ctrl_if_ovrrd_val[N]. */
		unsigned ctrl_if_ovrrd_en:5;
		/* bit[29:25] Override values for controller handshake signals when corresponding
		 * ctrl_if_ovvrd_en[N] is high. */
		unsigned ctrl_if_ovrrd_val:5;
		/* bit[31:30] */
		unsigned RSVD31_30:2;
	} b;
} SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_DBG0_o, *SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_DBG0_p;

#define SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_DBG1  0x002c
typedef union {
	u32 data;
	struct {
		/* bit[15:0] TCPC req completion cycles
		 * Indicates number of tca_clk clock cycles it took to
		 * complete the TCPC mode change request using the TCA_TCPC register.
		 * Calculated as:
		 * xa_req_comp_cycles * 2 ^ (FLD_XA_REQ_COMP_MULT) */
		unsigned xa_req_compl_cycles:16;
		/* bit[16] Indicates if the overrun condition has occurred on
		 * xa_req_comp_cycles[15:0] status register.
		 * Need to set appropriately high value of FLD_XA_REQ_COMP_MULT. */
		unsigned xa_req_compl_cycles_overrun:1;
		/* bit[31:17] */
		unsigned RSVD31_17:15;
	} b;
} SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_DBG1_o, *SNPS_USBDPPHY_TCA_TCA_CTRLSYNCMODE_DBG1_p;

#define SNPS_USBDPPHY_TCA_TCA_PSTATE     0x0030
typedef union {
	u32 data;
	struct {
		/* bit[1:0] ssrx_pstate[1:0] status/value from the PHY */
		unsigned ssrx_pstate:2;
		/* bit[2] ssrx_req status/value from the PHY  */
		unsigned ssrx_req:1;
		/* bit[3] ssrx_ack status/value from the PHY */
		unsigned ssrx_ack:1;
		/* bit[5:4] sstx_pstate[1:0] status/value from the PHY  */
		unsigned sstx_pstate:2;
		/* bit[6] sstx_req status/value from the PHY  */
		unsigned sstx_req:1;
		/* bit[7] sstx_ack status/value from the PHY  */
		unsigned sstx_ack:1;
		/* bit[9:8] dp_tx0_pstate[1:0] status/value from the PHY  */
		unsigned dp_tx0_pstate:2;
		/* bit[10] dp_tx0_req status/value from the PHY  */
		unsigned dp_tx0_req:1;
		/* bit[11] dp_tx0_ack status/value from the PHY  */
		unsigned dp_tx0_ack:1;
		/* bit[13:12] dp_tx1_pstate[1:0] status/value from the PHY */
		unsigned dp_tx1_pstate:2;
		/* bit[14] dp_tx1_req status/value from the PHY  */
		unsigned dp_tx1_req:1;
		/* bit[15] dp_tx1_ack status/value from the PHY  */
		unsigned dp_tx1_ack:1;
		/* bit[17:16] dp_tx2_pstate[1:0] status/value from the PHY  */
		unsigned dp_tx2_pstate:2;
		/* bit[18] dp_tx2_req status/value from the PHY  */
		unsigned dp_tx2_req:1;
		/* bit[19] dp_tx2_ack status/value from the PHY  */
		unsigned dp_tx2_ack:1;
		/* bit[21:20] dp_tx3_pstate[1:0] status/value from the PHY  */
		unsigned dp_tx3_pstate:2;
		/* bit[22] dp_tx3_req status/value from the PHY  */
		unsigned dp_tx3_req:1;
		/* bit[23] dp_tx3_ack status/value from the PHY   */
		unsigned dp_tx3_ack:1;
		/* bit[31:24] */
		unsigned RSVD31_24:8;
	} b;
} SNPS_USBDPPHY_TCA_TCA_PSTATE_o, *SNPS_USBDPPHY_TCA_TCA_PSTATE_p;

#define SNPS_USBDPPHY_TCA_TCA_GEN_STATUS     0x0034
typedef union {
	u32 data;
	struct {
		/* bit[1:0] phy_typec_conn_mode[1:0] value as driven by TCA to the DPAlt_XBar   */
		unsigned phy_typec_conn_mode:2;
		/* bit[2] phy_typec_flip value as driven by TCA to the DPAlt_XBar  */
		unsigned phy_typec_flip:1;
		/* bit[3] phy_typec_disable value as driven by TCA to the DPAlt_XBar  */
		unsigned phy_typec_disable:1;
		/* bit[4] typec_flip_invert ComboPHY input value as read by the TCA.  */
		unsigned typec_flip_invert:1;
		/* bit[7:5] */
		unsigned RSVD7_5:3;
		/* bit[8] ref_clk_sel as driven by PMA in the PHY and read by TCA.  */
		unsigned ref_clk_sel:1;
		/* bit[11:9] */
		unsigned RSVD11_9:3;
		/* bit[12] usb_dev_por ComboPHY input value as read by the TCA. */
		unsigned usb_dev_por:1;
		/* bit[13] dp4_por ComboPHY input value as read by the TCA.  */
		unsigned dp4_por:1;
		/* bit[31:14] */
		unsigned RSVD31_14:18;
	} b;
} SNPS_USBDPPHY_TCA_TCA_GEN_STATUS_o, *SNPS_USBDPPHY_TCA_TCA_GEN_STATUS_p;

#define SNPS_USBDPPHY_TCA_TCA_VBUS_CTRL  0x0040
typedef union {
	u32 data;
	struct {
		/* bit[1:0] tca_vbusvalid override
		 *  - 2'b00: drive 0
		 *  - 2'b01: drive 1
		 *  - 2'b10: follow sys_vbusvalid
		 *  - 2'b11: follow sys_vbusvalid with XBar_Assist
		 * associated override using xbar_ready */
		unsigned vbusvalid_overrd:2;
		/* bit[3:2] tca_powerpresent override
		 *  - 2'b00: drive 0
		 *  - 2'b01: drive 1
		 *  - 2'b10: follow sys_vbusvalid
		 *  - 2'b11: follow sys_vbusvalid with XBar_Assist
		 * associated override using xbar_ready */
		unsigned powerpresent_overrd:2;
		/* bit[5:4] tca_drv_host_vbus overdrive
		 *  - 2'b00: drive 0
		 *  - 2'b01: drive 1
		 *  - 2'b10: drive 0
		 *  - 2'b11: use XBar_Assist sttes to drive tca_drv_host_vbus */
		unsigned drv_host_vbus_overrd:2;
		/* bit[7:6] */
		unsigned RSVD7_6:2;
		/* bit[8] tca_iddig control
		 *  - 1'b0: drive 0
		 *  - 1'b1: drive 1 */
		unsigned tca_iddig:1;
		/* bit[14:9] tca miscellaneous control tca_misc_control[5:0], for each bit
		 *  - 1'b0: drive 0
		 *  - 1'b1: drive 1 */
		unsigned tca_misc_ctrl:6;
		/* bit[31:15] */
		unsigned RSVD31_15:17;
	} b;
} SNPS_USBDPPHY_TCA_TCA_VBUS_CTRL_o, *SNPS_USBDPPHY_TCA_TCA_VBUS_CTRL_p;

#define SNPS_USBDPPHY_TCA_TCA_VBUS_STATUS    0x0044
typedef union {
	u32 data;
	struct {
		/* bit[0] sys_vbusvalid status/value as seen on TCA block input. */
		unsigned sys_vbusvalid:1;
		/* bit[1] tca_vbusvalid status/value as seen on TCA block output. */
		unsigned tca_vbusvalid:1;
		/* bit[2] tca_powerpresent status/value as seen on TCA block output. */
		unsigned tca_powerpresent:1;
		/* bit[3] tca_drv_host_vbus status/value as seen on TCA block output. */
		unsigned tca_drv_host_vbus:1;
		/* bit[31:4] */
		unsigned RSVD31_4:28;
	} b;
} SNPS_USBDPPHY_TCA_TCA_VBUS_STATUS_o, *SNPS_USBDPPHY_TCA_TCA_VBUS_STATUS_p;

#define SNPS_USBDPPHY_TCA_TCA_INFO_REG   0x00fc
typedef union {
	u32 data;
	struct {
		/* bit[31:0] Version ID for TCA. */
		unsigned version_id:32;
	} b;
} SNPS_USBDPPHY_TCA_TCA_INFO_REG_o, *SNPS_USBDPPHY_TCA_TCA_INFO_REG_p;

#endif
