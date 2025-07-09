/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung EXYNOS SoC series USB PHY driver
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef _USB_USBPHY_CAL_PHY_EXYNOS_USBCON_LINK_SFR_H_
#define _USB_USBPHY_CAL_PHY_EXYNOS_USBCON_LINK_SFR_H_

#define USBCON_REG_VERSION		0x0000
/*
 * Description: USB Sub-CTRL version register
 * [31:24] - Major version,
 * [23:16] - Minor version
 * [15:8] - USB3 PHY
 * 		8'h00 : None
 *		8'h01 : SEC SS Combo PHY
 *		8'h02 : SEC SSP Combo PHY
 *		8'h41 : Synopsys 30 PHY (SS+HS)
 * 		8'h42 : Synopsys SS PHY
 * 		8'h43 : Synopsys SSP PHY
 * [7:0] - USB2 PHY
 *      8'h00 : None
 *      8'h41 : Synopsys 20 PHY
 *      8'h42 : Synopsys eUSB PHY
 */
typedef union {
	u32 data;
	struct {
		/* bit[7:0] : USB2 PHY */
		unsigned USB2PHY	: 8;
		/* bit[15:8] : USB3 PHY */
		unsigned USB3PHY	: 8;
		/* bit[23:16] : */
		unsigned MINOR_VER	: 8;
		/* bit[31:24] : */
		unsigned MAJOR_VER	: 8;
	} b;
} USBCON_REG_VERSION_o, *USBCON_REG_VERSION_p;

#define USBCON_REG_LINKCTRL		0x0004
typedef union {
	u32 data;
	struct {
		/*
		 * bit[0]
		 * Function: This signal is used to generate the PME# (Power Management Event).
		 * When the Run/Stop bit of the USB Command Register is cleared during USB suspend
		 * mode, the core cannot generate event and cannot assert regular interrupt.
		 * In this case, the core asserts pme_generation signal to report any wakeup
		 * condition if pme_en is high. If the system does not support PCI-like PME
		 * interface, then it should not clear the Run/Stop bit during USB suspend mode.
		 * In this case, the core generates event and asserts interrupt when there is
		 * any wakeup event. If the pme interface is not used, connect pme_en to zero and
		 * keep pme_generation unconnected. This is a level signal that gets cleared when
		 * the software clears CSC, OCC and PLC bits in all the PORTSC registers.
		 * In device mode connect pme_en to zero and keep pme_generation unconnected.
		 */
		unsigned pme_generation:1;
		/*
		 * bit[1] Enable Signal for the pme_generation.
		 * Function: Enable the core to assert pme_generation. Refer to the
		 * description of pme_generation signal for details.
		 */
		unsigned pme_en:1;
		/*
		 * Bit[2] Indicates that a Host System Error has occurred as
		 * reflected in the USBSTS.HSE field.
		 * It can occur when the host controller encounters an 'Error'
		 * response in the AHB, the AXI, or the Native Master Bus.
		 */
		unsigned host_system_err:1;
		/* Bit[7:4] */
		unsigned polarity_inversion:1;
		/*
		 * This signal disables the internal bus filters that are enabled by
		 * DWC_USB31_EN_BUS_FILTERS coreConsultant parameter. This static signal is present
		 * only when DWC_USB31_EN_BUS_FILTERS is 1. It is expected that this signal is set
		 * or reset at power-on reset and is not changed during the normal operation of the
		 * controller.
		 *
		 * The function of each bit is:
		 *   ■ bus_filter_bypass[2]: Bypass the filter for utmisrp_bvalid
		 *   ■ bus_filter_bypass[1]: Bypass the filter for pipe_PowerPresent all U3 ports
		 *   ■ bus_filter_bypass[0]: Bypass the filter for utmiotg_vbusvalid all U2 ports
		 * In Host mode, internal bus filters are not needed.
		 * Therefore, bus_filter_bypass[2:0] must be connected to logic high value (3'b111)
		 */
		unsigned bus_filter_bypass:4;
		/* Bit[8] disable HWACG */
		unsigned force_qact:1;
		/* Bit[9] Masking bvaild signal for HWACG */
		unsigned dis_bvalid_qact:1;
		/* Bit[10] Masking bvaild signal for HWACG */
		unsigned dis_vbusvalid_qact:1;
		/* Bit[11] Masking bvaild signal for HWACG */
		unsigned dis_id0_qact:1;
		/* Bit[12] Masking bvaild signal for HWACG */
		unsigned dis_linkgate_qact:1;
		/* Bit[13] Masking bvaild signal for HWACG */
		unsigned dis_buspend_qact:1;
		/* Bit[15:14] Reserved */
		unsigned RSVD15_14:2;
		/* Bit[16] */
		unsigned force_pipe_en:1;
		/* Bit[17] */
		unsigned force_phystatus:1;
		/* Bit[18] */
		unsigned force_rxelecidle:1;
		/* Bit[19] Reserved */
		unsigned RSVD19:1;
		/* Bit[21:20] Selection for SoF Out signal */
		unsigned sel_sof:2;
		/* Bit[22] */
		unsigned pmgt_ref_clk_ok:1;
		/* Bit[23] */
		unsigned pmgt_ref_clk_off:1;
		/* Bit[24] */
		unsigned pmgt_ext_bus_clk_gated:1;
		/* Bit[25] */
		unsigned pmgt_ext_bus_clk_ok:1;
		/* Bit[26] */
		unsigned pmgt_ext_bus_clk_off:1;
		/* Bit[27] */
		unsigned _20phy_ref_clk_req:1;
		/*
		 * Bit[31:28] Option to Over-ride the device speed
		 * [3] : Enable/Disable
		 * [2:0]: set speed
		 */
		unsigned devspd_ovrd:4;
	} b;
} USBCON_REG_LINKCTRL_o, *USBCON_REG_LINKCTRL_p;

#define USBCON_REG_LINKPORT		0x0008
typedef union {
	u32 data;
	struct {
		/*
		 * bit[0] : Indicates if the device attached to a downstream usb2 port is
		 * permanently attached or not.
		 *  ■ 0: Not permanently attached
		 *  ■ 1: Permanently attached
		 */
		unsigned hub_perm_attach_u2:1;
		/*
		 * bit[1] : Indicates if the device attached to a downstream usb3 port is
		 * permanently attached or not.
		 *  ■ 0: Not permanently attached
		 *  ■ 1: Permanently attached
		 */
		unsigned hub_perm_attach_u3:1;
		/*
		 * bit[2] Select over-current receiving for usb2
		 *  ■ 0 :
		 *  ■ 1 :
		 */
		unsigned hub_port_overcurrent_sel_u2:1;
		/*
		 * bit[3] Select over-current receiving for usb3
		 *  ■ 0 :
		 *  ■ 1 :
		 */
		unsigned hub_port_overcurrent_sel_u3:1;
		/*
		 * bit[4] Value of Over-current of usb2 port
		 *  ■ 0 : Over-current status occurred
		 *  ■ 1 : Normal status
		 */
		unsigned hub_port_overcurrent_u2:1;
		/*
		 * bit[5] Value of Over-current of usb3 port
		 *  ■ 0 : Over-current status occurred
		 *  ■ 1 : Normal status
		 */
		unsigned hub_port_overcurrent_u3:1;
		/*
		 * bit[6] : This port defines the bit [3] of Capability Parameters (HCCPARAMS).
		 *          Change the PPC value through the pin Port Power Control (PPC).
		 *          This indicates whether the host controller implementation includes port
		 *          power control.
		 *		■ 0: Indicates that the port does not have port power switches.
		 *		■ 1: Indicates that the port has port power switches.
		 */
		unsigned host_port_power_con_present:1;
		/* bit[7] : */
		unsigned RSVD7:1;
		/*
		 * bit[8] : USB 2.0 Port Disable control
		 *  ■ 0: Port Enabled
		 *  ■ 1: Port Disabled
		 * This signal, when '1', stops reporting connect/disconnect events on the port.
		 * This could be used for security reasons where hardware can disable a port
		 * irrespective of whether xHCI driver enables a port or not. The 'Number of Ports'
		 * field of the HCSPARAMS1 register is not affected by this signal.
		 * This signal should be static after vcc_reset_n is de-asserted (should not change
		 * during operation).
		 * Note: This signal should not be asserted to disable USB2 Port0.
		 */
		unsigned host_u2_port_disable:1;
		/*
		 * bit[9] : USB 3.1 ESS Port Disable control
		 *  ■ 0: Port Enabled
		 *  ■ 1: Port Disabled
		 * This signal, when '1', stops reporting connect/disconnect events on the port.
		 * This could be used for security reasons where hardware can disable a port
		 * irrespective of whether xHCI driver enables a port or not. The 'Number of Ports'
		 * field of the HCSPARAMS1 register is not affected by this signal.
		 * This signal should be static after vcc_reset_n is de-asserted (should not change
		 * during operation).
		 * Note: This signal should not be asserted to disable eSS Port0.
		 */
		unsigned host_u3_port_disable:1;
		/* bit[10] Indicator of Port Power control usb2 port */
		unsigned hub_vbus_ctrl_u2:1;
		/* bit[11] Indicator of Port Power control eSS usb3 port */
		unsigned hub_vbus_ctrl_u3:1;
		/* bit[12]  */
		unsigned host_num_u2:1;
		/* bit[13] */
		unsigned host_num_u3:1;
		/* Bit[31:20] Reserved */
		unsigned RSVD131_20:12;
	} b;
} USBCON_REG_LINKPORT_o, *USBCON_REG_LINKPORT_p;

#define USBCON_REG_LINK_CLKRST		0x000C
typedef union {
	u32 data;
	struct {
		/* bit[0] : link reset overriding (to Link) with preset_n */
		unsigned link_sw_rst:1;
		/* bit[1] : Link pclk selection signal- 0:suspend_clk, 1:pipe3_clock */
		unsigned link_pclk_sel:1;
		/* Reserved 3:2 */
		unsigned RSVD3_2:2;
		/* bit[4] Clock selection for USB Audio (0: ref_clk, 1: utmi_clk) */
		unsigned usbaudio_clk_sel:1;
		/* bit[5] Enable or disable clock gate out for usb audio */
		unsigned usbaudio_clk_gate_en:1;
		/* bit[6] Glitch Free Mux reset for USB Audio clock */
		unsigned usbaudioclk_gfmux_rst:1;
		// bit[31:7] :
		unsigned RSVD31_7:25;
	} b;
} USBCON_REG_LINK_CLKRST_o, *USBCON_REG_LINK_CLKRST_p;

#define USBCON_REG_UTMI_CTRL		0x0010
typedef union {
	u32 data;
	struct {
		/* bit[0] : force b-valid signal to high */
		unsigned force_bvalid:1;
		/* bit[1] : force vbus-valid signal to high */
		unsigned force_vbusvalid:1;
		/* bit[2] : force tx bit stuffen */
		unsigned force_utmi_txbitstuffen:1;
		/* bit[3] */
		unsigned RSVD3:1;
		/* bit[4] Enable forcing opmode over-ride */
		unsigned force_utmi_opmode_en:1;
		/* bit[6:5] force opmode signal */
		unsigned force_utmi_opmode:2;
		// bit[7] :
		unsigned RSVD7:1;
		/* bit[8] forcing DM-Pull down enable signal to high */
		unsigned force_DMPULLDOWN:1;
		/* bit[9] forcing DP-Pull down enable signal to high */
		unsigned force_DPPULLDOWN:1;
		// bit[11:10] :
		unsigned RSVD11_10:2;
		/* bit[12] Forcing sleep_n signal */
		unsigned force_utmi_sleep:1;
		/* bit[13] Forcing suspend_n signal */
		unsigned force_utmi_suspend:1;
		// bit[31:14] :
		unsigned RSVD2:18;
	} b;
} USBCON_REG_UTMI_CTRL_o, *USBCON_REG_UTMI_CTRL_p;

#define USBCON_REG_ESS_CTRL		0x0014
typedef union {
	u32 data;
	struct {
		/* bit[0] : Force Host Speed gen1 */
		unsigned host_force_gen1_speed:1;
		/* bit[3:1] */
		unsigned RSVD3_1:3;
		/*
		 * bit[4] : PTM time adjustment valid
		 * Indicates the ptm_time specified is valid for capture With ptm_time_vld
		 * tied to zero ptm_time doesn't update host PTM clock You must set this
		 * signal high when ptm_time indicates the wall clock in the system and
		 * ready to be loaded into Host ptm_time_vld loads ptm_time in to host
		 * SOF/ITP counters If your system doesn't support PTM, it is recommended
		 * to tie this input to 0.
		 */
		unsigned ptm_time_vld:1;
		/* bit[7:5] */
		unsigned RSVD7_5:3;
		/*
		 * bit[8] Start Receiver Detection in U3/Rx.Detect (StartRxdetU3RxDet)
		 * If DisRxDetU3RxDet is set and the link is in either U3 or Rx.Detect
		 * state, the controller starts receiver detection on the rising edge of this
		 * bit. This signal can only be used for downstream ports. It must be set
		 * to '0' for upstream ports.
		 * Note: This signal is for debug purpose only, it should be tied to 0 in normal
		 * operation.
		 */
		unsigned StartRxDetU3RxDet:1;
		/*
		 * bit[9] DisRxDetU3RxDet_ack of USB 3.1 SS Ports
		 * This input signal requests the controller to stop issuing more PHY commands and
		 * release the PIPE ownership. The controller acknowledges this request by
		 * asserting the DisRxDetU3RxDet_ack signal.
		 * This signal is for Type-C support.
		 */
		unsigned DisRxDetU3RxDet:1;
		/*
		 * bit[10] Indicator DisRxDetU3RxDet_ack of USB 3.1 SS Ports
		 * This signal is an acknowledgment of DisRxDetU3RxDet. When this signal is 1'b1,
		 * the controller releases the ownership of the PIPE interface and does not issue
		 * PHY commands to the PIPE interface.
		 */
		unsigned DisRxDetU3RxDet_ack:1;
		// RSVD[31:11]
		unsigned RSVD31_11:21;
	} b;
} USBCON_REG_ESS_CTRL_o, *USBCON_REG_ESS_CTRL_p;

#define USBCON_REG_PTM_TIME		0x0018
typedef union {
	u32 data;
	struct {
		/*
		 * bit[31:0] : PTM time adjustment value
		 * This signal indicates the ptm_time in terms of milliseconds and nanoseconds.
		 * With ptm_time_vld tied to zero ptm_time does not update the host PTM clock.
		 * You must set this signal valid when ptm_time_vld is asserted.
		 * 		■ ptm_time[31:20] indicates wall clock in milliseconds
		 * 		■ ptm_time[19:0] indicates wall clock in nanoseconds
		 * 		■ ptm_time_vld loads ptm_time in to host SOF/ITP counters initial
		 * 		  values when the Host controller exits halted state.
		 * If your system does not support PTM, tie this input to 0.
		 */
		unsigned ptm_time:32;
	} b;
} USBCON_REG_ESS_PTM_TIME_o, *USBCON_REG_ESS_PTM_TIME_p;

#define USBCON_REG_LINK_SOCBW		0x001C
typedef union {
	u32 data;
	struct {
		/*
		 * bit[14:0] :
		 * This signal is used to indicate maximum write bandwidth on SoC
		 * available in terms of kilo bytes per micro-frame.
		 * For example, if the AXI write bandwidth is 150MBps, then the value
		 * for this signal is 150/8 = 19.
		 */
		unsigned soc_wr_uF_kB_bandwidth:15;
		/* Bit[15] */
		unsigned RSVD15:1;
		/*
		 * bit[14:0] :
		 * This signal is used to indicate maximum read bandwidth on SoC
		 * available in terms of kilo bytes per micro-frame.
		 * For example, if the AXI read bandwidth is 150MBps, then the value
		 * for this signal is 150/8 = 19.
		 */
		unsigned soc_rd_uF_kB_bandwidth:15;
		/* Bit[31] */
		unsigned RSVD31:1;

	} b;
} USBCON_REG_LINK_SOCBW_o, *USBCON_REG_LINK_SOCBW_p;

#define USBCON_REG_REWA_CTL	 0x0050
typedef union {
	u32 data;
	struct {
		/* bit[0] HS ReWA Enable */
		unsigned hsrewa_en:1;
		/* bit[31:1] */
		unsigned RSVD31_1:31;
	} b;
} USBCON_REG_REWA_CTL_o, *USBCON_REG_REWA_CTL_p;

#define USBCON_REG_HSREWA_INTR	 0x0054
typedef union {
	u32 data;
	struct {
		/*
		 * bit[0] Remote wake-up interrupt mask (to GIC)
		 * If this bit set to 1, than usb2_wakeup_int0 interrupt is not generated.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt.
		 */
		unsigned wakeup_intr_mask:1;
		/* bit[3:1] */
		unsigned RSVD3_1:3;
		/* bit[4] Status change and exception interrupt mask (to GIC)
		 * If this bit set to 1, than usb2_wakeup_int1 interrupt is not generated.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt. */
		unsigned event_intr_mask:1;
		/* bit[7:5] */
		unsigned RSVD7_5:3;
		/*
		 * bit[8] Host K timeout interrupt mask (to GIC)
		 * If this bit set to 1, than usb2_wakeup_int2 interrupt is not generated.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt.
		 */
		unsigned timeout_intr_mask:1;
		/* bit[11:9] */
		unsigned RSVD11_9:3;
		/*
		 * bit[12] Wakeup interrupt mask (to PMU)
		 * If this bit set to 1, than usb2_wakeup interrupt is not generated.
		 * This interrupt asserted by usb2_wakeup_int0 and usb2_wakeup_int1.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt.
		 */
		unsigned wakeup_req_mask:1;
		/* bit[31:13] */
		unsigned RSVD31_13:19;
	} b;
} USBCON_REG_HSREWA_INTR_o, *USBCON_REG_HSREWA_INTR_p;

#define USBCON_REG_HSREWA_CTL	 0x0058
typedef union {
	u32 data;
	struct {
		/*
		 * bit[0] Done indicator bit for system S/W
		 * This bit value means that finish for ReWA operation.
		 * 0: Not done
		 * 1: Done
		 */
		unsigned hs_rewa_done:1;
		/* bit[3:1] */
		unsigned RSVD3_1:3;
		/*
		 * bit[4] Error indicator bit for system S/W.
		 * This bit value means that error detected during a ReWA is enabled.
		 * If this bit value is 1, than S/W shuld check the HSREWA_INT1_EVNT register to get
		 * the error status.
		 * 0: No errors
		 * 1: Error detected
		 */
		unsigned hs_rewa_error:1;
		/* bit[15:5] */
		unsigned RSVD15_5:11;
		/*
		 * bit[16] System valid flag for ReWA.
		 * System S/W should set this bit when finished wake-up sequence from sleep mode.
		 * 0: System clock is invalid
		 * 1: System clock is valid
		 */
		unsigned hs_sys_valid:1;
		/* bit[19:17] */
		unsigned RSVD19_17:3;
		/*
		 * bit[20] USB Link ready flag for ReWA.
		 * System S/W should set this bit when finished USB link event handling that port
		 * status change and host K drive command.
		 * 0: Link is not ready
		 * 1: Link is ready
		 */
		unsigned hs_link_ready:1;
		/* bit[23:21] */
		unsigned RSVD23_21:3;
		/*
		 * bit[24] DP/DM monitoring method selector.
		 * HS-ReWA provide two DP/DM monitoring method for detect DP/DM status changes when
		 * ReWA is enabled.
		 * This bit value is should be stable before Enable the ReWA.
		 * 0: FSVPLUS/FSVMINUS
		 * 1: LINESTATE[1:0]
		 */
		unsigned dpdm_mon_sel:1;
		/* bit[27:25] */
		unsigned RSVD27_25:3;
		/* bit[28] Digital bypass controller enable.
		 * HS-ReWA use to PHY's digital bypass feature for host k generation.
		 * ??This bit value is should be stable before Enable the ReWA.
		 * 0: Disable digital bypass controller.
		 * 1: Enable digital bypass controller. */
		unsigned dig_bypass_con_en:1;
		/* bit[31:29] */
		unsigned RSVD31_29:3;
	} b;
} USBCON_REG_HSREWA_CTL_o, *USBCON_REG_HSREWA_CTL_p;

#define USBCON_REG_HSREWA_REFTO	 0x005c
typedef union {
	u32 data;
	struct {
		/*
		 * bit[31:0] Reference value to determine host K timeout from host K drive.
		 * The unit of this value is i_clk tick, In other word value 1 means 30.5us.
		 *
		 * According to USB specification, Nominal host K drive time is 20ms.
		 * ReWA provide host K timeout interrupt and this value is expire time reference
		 * value of internal counter.
		 *
		 * Default value of this register means about 20ms.
		 *
		 * If this register value is 0 then ReWA override this value to default value
		 * (0x28d). So the minimum value of this register is 1.
		 *
		 * The original value of this register is 0x290, but the initial value is include a
		 * minus offset that 3. Because of the ReWA has an internal delay time about 3 tick
		 * of i_clk for dp/dm linestate detection.
		 */
		unsigned host_k_timeout:32;
	} b;
} USBCON_REG_HSREWA_REFTO_o, *USBCON_REG_HSREWA_REFTO_p;

#define USBCON_REG_HSREWA_HSTK	 0x0060
typedef union {
	u32 data;
	struct {
		/*
		 * bit[31:0] Reference value to determine host K delay from device K event.
		 * The unit of this value is i_clk tick, In other word value 1 means 30.5us.
		 * According to USB specification, Host should respond for remote wakeup request
		 * from device at least 900us.
		 *
		 * Default value of this register means about 854us.
		 *
		 * If this register value is 0 then ReWA override this value to default value
		 * (0x19). So the minimum value of this register is 1.
		 *
		 * The original value of this register is 0x1c, but the initial value is include a
		 * minus offset that 3. Because of the ReWA has an internal delay time about 3 tick
		 * of i_clk for dp/dm linestate detection.
		 */
		unsigned host_k_delay:32;
	} b;
} USBCON_REG_HSREWA_HSTK_o, *USBCON_REG_HSREWA_HSTK_p;

#define USBCON_REG_HSREWA_CNT	 0x0064
typedef union {
	u32 data;
	struct {
		/* bit[31:0] Current timer value that accumulated from device K event. */
		unsigned wakeup_cnt:32;
	} b;
} USBCON_REG_HSREWA_CNT_o, *USBCON_REG_HSREWA_CNT_p;

#define USBCON_REG_HSREWA_INT1_EVNT	 0x0068
typedef union {
	u32 data;
	struct {
		/* bit[0] UTMI signal retention enabled event flag
		 * This bit value means that all of UTMI signals will be retained.
		 * 0: UTMI retention is not enabled
		 * 1: UTMI retention is enabled */
		unsigned ret_en:1;
		/* bit[1] UTMI signal retention disabled event flag
		 * This bit value means that retention disabled (bypass) for all of UTMI signals.
		 * 0: UTMI retention is not disabled
		 * 1: UTMI retention is disabled */
		unsigned ret_dis:1;
		/* bit[2] Digital BYPASS disabled event flag
		 * This bit value means that Digital BYPASS control signals is disabled, and the
		 * ownership of DP/DM is return to UTMI signals.
		 *
		 * 0: Digital BYPASS is not disabled
		 * 1: Digital BYPASS is disabled */
		unsigned bypass_dis:1;
		/* bit[15:3] */
		unsigned RSVD15_3:13;
		/* bit[16] Disconnect detected error event flag
		 * This bit value means that disconnect error detected during a ReWA is enabled.
		 * 0: Disconnect event is not detected
		 * 1: Disconnect event is detected */
		unsigned discon:1;
		/* bit[17] Invalid device K error event flag
		 * This bit value means that invalid device K detected during a waiting for host K
		 * drive after device K event.
		 *
		 * 0: Device K error is not detected
		 * 1: Device K error is detected */
		unsigned err_dev_k:1;
		/* bit[18] Suspend state error event flag
		 * This bit value means that USB PHY is not suspend state when the ReWA is enabled.
		 * 0: Suspend error is not detected
		 * 1: Suspend error is detected */
		unsigned err_sus:1;
		/* bit[31:19] */
		unsigned RSVD31_19:13;
	} b;
} USBCON_REG_HSREWA_INT1_EVNT_o, *USBCON_REG_HSREWA_INT1_EVNT_p;

#define USBCON_REG_HSREWA_INT1_EVNT_MSK	 0x006c
typedef union {
	u32 data;
	struct {
		/* bit[0] Status event mask for interrupt generation.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt. */
		unsigned ret_en_mask:1;
		/* bit[1] Status event mask for interrupt generation.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt. */
		unsigned ret_dis_mask:1;
		/* bit[2] Status event mask for interrupt generation.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt. */
		unsigned bypass_dis_mask:1;
		/* bit[15:3] Reserved */
		unsigned reserve_mask0:13;
		/* bit[16] Exception event mask for interrupt generation.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt. */
		unsigned discon_mask:1;
		/* bit[17] Exception event mask for interrupt generation.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt. */
		unsigned err_dev_k_mask:1;
		/* bit[18] Exception event mask for interrupt generation.
		 * 0: Enable Interrupt.
		 * 1: Mask interrupt. */
		unsigned err_sus_mask:1;
		/* bit[31:19] Reserved */
		unsigned reserve_mask1:13;
	} b;
} USBCON_REG_HSREWA_INT1_EVNT_MSK_o, *USBCON_REG_HSREWA_INT1_EVNT_MSK_p;

#define USBCON_REG_U3REWA_CTRL	 0x0070
typedef union {
	u32 data;
	struct {
		/*
		 * bit[0] U3 rewa operation enable. Actullay, when this bit is enabled, RxElecIdle
		 * signal is masked to Link. Before Link enter the U3 state, this bit is set. And,
		 * If the system is enabled, it should be released.
		 */
		unsigned u3rewa_blk_en:1;
		/*
		 * bit[1] when this bit is set, it generates tx LFPS which can overlap absolutely
		 * with Rx LFPS.
		 */
		unsigned overlap_lfps:1;
		/* bit[2] check in U3 when the U3Rewa works */
		unsigned check_u3:1;
		/* bit[3] */
		unsigned RSVD3:1;
		/* bit[7:4] The number of skipped LFPS burst due to U3Rewa masking */
		unsigned skip_wakeuplfps_cnt:4;
		/* bit[23:8] */
		unsigned RSVD23_8:16;
		/* bit[24] System wakeup Interrupt masking */
		unsigned sys_wakeup_intr_mask:1;
		/* bit[27:25] */
		unsigned RSVD27_25:3;
		/* bit[28] System wakeup Interrupt */
		unsigned sys_wakeup_intr:1;
		/* bit[31:29] */
		unsigned RSVD31_29:3;
	} b;
} USBCON_REG_U3REWA_CTRL_o, *USBCON_REG_U3REWA_CTRL_p;

#define USBCON_REG_U3REWA_LMT_CNT	 0x0074
typedef union {
	u32 data;
	struct {
		/*
		 * bit[31:0] Link needs some time to generate LFPS from receiving Rx LFPS. So, when
		 * Link receives the end of Rx LFPS by ReWa Masking, LFPS overlapping may not be
		 * achieved. At this time, it should ignore this Rx LFPS. i_lfpsresp_limit_cnt is
		 * limitation counter value for release RxElecIdle mask with i_suspend_clk. This
		 * value can be set as following.
		 * ((Expected RxLFPS Burst period (10ms) - margin (1ms)) /
		 * i_suspend_clk period(38.26ns))
		 */
		unsigned lfpsresp_limit_cnt:32;
	} b;
} USBCON_REG_U3REWA_LMT_CNT_o, *USBCON_REG_U3REWA_LMT_CNT_p;

#define USBCON_REG_LTSTATE_HIS	 0x0080
typedef union {
	u32 data;
	struct {
		/* bit[3:0] LTSSM State history buffer 0 */
		unsigned ltstate_his0:4;
		/* bit[7:4] LTSSM State history buffer 1 */
		unsigned ltstate_his1:4;
		/* bit[11:8] LTSSM State history buffer 2 */
		unsigned ltstate_his2:4;
		/* bit[15:12] LTSSM State history buffer 3 */
		unsigned ltstate_his3:4;
		/* bit[19:16] LTSSM State history buffer 4 */
		unsigned ltstate_his4:4;
		/* bit[30:20] */
		unsigned RSVD30_20:11;
		/* bit[31] Flag for the successful link training sequence (POLL=>U0) */
		unsigned linktrn_done:1;
	} b;
} USBCON_REG_LTSTATE_HIS_o, *USBCON_REG_LTSTATE_HIS_p;

#define USBCON_REG_LINK_DEBUG_L	 0x0084
typedef union {
	u32 data;
	struct {
		/* bit[31:0] Link internal design signal value for debugging
		 * Refer to the logic_analyzer_trace signal description in link databook */
		unsigned debug_l:32;
	} b;
} USBCON_REG_LINK_DEBUG_L_o, *USBCON_REG_LINK_DEBUG_L_p;

#define USBCON_REG_LINK_DEBUG_H	 0x0088
typedef union {
	u32 data;
	struct {
		/* bit[31:0] Link internal design signal value for debugging
		 * Refer to the logic_analyzer_trace signal description in link databook */
		unsigned debug_h:32;
	} b;
} USBCON_REG_LINK_DEBUG_H_o, *USBCON_REG_LINK_DEBUG_H_p;

#define USBCON_REG_HSP_MISC	 0x0114
typedef union {
	u32 data;
	struct {
		/* bit[1:0] selection of resistance tune chain
		 * 0: Phy2 -> Phy1
		 * 1: Phy1 -> Phy2
		 * 2: Phy1 only
		 * 3: Phy2 only */
		unsigned sel_res_tune_mux:2;
		/* bit[3:2] */
		unsigned RSVD3_2:2;
		/* bit[4] set value for req_in port of phy2 */
		unsigned set_req_in2:1;
		/* bit[5] set value for ack_in port of phy2 */
		unsigned set_ack_in2:1;
		/* bit[6] set value for req_in port of phy1 */
		unsigned set_req_in1:1;
		/* bit[7] set value for ack_in port of phy1 */
		unsigned set_ack_in1:1;
		/* bit[9:8] host_disconnect filtering mode
		 * 0: Disable Filtering
		 * 1: host_disconnect from phy is blocked
		 * 2: Normal Filtering - host_disconnect signal from phy can be transferred to link,
		 *    if host_disconnect drive high more than valid_hsdiscon_count during
		 *    mon_sof_count frames.
		 * 3: No limit of monitoring sof counter. Only if host_disconnect drive high more
		 *    than valid_hsdiscon_count, then host_disconnect signal from phy can be
		 *    transferred to link.
		 */
		unsigned hsdiscon_filter_mode:2;
		/* bit[11:10] */
		unsigned RSVD11_10:2;
		/* bit[12] */
		unsigned fsvplus:1;
		/* bit[13] */
		unsigned fsvminus:1;
		/* bit[14] */
		unsigned fsvp_out_en:1;
		/* bit[15] */
		unsigned fsvm_out_en:1;
		/* bit[23:16] host_disconnect monitoring period by number sof count */
		unsigned mon_sof_count:8;
		/* bit[31:24] count of asserted host_disconnect in monitoring frames */
		unsigned valid_hsdiscon_count:8;
	} b;
} USBCON_REG_HSP_MISC_o, *USBCON_REG_HSP_MISC_p;


#endif
