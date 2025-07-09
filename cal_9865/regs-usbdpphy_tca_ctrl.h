/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register definition file for Samsung DisplayPort driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _USBDPPHY_TCA_CTRL_REGS_H_
#define _USBDPPHY_TCA_CTRL_REGS_H_

//TCA : Type-C Assist block
/* USBDPPHY_TCA_CTRL base address : 0x11140000 */

#define TCA_REG_TCA_CLK_RST			   (0x0000)
#define TCA_REG_VBA_RST_SW			   (0x1 << 10)
#define TCA_REG_XA_RST_SW			   (0x1 << 9)
#define TCA_REG_PHY_RST_SW			   (0x1 << 8)
#define TCA_REG_TCA_CLK_AUTO_GATE_EN		   (0x1 << 2)
#define TCA_REG_TCA_REF_CLK_EN			   (0x1 << 1)
#define TCA_REG_SUSPEND_CLK_EN			   (0x1 << 0)

#define TCA_REG_TCA_INTR_EN			   (0x0004)
#define TCA_REG_SS_LANE1_ACTIVE_EVT_EN		   (0x1 << 16)
#define TCA_REG_TCA_DRV_HOST_VBUS_EVT_EN	   (0x1 << 12)
#define TCA_REG_TCA_VBUSVALID_EVT_EN		   (0x1 << 10)
#define TCA_REG_SYS_VBUSVALID_EVT_EN		   (0x1 << 8)
#define TCA_REG_XA_TIMEOUT_EVT_EN		   (0x1 << 1)
#define TCA_REG_XA_ACK_EVT_EN			   (0x1 << 0)

#define TCA_REG_TCA_INTR_STS			   (0x0008)
#define TCA_REG_SS_LANE1_ACTIVE			   (0x1 << 17)
#define TCA_REG_SS_LANE1_ACTIVE_EVT		   (0x1 << 16)
#define TCA_REG_TCA_DRV_HOST_VBUS		   (0x1 << 13)
#define TCA_REG_TCA_DRV_HOST_VBUS_EVT		   (0x1 << 12)
#define TCA_REG_TCA_VBUSVALID			   (0x1 << 11)
#define TCA_REG_TCA_VBUSVALID_EVT		   (0x1 << 10)
#define TCA_REG_SYS_VBUSVALID			   (0x1 << 9)
#define TCA_REG_SYS_VBUSVALID_EVT		   (0x1 << 8)
#define TCA_REG_XA_TIMEOUT_EVT			   (0x1 << 1)
#define TCA_REG_XA_ACK_EVT			   (0x1 << 0)

#define TCA_REG_TCA_GCFG			   (0x0010)
#define TCA_REG_PHYSAFE_RESET_EN		   (0x1 << 8)
#define TCA_REG_ROLE_HSTDEV			   (0x1 << 4)
#define TCA_REG_OP_MODE				   (0x3 << 0)

#define TCA_REG_TCA_TCPC			   (0x0014)
#define TCA_REG_TCPC_VALID_SET(_v)		   (((_v)&0x1) << 4)
#define TCA_REG_TCPC_VALID_MASK			   (0x1 << 4)
#define TCA_REG_TCPC_LOW_POWER_EN_SET(_v)	   (((_v)&0x1) << 3)
#define TCA_REG_TCPC_LOW_POWER_EN_MASK		   (0x1 << 3)
#define TCA_REG_TCPC_CONNECTOR_ORIENTATION_SET(_v) (((_v)&0x1) << 2)
#define TCA_REG_TCPC_CONNECTOR_ORIENTATION_MASK	   (0x1 << 2)
#define TCA_REG_TCPC_MUX_CONTROL_SET(_v)	   (((_v)&0x3) << 0)
#define TCA_REG_TCPC_MUX_CONTROL_MASK		   (0x3 << 0)

enum tcpc_mux_con {
	NO_CONNECTION = 0,
	USB31_CONNECTED = 1,
	DP_ALT4 = 2,
	USB31_DP_ALT2 = 3,
};

enum tcpc_orientation {
	NORMAL = 0,
	FLIPPED,
};

enum tcpc_low_pwr_en {
	STANDARD_OPERATION = 0,
	LOW_POWER = 1,
};

enum tca_tcpc_valid {
	VALID_I = 1,
};

#define TCA_REG_TCA_SYSMODE_CFG		     (0x0018)
#define TCA_REG_TYPEC_RESET		     (0x1 << 5)
#define TCA_REG_TYPEC_PHYSAFE		     (0x1 << 4)
#define TCA_REG_TYPEC_DISABLE		     (0x1 << 3)
#define TCA_REG_TYPEC_FLIP		     (0x1 << 2)
#define TCA_REG_TYPEC_CONN_MODE		     (0x3 << 0)

#define TCA_REG_TCA_CTRLSYNCMODE_CFG0	     (0x0020)
#define TCA_REG_AUTO_SAFE_STATE		     (0x1 << 16)
#define TCA_REG_DP_ACK_CHECK_BYPASS_EN	     (0x1 << 13)
#define TCA_REG_USB_ACK_CHECK_BYPASS_EN	     (0x1 << 12)
#define TCA_REG_DP_P2_SWITCH_EN		     (0x1 << 11)
#define TCA_REG_USB_P2_SWITCH_EN	     (0x1 << 10)
#define TCA_REG_DP_HDSHK_REQ		     (0x1 << 9)
#define TCA_REG_SS_HDSHK_REQ		     (0x1 << 8)
#define TCA_REG_SSTX_DETRX_REQ_BLOCK_EN	     (0x1 << 3)
#define TCA_REG_AUTO_BLOCK_SS_OP_EN	     (0x3 << 1)
#define TCA_REG_BLOCK_SS_OP		     (0x1 << 0)

#define TCA_REG_TCA_CTRLSYNCMODE_CFG1	     (0x0024)
#define TCA_REG_XA_TIMEOUT_VAL		     (0xFFFFF << 0)

#define TCA_REG_TCA_CTRLSYNCMODE_DBG0	     (0x0028)
#define TCA_REG_DBG0_CTRL_IF_OVRRD_VAL	     (0x1F << 25)
#define TCA_REG_DBG0_CTRL_IF_OVRRD_EN	     (0x1F << 20)
#define TCA_REG_DBG0_XA_REQ_COMPL_MULT	     (0x1F << 15)
#define TCA_REG_DBG0_PSTATE_SYNCED	     (0x1 << 13)
#define TCA_REG_DBG0_PSTATE_TIMEOUT_VAL	     (0x1 << 14)
#define TCA_REG_DBG0_XBAR_READY		     (0x1 << 12)
#define TCA_REG_DBG0_SS_LANE1_ACTIVE	     (0x1 << 10)
#define TCA_REG_DBG0_SS_LANE0_ACTIVE	     (0x1 << 9)
#define TCA_REG_DBG0_BLOCK_SS_OP	     (0x1 << 8)
#define TCA_REG_DBG0_SS_RXDET_DISABLE_ACK_1  (0x1 << 7)
#define TCA_REG_DBG0_SS_RXDET_DISABLE_1	     (0x1 << 6)
#define TCA_REG_DBG0_SS_RXDETECT_DISABLE_ACK (0x1 << 5)
#define TCA_REG_DBG0_SS_RXDETECT_DISABLE     (0x1 << 4)
#define TCA_REG_DBG0_DPALT_DISABLE_ACK	     (0x1 << 2)
#define TCA_REG_DBG0_DPALT_DISABLE	     (0x1 << 1)
#define TCA_REG_DBG0_DPALT_DP4		     (0x1 << 0)

#define TCA_REG_TCA_CTRLSYNCMODE_DBG1	     (0x002C)
#define TCA_REG_XA_REQ_COMPL_CYCLES_OVERRUN  (0x1 << 16)
#define TCA_REG_XA_REQ_COMPL_CYCLES	     (0xFFFF << 0)

#define TCA_REG_TCA_PSTATE		     (0x0030)
#define TCA_REG_DP_TX3_ACK		     (0x1 << 23)
#define TCA_REG_DP_TX3_REQ		     (0x1 << 22)
#define TCA_REG_DP_TX3_PSTATE		     (0x3 << 20)
#define TCA_REG_DP_TX2_ACK		     (0x1 << 19)
#define TCA_REG_DP_TX2_REQ		     (0x1 << 18)
#define TCA_REG_DP_TX2_PSTATE		     (0x3 << 16)
#define TCA_REG_DP_TX1_ACK		     (0x1 << 15)
#define TCA_REG_DP_TX1_REQ		     (0x1 << 14)
#define TCA_REG_DP_TX1_PSTATE		     (0x3 << 12)
#define TCA_REG_DP_TX0_ACK		     (0x1 << 11)
#define TCA_REG_DP_TX0_REQ		     (0x1 << 10)
#define TCA_REG_DP_TX0_PSTATE		     (0x3 << 8)
#define TCA_REG_SSTX_ACK		     (0x1 << 7)
#define TCA_REG_SSTX_REQ		     (0x1 << 6)
#define TCA_REG_SSTX_PSTATE		     (0x3 << 4)
#define TCA_REG_SSRX_ACK		     (0x1 << 3)
#define TCA_REG_SSRX_REQ		     (0x1 << 2)
#define TCA_REG_SSRX_PSTATE		     (0x3 << 0)

#define TCA_REG_TCA_GEN_STATUS		     (0x0034)
#define TCA_REG_DP4_POR			     (0x1 << 13)
#define TCA_REG_USB_DEV_POR		     (0x1 << 12)
#define TCA_REG_REF_CLK_SEL		     (0x1 << 8)
#define TCA_REG_TYPEC_FLIP_INVERT	     (0x1 << 4)
#define TCA_REG_PHY_TYPEC_DISABLE	     (0x1 << 3)
#define TCA_REG_PHY_TYPEC_FLIP		     (0x1 << 2)
#define TCA_REG_PHY_TYPEC_CONN_MODE	     (0x3 << 0)

#define TCA_REG_TCA_VBUS_CTRL		     (0x0040)
#define TCA_REG_TCA_MISC_CTRL		     (0x3F << 9)
#define TCA_REG_TCA_IDDIG		     (0x1 << 8)
#define TCA_REG_DRV_HOST_VBUS_OVERRD	     (0x3 << 4)
#define TCA_REG_POWERPRESENT_OVERRD	     (0x3 << 2)
#define TCA_REG_VBUSVALID_OVERRD	     (0x3 << 0)

#define TCA_REG_TCA_VBUS_STATUS		     (0x0044)
#define TCA_REG_TCA_DRV_HOST_VBUS_STATUS     (0x1 << 3)
#define TCA_REG_TCA_POWERPRESENT_STATUS	     (0x1 << 2)
#define TCA_REG_TCA_VBUSVALID_STATUS	     (0x1 << 1)
#define TCA_REG_SYS_VBUSVALID_STATUS	     (0x1 << 0)

#define TCA_REG_TCA_INFO_REG		     (0x00FC)
#define TCA_REG_VERSION_ID		     (0xFFFFFFFF << 0)

#endif /* _USBDPPHY_TCA_CTRL_REGS_H_ */
