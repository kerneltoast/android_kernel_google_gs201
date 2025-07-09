/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register definition file for Samsung USBDP Combo PHY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _USBDP_COMBO_PHY_REGS_H_
#define _USBDP_COMBO_PHY_REGS_H_

#define CMN_REG0008			       (0x0020)
#define OVRD_AUX_ENABLE			       (1 << 3)
#define OVRD_AUX_DISABLE		       (0 << 3)
#define OVRD_AUX_EN_MASK		       (0x1 << 3)
#define AUX_EN				       (1 << 2)
#define AUX_EN_MASK			       (0x1 << 2)

#define CMN_REG000A			       (0x0028)
#define ANA_AUX_TX_LVL_CTRL_SET(_v)	       (((_v)&0xF) << 3)
#define ANA_AUX_TX_LVL_CTRL_MASK	       (0xF << 3)
#define ANA_AUX_RX_VCM_P_CTRL_SET(_v)	       (((_v)&0x3) << 0)
#define ANA_AUX_RX_VCM_P_CTRL_MASK	       (0x3 << 0)

#define CMN_REG002E			       (0x00B8)
#define OVRD_LCPLL_REF_CLK_SEL		       (0x01 << 2)
#define LCPLL_REF_CLK_SEL		       (0x03 << 0)

#define CMN_REG0051			       (0x0144)
#define ROPLL_EN			       (0x1 << 6)

#define CMN_REG005E			       (0x0178)
#define ANA_ROPLL_ANA_VCI_SEL_SET(_v)	       (((_v)&0x7) << 1)
#define ANA_ROPLL_ANA_VCI_SEL_MASK	       (0x7 << 1)

// ROPLL_PMS_MDIV
#define CMN_REG0066			       (0x0198)
#define CMN_REG0067			       (0x019C)
#define CMN_REG0068			       (0x01A0)
#define CMN_REG0069			       (0x01A4)

// ROPLL_PMS_MDIV_AFC
#define CMN_REG006C			       (0x01B0)
#define CMN_REG006D			       (0x01B4)
#define CMN_REG006E			       (0x01B8)
#define CMN_REG006F			       (0x01BC)

// ROPLL_SDM_DENOMINATOR
#define CMN_REG007B			       (0x01EC)
#define CMN_REG007C			       (0x01F0)
#define CMN_REG007D			       (0x01F4)
#define CMN_REG007E			       (0x01F8)

// ROPLL_SDM_NUMERATOR
#define CMN_REG0082			       (0x0208)
#define CMN_REG0083			       (0x020C)
#define CMN_REG0084			       (0x0210)
#define CMN_REG0085			       (0x0214)

// ROPLL_SDC_N
#define CMN_REG0087			       (0x021C)
#define CMN_REG0088			       (0x0220)
#define CMN_REG0089			       (0x0224)

// ROPLL_SDC_NUMERATOR
#define CMN_REG008C			       (0x0230)
#define CMN_REG008D			       (0x0234)
#define CMN_REG008E			       (0x0238)
#define CMN_REG008F			       (0x023C)

// ROPLL_SDC_DENOMINATOR
#define CMN_REG0092			       (0x0248)
#define CMN_REG0093			       (0x024C)
#define CMN_REG0094			       (0x0250)
#define CMN_REG0095			       (0x0254)

// ROPLL_SSC_FM_DEVIATION
#define CMN_REG0099			       (0x0264)
#define CMN_REG009A			       (0x0268)
#define CMN_REG009B			       (0x026C)
#define CMN_REG009C			       (0x0270)

// ROPLL_SSC_FM_FREQ
#define CMN_REG009F			       (0x027C)
#define CMN_REG00A0			       (0x0280)
#define CMN_REG00A1			       (0x0284)
#define CMN_REG00A2			       (0x0288)

// DP TX Lane Configurations
#define CMN_REG00B8			       (0x02E0)
#define SSC_EN_SET(_v)			       (((_v)&0x1) << 5)
#define SSC_EN_MASK			       (0x1 << 5)
#define LANE_MUX_SEL_DP_LN3		       (1 << 3)
#define LANE_MUX_SEL_DP_LN2		       (1 << 2)
#define LANE_MUX_SEL_DP_LN1		       (1 << 1)
#define LANE_MUX_SEL_DP_LN0		       (1 << 0)
#define LANE_MUX_SEL_DP_MASK		       (0xF << 0)

#define CMN_REG00B9			       (0x02E4)
#define DP_TX_LINK_BW_GET(_v)		       (((_v) >> 4) & 0x3)
#define DP_TX_LINK_BW_SET(_v)		       (((_v)&0x3) << 4)
#define DP_TX_LINK_BW_MASK		       (0x3 << 4)
#define DP_LANE_EN_LN3			       (1 << 3)
#define DP_LANE_EN_LN2			       (1 << 2)
#define DP_LANE_EN_LN1			       (1 << 1)
#define DP_LANE_EN_LN0			       (1 << 0)
#define DP_LANE_EN_MASK			       (0xF << 0)

// DP Reset Controls
#define CMN_REG00BD			       (0x02F4)
#define DP_INIT_RSTN_RESET		       (0 << 1)
#define DP_INIT_RSTN_ACTIVE		       (1 << 1)
#define DP_INIT_RSTN_SET(_v)		       (((_v)&0x1) << 1)
#define DP_INIT_RSTN_MASK		       (0x1 << 1)
#define DP_CMN_RSTN_RESET		       (0 << 0)
#define DP_CMN_RSTN_ACTIVE		       (1 << 0)
#define DP_CMN_RSTN_SET(_v)		       (((_v)&0x1) << 0)
#define DP_CMN_RSTN_MASK		       (0x1 << 0)

// DP TX
#define CMN_REG0189			       (0x0624)
#define DP_TX_CLK_INV_RBR_SET(_v)	       (((_v)&0xF) << 4)
#define DP_TX_CLK_INV_RBR_MASK		       (0xF << 4)
#define DP_TX_CLK_INV_HBR_SET(_v)	       (((_v)&0xF) << 0)
#define DP_TX_CLK_INV_HBR_MASK		       (0xF << 0)

#define CMN_REG018A			       (0x0628)
#define DP_TX_CLK_INV_HBR2_SET(_v)	       (((_v)&0xF) << 4)
#define DP_TX_CLK_INV_HBR2_MASK		       (0xF << 4)
#define DP_TX_CLK_INV_HBR3_SET(_v)	       (((_v)&0xF) << 0)
#define DP_TX_CLK_INV_HBR3_MASK		       (0xF << 0)

#define CMN_REG0194			       (0x0650)
#define ANA_ROPLL_CLK_OUT_TO_EXT_IO_EN_SET(_v) (((_v)&0x1) << 4)
#define ANA_ROPLL_CLK_OUT_TO_EXT_IO_EN_MASK    (0x1 << 4)

#define CMN_REG00B1			       (0x02C4)
#define ANA_ROPLL_RESERVED_SET(_v)	       (((_v)&0xFF) << 0)
#define ANA_ROPLL_RESERVED_MASK		       (0xFF << 0)

#define CMN_REG00AC			       (0x02B0)
#define ROPLL_MISC_CLK_DIV_RBR_SET(_v)	       (((_v)&0x1F) << 0)
#define ROPLL_MISC_CLK_DIV_RBR_MASK	       (0x1F << 0)

#define CMN_REG00AD			       (0x02B4)
#define ROPLL_MISC_CLK_DIV_HBR_SET(_v)	       (((_v)&0x1F) << 0)
#define ROPLL_MISC_CLK_DIV_HBR_MASK	       (0x1F << 0)

#define CMN_REG00AE			       (0x02B8)
#define ROPLL_MISC_CLK_DIV_HBR2_SET(_v)	       (((_v)&0x1F) << 0)
#define ROPLL_MISC_CLK_DIV_HBR2_MASK	       (0x1F << 0)

#define CMN_REG00AF			       (0x02BC)
#define ROPLL_MISC_CLK_DIV_HBR3_SET(_v)	       (((_v)&0x1F) << 0)
#define ROPLL_MISC_CLK_DIV_HBR3_MASK	       (0x1F << 0)

#define CMN_REG0105			       (0x0414)
#define DP_LINK_DIV10_CLK_OUT_SEL_SET(_v)      (((_v)&0x3) << 0)
#define DP_LINK_DIV10_CLK_OUT_SEL_MASK	       (0x3 << 0)

#define CMN_REG01C1			       (0x0704)
#define ANA_ROPLL_LOCK_DONE		       (0x01 << 1)
#define ANA_ROPLL_AFC_DONE		       (0x01 << 0)

#define CMN_REG01C3			       (0x070C)
#define MON_CMN_STATE			       (0x1F << 0)

#define CMN_REG01C4			       (0x0710)
#define MON_CMN_TIME__14_8		       (0x7F << 0)

#define CMN_REG01C5			       (0x0714)
#define MON_CMN_TIME__7_0		       (0xFF << 0)

#define CMN_REG01C9			       (0x0724)
#define MON_ROPLL_FLD_VCO_CNT_MON_VALUE__11_8  (0xF << 0)

#define CMN_REG01CA			       (0x0728)
#define MON_ROPLL_FLD_VCO_CNT_MON_VALUE__7_0   (0xFF << 0)

// Lane0 TX Driver Level
#define TRSV_REG0204			       (0x0810)
#define TRSV_REG0205			       (0x0814)
#define TRSV_REG0206			       (0x0818)
#define TRSV_REG0207			       (0x081C)
#define TRSV_REG0208			       (0x0820)

// Lane1 TX Driver Level
#define TRSV_REG0404			       (0x1010)
#define TRSV_REG0405			       (0x1014)
#define TRSV_REG0406			       (0x1018)
#define TRSV_REG0407			       (0x101C)
#define TRSV_REG0408			       (0x1020)

#define TRSV_REG0413			       (0x104C)
#define OVRD_LN1_TX_RXD_COMP_EN		       (0x1 << 7)
#define OVRD_LN1_TX_RXD_EN		       (0x1 << 5)
#define LN1_TX_SER_40BIT_EN_SP		       (0x1 << 0)

// Lane2 TX Driver Level
#define TRSV_REG0604			       (0x1810)
#define TRSV_REG0605			       (0x1814)
#define TRSV_REG0606			       (0x1818)
#define TRSV_REG0607			       (0x181C)
#define TRSV_REG0608			       (0x1820)

// Lane3 TX Driver Level
#define TRSV_REG0804			       (0x2010)
#define TRSV_REG0805			       (0x2014)
#define TRSV_REG0806			       (0x2018)
#define TRSV_REG0807			       (0x201C)
#define TRSV_REG0808			       (0x2020)

#define TRSV_REG0813			       (0x204C)
#define OVRD_LN3_TX_RXD_COMP_EN		       (0x1 << 7)
#define OVRD_LN3_TX_RXD_EN		       (0x1 << 5)
#define LN3_TX_SER_40BIT_EN_SP		       (0x1 << 0)

#endif /* _USBDP_COMBO_PHY_REGS_H_ */
