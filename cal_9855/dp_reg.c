// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register access functions for Display Port driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <cal_config.h>
#include <dp_cal.h>

#include "regs-dp.h"
#include "regs-usbdpphy.h"

/* DP register access functions */
static struct cal_regs_desc regs_dp[REGS_DP_TYPE_MAX][SST_MAX];

#define dp_regs_desc(id)	  (&regs_dp[REGS_LINK][id])
#define dp_read(id, offset)	  cal_read(dp_regs_desc(id), offset)
#define dp_write(id, offset, val) cal_write(dp_regs_desc(id), offset, val)
#define dp_read_mask(id, offset, mask)                                         \
	cal_read_mask(dp_regs_desc(id), offset, mask)
#define dp_write_mask(id, offset, val, mask)                                   \
	cal_write_mask(dp_regs_desc(id), offset, val, mask)

#define dp_phy_regs_desc(id)	(&regs_dp[REGS_PHY][id])
#define dp_phy_read(id, offset) cal_read(dp_phy_regs_desc(id), offset)
#define dp_phy_write(id, offset, val)                                          \
	cal_write(dp_phy_regs_desc(id), offset, val)
#define dp_phy_read_mask(id, offset, mask)                                     \
	cal_read_mask(dp_phy_regs_desc(id), offset, mask)
#define dp_phy_write_mask(id, offset, val, mask)                               \
	cal_write_mask(dp_phy_regs_desc(id), offset, val, mask)

/* DP memory map interface */
void dp_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		       enum dp_regs_type type, unsigned int id)
{
	cal_regs_desc_check(type, id, REGS_DP_TYPE_MAX, SST_MAX);
	cal_regs_desc_set(regs_dp, regs, start, name, type, id);
}

u32 phy_tune_parameters[4][4][5] = {
	/* {amp, post, pre, idrv, accdrv} */
	{
		/* Swing Level_0 */
		{ 0x22, 0x10, 0x42, 0x82, 0x00 }, /* Pre-emphasis Level_0 */
		{ 0x26, 0x15, 0x42, 0x82, 0x00 }, /* Pre-emphasis Level_1 */
		{ 0x26, 0x17, 0x43, 0x82, 0x00 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x1C, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_1 */
		{ 0x26, 0x10, 0x42, 0x82, 0x00 }, /* Pre-emphasis Level_0 */
		{ 0x2B, 0x14, 0x42, 0x83, 0x30 }, /* Pre-emphasis Level_1 */
		{ 0x2B, 0x18, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x18, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_2 */
		{ 0x2A, 0x10, 0x42, 0x83, 0x30 }, /* Pre-emphasis Level_0 */
		{ 0x2B, 0x17, 0x43, 0x83, 0x38 }, /* Pre-emphasis Level_1 */
		{ 0x2B, 0x17, 0x43, 0x83, 0x38 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x17, 0x43, 0x83, 0x38 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_3 */
		{ 0x2B, 0x10, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_0 */
		{ 0x2B, 0x10, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_1 */
		{ 0x2B, 0x10, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x10, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_3 */
	},
};

static u32 m_aud_master[7] = {
	32000, 44100, 48000, 88200, 96000, 176000, 192000
};
static u32 n_aud_master[4] = { 81000000, 135000000, 270000000, 405000000 };

static u32 audio_async_m_n[2][4][7] = {
	{
		/* M value set */
		{ 3314, 4567, 4971, 9134, 9942, 18269, 19884 },
		{ 1988, 2740, 2983, 5481, 5695, 10961, 11930 },
		{ 994, 1370, 1491, 2740, 2983, 5481, 5965 },
		{ 663, 913, 994, 1827, 1988, 3654, 3977 },
	},
	{
		/* N value set */
		{ 32768, 32768, 32768, 32768, 32768, 32768, 32768 },
		{ 32768, 32768, 32768, 32768, 32768, 32768, 32768 },
		{ 32768, 32768, 32768, 32768, 32768, 32768, 32768 },
		{ 32768, 32768, 32768, 32768, 32768, 32768, 32768 },
	}
};

static u32 audio_sync_m_n[2][4][7] = {
	{
		/* M value set */
		{ 1024, 784, 512, 1568, 1024, 3136, 2048 },
		{ 1024, 784, 512, 1568, 1024, 3136, 2048 },
		{ 1024, 784, 512, 784, 512, 1568, 1024 },
		{ 1024, 784, 512, 1568, 1024, 3136, 2048 },
	},
	{
		/* N value set */
		{ 10125, 5625, 3375, 5625, 3375, 5625, 3375 },
		{ 16875, 9375, 5625, 9375, 5625, 9375, 5625 },
		{ 33750, 18750, 11250, 9375, 5625, 9375, 5625 },
		{ 50625, 28125, 16875, 28125, 16875, 28125, 16875 },
	}
};

/*
 * USBDP Combo PHY Control Functions
 */

/* USBDP PHY Common Registers */
static void dpphy_reg_reset(u32 en)
{
	dp_phy_write_mask(SST1, CMN_REG00BD, DP_INIT_RSTN_SET(~en),
			  DP_INIT_RSTN_MASK);
	udelay(2);
	dp_phy_write_mask(SST1, CMN_REG00BD, DP_CMN_RSTN_SET(~en),
			  DP_CMN_RSTN_MASK);
	udelay(2);
}

static void dpphy_reg_set_default_config(struct dp_hw_config *hw_config)
{
	dp_phy_write_mask(SST1, CMN_REG0008, OVRD_AUX_DISABLE,
			  OVRD_AUX_EN_MASK);

	/* DP Default Settings */
	dp_phy_write_mask(SST1, CMN_REG0189, DP_TX_CLK_INV_RBR_SET(15),
			  DP_TX_CLK_INV_RBR_MASK);
	dp_phy_write_mask(SST1, CMN_REG0189, DP_TX_CLK_INV_HBR_SET(15),
			  DP_TX_CLK_INV_HBR_MASK);
	dp_phy_write_mask(SST1, CMN_REG018A, DP_TX_CLK_INV_HBR2_SET(15),
			  DP_TX_CLK_INV_HBR2_MASK);
	dp_phy_write_mask(SST1, CMN_REG018A, DP_TX_CLK_INV_HBR3_SET(15),
			  DP_TX_CLK_INV_HBR3_MASK);
	dp_phy_write_mask(SST1, CMN_REG005E, ANA_ROPLL_ANA_VCI_SEL_SET(1),
			  ANA_ROPLL_ANA_VCI_SEL_MASK);

	/* ROPLL for DP Settings for 19.2MHz Ref.Clk */
	// ROPLL_PMS_MDIV
	dp_phy_write(SST1, CMN_REG0066, 0xD4);
	dp_phy_write(SST1, CMN_REG0067, 0x8E);
	dp_phy_write(SST1, CMN_REG0068, 0x8E);
	dp_phy_write(SST1, CMN_REG0069, 0xD4);
	// ROPLL_PMS_MDIV_AFC
	dp_phy_write(SST1, CMN_REG006C, 0xD4);
	dp_phy_write(SST1, CMN_REG006D, 0x8E);
	dp_phy_write(SST1, CMN_REG006E, 0x8E);
	dp_phy_write(SST1, CMN_REG006F, 0xD4);
	// ROPLL_SDM_DENOMINATOR
	dp_phy_write(SST1, CMN_REG007B, 0x2C);
	dp_phy_write(SST1, CMN_REG007C, 0x14);
	dp_phy_write(SST1, CMN_REG007D, 0x14);
	dp_phy_write(SST1, CMN_REG007E, 0x2C);
	// ROPLL_SDM_NUMERATOR
	dp_phy_write(SST1, CMN_REG0082, 0x11);
	dp_phy_write(SST1, CMN_REG0083, 0x0A);
	dp_phy_write(SST1, CMN_REG0084, 0x0A);
	dp_phy_write(SST1, CMN_REG0085, 0x11);
	// ROPLL_SDC_N
	dp_phy_write(SST1, CMN_REG0087, 0x01);
	dp_phy_write(SST1, CMN_REG0088, 0x00);
	dp_phy_write(SST1, CMN_REG0089, 0x03);
	// ROPLL_SDC_NUMERATOR
	dp_phy_write(SST1, CMN_REG008C, 0x02);
	dp_phy_write(SST1, CMN_REG008D, 0x09);
	dp_phy_write(SST1, CMN_REG008E, 0x09);
	dp_phy_write(SST1, CMN_REG008F, 0x02);
	// ROPLL_SDC_DENOMINATOR
	dp_phy_write(SST1, CMN_REG0092, 0x0A);
	dp_phy_write(SST1, CMN_REG0093, 0x14);
	dp_phy_write(SST1, CMN_REG0094, 0x14);
	dp_phy_write(SST1, CMN_REG0095, 0x0A);

	if (hw_config->use_ssc) {
		// ROPLL_SSC_FM_DEVIATION
		// ROPLL_SSC_FM_FREQ
	}
}

static void dpphy_reg_set_link_bw(enum dp_link_rate_type link_rate)
{
	dp_phy_write_mask(SST1, CMN_REG00B9, DP_TX_LINK_BW_SET(link_rate),
			  DP_TX_LINK_BW_MASK);
}

static void dpphy_reg_set_lane_config(struct dp_hw_config *hw_config)
{
	u32 lane_mux_config = 0;
	u32 lane_en_config = 0;
	u32 lane1_config = 0;
	u32 lane3_config = 0;

	switch (hw_config->pin_type) {
	case PIN_TYPE_A:
	case PIN_TYPE_C:
	case PIN_TYPE_E:
		/* 4 Lanes Mode Configuration */
		lane_mux_config = LANE_MUX_SEL_DP_LN3 | LANE_MUX_SEL_DP_LN2 |
				  LANE_MUX_SEL_DP_LN1 | LANE_MUX_SEL_DP_LN0;
		lane_en_config = DP_LANE_EN_LN3 | DP_LANE_EN_LN2 |
				 DP_LANE_EN_LN1 | DP_LANE_EN_LN0;

		lane1_config = OVRD_LN1_TX_RXD_COMP_EN | OVRD_LN1_TX_RXD_EN |
			       LN1_TX_SER_40BIT_EN_SP;
		lane3_config = OVRD_LN3_TX_RXD_COMP_EN | OVRD_LN3_TX_RXD_EN |
			       LN3_TX_SER_40BIT_EN_SP;
		break;

	case PIN_TYPE_B:
	case PIN_TYPE_D:
	case PIN_TYPE_F:
		/* 2 Lanes Mode Configuration */
		if (hw_config->orient_type == PLUG_NORMAL) {
			lane_mux_config =
				LANE_MUX_SEL_DP_LN3 | LANE_MUX_SEL_DP_LN2;
			lane_en_config = DP_LANE_EN_LN3 | DP_LANE_EN_LN2;

			lane1_config = LN1_TX_SER_40BIT_EN_SP;
			lane3_config = OVRD_LN3_TX_RXD_COMP_EN |
				       OVRD_LN3_TX_RXD_EN |
				       LN3_TX_SER_40BIT_EN_SP;
		} else {
			lane_mux_config =
				LANE_MUX_SEL_DP_LN1 | LANE_MUX_SEL_DP_LN0;
			lane_en_config = DP_LANE_EN_LN1 | DP_LANE_EN_LN0;

			lane1_config = OVRD_LN1_TX_RXD_COMP_EN |
				       OVRD_LN1_TX_RXD_EN |
				       LN1_TX_SER_40BIT_EN_SP;
			lane3_config = LN3_TX_SER_40BIT_EN_SP;
		}
		break;

	default:
		break;
	}

	dp_phy_write_mask(SST1, CMN_REG00B8, lane_mux_config,
			  LANE_MUX_SEL_DP_MASK);
	dp_phy_write_mask(SST1, CMN_REG00B9, lane_en_config, DP_LANE_EN_MASK);

	dp_phy_write_mask(SST1, TRSV_REG0413, lane1_config,
			  OVRD_LN1_TX_RXD_COMP_EN | OVRD_LN1_TX_RXD_EN |
				  LN1_TX_SER_40BIT_EN_SP);
	dp_phy_write_mask(SST1, TRSV_REG0813, lane3_config,
			  OVRD_LN3_TX_RXD_COMP_EN | OVRD_LN3_TX_RXD_EN |
				  LN3_TX_SER_40BIT_EN_SP);
}

static void dpphy_reg_set_ssc(u32 en)
{
	dp_phy_write_mask(SST1, CMN_REG00B8, SSC_EN_SET(en), SSC_EN_MASK);
}

/* USBDP PHY TRSV Registers */
static void dpphy_reg_set_lane0_tx_level(u32 amp_lvl, u32 pre_emp_lvl)
{
	dp_phy_write(SST1, TRSV_REG0204,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][0]);
	dp_phy_write(SST1, TRSV_REG0205,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][1]);
	dp_phy_write(SST1, TRSV_REG0206,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][2]);
	dp_phy_write(SST1, TRSV_REG0207,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][3]);
	dp_phy_write(SST1, TRSV_REG0208,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][4]);
}

static void dpphy_reg_set_lane1_tx_level(u32 amp_lvl, u32 pre_emp_lvl)
{
	dp_phy_write(SST1, TRSV_REG0404,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][0]);
	dp_phy_write(SST1, TRSV_REG0405,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][1]);
	dp_phy_write(SST1, TRSV_REG0406,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][2]);
	dp_phy_write(SST1, TRSV_REG0407,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][3]);
	dp_phy_write(SST1, TRSV_REG0408,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][4]);
}

static void dpphy_reg_set_lane2_tx_level(u32 amp_lvl, u32 pre_emp_lvl)
{
	dp_phy_write(SST1, TRSV_REG0604,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][0]);
	dp_phy_write(SST1, TRSV_REG0605,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][1]);
	dp_phy_write(SST1, TRSV_REG0606,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][2]);
	dp_phy_write(SST1, TRSV_REG0607,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][3]);
	dp_phy_write(SST1, TRSV_REG0608,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][4]);
}

static void dpphy_reg_set_lane3_tx_level(u32 amp_lvl, u32 pre_emp_lvl)
{
	dp_phy_write(SST1, TRSV_REG0804,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][0]);
	dp_phy_write(SST1, TRSV_REG0805,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][1]);
	dp_phy_write(SST1, TRSV_REG0806,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][2]);
	dp_phy_write(SST1, TRSV_REG0807,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][3]);
	dp_phy_write(SST1, TRSV_REG0808,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][4]);
}

/* USBDP PHY functions */
static void dpphy_reg_init(struct dp_hw_config *hw_config)
{
	/* Hold PHY soft reset */
	dpphy_reg_reset(1);

	/* USBDP PHY Settings */
	dpphy_reg_set_default_config(hw_config);
	dpphy_reg_set_link_bw(hw_config->link_rate);
	dpphy_reg_set_lane_config(hw_config);
	dpphy_reg_set_ssc(hw_config->use_ssc ? 1 : 0);

	/* wait for 60us */
	udelay(60);

	/* Release PHY soft reset */
	dpphy_reg_reset(0);
}

/*
 * DP Link Control Functions
 */

/* System Common Registers */
// System Device Version
static u32 dp_reg_get_version(void)
{
	return dp_read(SST1, SYSTEM_DEVICE_VERSION);
}

// System SW Reset Control Configuration
static void dp_reg_set_swreset(void)
{
	u32 val;

	dp_write_mask(SST1, SYSTEM_SW_RESET_CONTROL, SW_RESET, SW_RESET_MASK);

	if (readl_poll_timeout_atomic(regs_dp[REGS_LINK][SST1].regs +
					      SYSTEM_SW_RESET_CONTROL,
				      val, !SW_RESET_GET(val), 10, 2000))
		cal_log_err(0, "%s is timeout.\n", __func__);
}

// System Clock Control Configuration
static u32 dp_reg_get_gfmux_status(void)
{
	return GFMUX_STATUS_TXCLK_GET(dp_read_mask(SST1, SYSTEM_CLK_CONTROL,
						   GFMUX_STATUS_TXCLK_MASK));
}

static void dp_reg_set_txclk_osc(void)
{
	dp_write_mask(SST1, SYSTEM_CLK_CONTROL, TXCLK_SEL_OSCCLK,
		      TXCLK_SEL_MASK);
}

static void dp_reg_set_txclk_txclk(void)
{
	dp_write_mask(SST1, SYSTEM_CLK_CONTROL, TXCLK_SEL_TXCLK,
		      TXCLK_SEL_MASK);
}

static void dp_reg_set_txclk_sel_mode_by_txclksel(void)
{
	dp_write_mask(SST1, SYSTEM_CLK_CONTROL, TXCLK_SEL_MODE_BY_TXCLK_SEL,
		      TXCLK_SEL_MODE_MASK);
}

// System Main Link Lane Count Configuration
static void dp_reg_set_lane_count(u8 lane_cnt)
{
	dp_write_mask(SST1, SYSTEM_MAIN_LINK_LANE_COUNT,
		      LANE_COUNT_SET(lane_cnt), LANE_COUNT_SET_MASK);
}

// System SW Function Enable Configuration
static void dp_reg_set_sw_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_SW_FUNCTION_ENABLE, SW_FUNC_EN_SET(en),
		      SW_FUNC_EN_MASK);
}

// System Common Module Functions Enable Configuration
static void dp_reg_set_hdcp22_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE,
		      HDCP22_FUNC_EN_SET(en), HDCP22_FUNC_EN_MASK);
}

static void dp_reg_set_hdcp13_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE,
		      HDCP13_FUNC_EN_SET(en), HDCP13_FUNC_EN_MASK);
}

static void dp_reg_set_gtc_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE, GTC_FUNC_EN_SET(en),
		      GTC_FUNC_EN_MASK);
}

static void dp_reg_set_pcs_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE, PCS_FUNC_EN_SET(en),
		      PCS_FUNC_EN_MASK);
}

static void dp_reg_set_aux_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE, AUX_FUNC_EN_SET(en),
		      AUX_FUNC_EN_MASK);
}

static void dp_hw_set_initial_common_funcs(void)
{
	dp_reg_set_hdcp22_func_en(0);
	dp_reg_set_hdcp13_func_en(0);
	dp_reg_set_gtc_func_en(0);
	dp_reg_set_pcs_func_en(0);
	dp_reg_set_aux_func_en(1);
}

// System SST1 Function Enable Configuration
static int dp_reg_wait_sst1_longhop_power_on(void)
{
	u32 val;

	if (readl_poll_timeout_atomic(
		    regs_dp[REGS_LINK][SST1].regs + SYSTEM_SST1_FUNCTION_ENABLE,
		    val, SST1_LH_PWR_ON_STATUS_GET(val), 10, 200)) {
		cal_log_err(0, "%s is timeout.\n", __func__);
		return -ETIME;
	} else
		return 0;
}

static void dp_reg_set_sst1_longhop_power(u32 on)
{
	dp_write_mask(SST1, SYSTEM_SST1_FUNCTION_ENABLE, SST1_LH_PWR_ON_SET(on),
		      SST1_LH_PWR_ON_MASK);
}

static void dp_reg_set_sst1_audio_fifo_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_SST1_FUNCTION_ENABLE,
		      SST1_AUDIO_FIFO_FUNC_EN_SET(en),
		      SST1_AUDIO_FIFO_FUNC_EN_MASK);
}

static void dp_reg_set_sst1_audio_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_SST1_FUNCTION_ENABLE,
		      SST1_AUDIO_FUNC_EN_SET(en), SST1_AUDIO_FUNC_EN_MASK);
}

static void dp_reg_set_sst1_video_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_SST1_FUNCTION_ENABLE,
		      SST1_VIDEO_FUNC_EN_SET(en), SST1_VIDEO_FUNC_EN_MASK);
}

// System Miscellaneous Control Configuration

// System HPD Control Configuration

// System PLL Lock Control Configuration
static int dp_reg_wait_phy_pll_lock(void)
{
	u32 val;

	if (readl_poll_timeout_atomic(regs_dp[REGS_LINK][SST1].regs +
					      SYSTEM_PLL_LOCK_CONTROL,
				      val, PLL_LOCK_STATUS_GET(val), 10, 200)) {
		cal_log_err(0, "%s is timeout.\n", __func__);
		return -ETIME;
	} else
		return 0;
}

/* OSC Clock Divider Control Registers */
static void dp_reg_set_osc_clk_div(unsigned int mhz)
{
	dp_write_mask(SST1, OSC_CLK_DIV_HPD, HPD_EVENT_CLK_COUNT_SET(mhz),
		      HPD_EVENT_CLK_COUNT_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_HDCP_10US, I2C_GEN10US_TIMER_SET(mhz),
		      I2C_GEN10US_TIMER_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_GTC_1MS, GTC_1MS_OSC_CLK_COUNT_SET(mhz),
		      GTC_1MS_OSC_CLK_COUNT_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_AUX_1US, AUX_1US_OSC_CLK_COUNT_SET(mhz),
		      AUX_1US_OSC_CLK_COUNT_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_AUX_MAN_UI,
		      AUX_MAN_UI_OSC_CLK_COUNT_SET(mhz),
		      AUX_MAN_UI_OSC_CLK_COUNT_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_AUX_10US,
		      AUX_10US_OSC_CLK_COUNT_SET(mhz),
		      AUX_10US_OSC_CLK_COUNT_MASK);
}

/* System Interrupt Registers */
static void dp_reg_get_interrupt_source(bool *common, bool *str1, bool *pcs)
{
	u32 val = dp_read(SST1, SYSTEM_INTERRUPT_REQUEST);

	*common = IRQ_COMMON_GET(val) ? true : false;
	*str1 = IRQ_STR1_GET(val) ? true : false;
	*pcs = IRQ_PCS_GET(val) ? true : false;
}

static u32 dp_reg_get_common_interrupt(void)
{
	return dp_read(SST1, SYSTEM_IRQ_COMMON_STATUS);
}

static void dp_reg_set_common_interrupt_clear(u32 val)
{
	dp_write(SST1, SYSTEM_IRQ_COMMON_STATUS, val);
}

static void dp_reg_set_common_interrupt_mask(u32 en, u32 intr_mask)
{
	if (en)
		dp_write_mask(SST1, SYSTEM_IRQ_COMMON_STATUS_MASK, intr_mask,
			      intr_mask);
	else
		dp_write_mask(SST1, SYSTEM_IRQ_COMMON_STATUS_MASK, 0,
			      intr_mask);
}

static void dp_hw_set_common_interrupt(u32 en)
{
	// Mask all common interrupts
	dp_reg_set_common_interrupt_mask(0, ~0);

	if (en) {
		// HDCP Interrupts

		// HPD Interrupts : Not used

		// AUX Interrupts

		// PLL Interrupt

		// SW Interrupt
	}
}

/* AUX Control Registers */
static void dp_reg_set_aux_reply_timeout(void)
{
	dp_write_mask(SST1, AUX_CONTROL, AUX_REPLY_TIMER_MODE_1800US,
		      AUX_REPLY_TIMER_MODE_MASK);
}

static void dp_reg_set_aux_pn(enum plug_orientation orient)
{
	u32 val = (orient == PLUG_FLIPPED) ? AUX_PN_INVERT : AUX_PN_NORMAL;

	dp_write_mask(SST1, AUX_CONTROL, val, AUX_PN_INV_MASK);
}

static int dp_reg_do_aux_transaction(void)
{
	u32 val;

	/*
	 * When write 1 to AUX_TRAN_START reg, AUX transaction starts.
	 * Once AUX transaction is completed, AUX_TRAN_START reg is auto cleared.
	 */
	dp_write_mask(SST1, AUX_TRANSACTION_START, AUX_TRAN_START,
		      AUX_TRAN_START_MASK);

	if (readl_poll_timeout_atomic(
		    regs_dp[REGS_LINK][SST1].regs + AUX_TRANSACTION_START, val,
		    !AUX_TRAN_START_GET(val), 10, 50000)) {
		cal_log_err(0, "%s is timeout.\n", __func__);
		return -ETIME;
	} else
		return 0;
}

static void dp_reg_aux_ch_buf_clr(void)
{
	dp_write_mask(SST1, AUX_BUFFER_CLEAR, AUX_BUF_CLR, AUX_BUF_CLR_MASK);
}

static void dp_reg_set_aux_ch_address_only_command(bool en)
{
	u32 val = en ? ADDR_ONLY_CMD : NORMAL_AUX_CMD;

	dp_write_mask(SST1, AUX_ADDR_ONLY_COMMAND, val, ADDR_ONLY_CMD_MASK);
}

static void dp_reg_set_aux_ch_command(enum dp_aux_ch_command_type aux_ch_mode)
{
	dp_write_mask(SST1, AUX_REQUEST_CONTROL, REQ_COMM_SET(aux_ch_mode),
		      REQ_COMM_MASK);
}

static void dp_reg_set_aux_ch_address(u32 aux_ch_address)
{
	dp_write_mask(SST1, AUX_REQUEST_CONTROL, REQ_ADDR_SET(aux_ch_address),
		      REQ_ADDR_MASK);
}

static void dp_reg_set_aux_ch_length(u32 aux_ch_length)
{
	dp_write_mask(SST1, AUX_REQUEST_CONTROL,
		      REQ_LENGTH_SET(aux_ch_length - 1), REQ_LENGTH_MASK);
}

static void dp_reg_aux_defer_ctrl(u32 en)
{
	u32 val = en ? ~0 : 0;

	dp_write_mask(SST1, AUX_COMMAND_CONTROL, DEFER_CTRL_EN_SET(val),
		      DEFER_CTRL_EN_MASK);
}

static int dp_reg_check_aux_monitors(void)
{
	u32 val0 = dp_read(SST1, AUX_MONITOR_1);
	u32 val1 = dp_read(SST1, AUX_MONITOR_2);

	if ((AUX_CMD_STATUS_GET(val0) != AUX_CMD_STATUS_OK) || val1) {
		cal_log_err(0, "AUX_MONITOR_1 : 0x%X, AUX_MONITOR_2 : 0x%X\n",
			    val0, val1);
		cal_log_err(
			0,
			"AUX_CONTROL : 0x%X, AUX_REQUEST_CONTROL : 0x%X, AUX_COMMAND_CONTROL : 0x%X\n",
			dp_read(SST1, AUX_CONTROL),
			dp_read(SST1, AUX_REQUEST_CONTROL),
			dp_read(SST1, AUX_COMMAND_CONTROL));

		usleep_range(400, 410);
		return -EIO;
	} else
		return 0;
}

static void dp_reg_aux_ch_send_buf(u8 *buf, u32 length)
{
	int i;

	for (i = 0; i < length; i++)
		dp_write_mask(SST1, AUX_TX_DATA_SET(i), TX_DATA_SET(buf[i], i),
			      TX_DATA_MASK(i));
}

static void dp_reg_aux_ch_received_buf(u8 *buf, u32 length)
{
	int i;

	for (i = 0; i < length; i++)
		buf[i] = RX_DATA_GET(dp_read_mask(SST1, AUX_RX_DATA_SET(i),
						  RX_DATA_MASK(i)),
				     i);
}

/* GTC (Global Time Code) Control Registers */

/* PCS (Scrambler/Encoder/FEC) Control Registers */
// PCS Control
static void dp_reg_set_training_pattern(dp_training_pattern pattern)
{
	dp_write_mask(SST1, PCS_CONTROL, LINK_TRAINING_PATTERN_SET(pattern),
		      LINK_TRAINING_PATTERN_MASK);
}

static void dp_reg_set_scramble_bypass(u32 en)
{
	dp_write_mask(SST1, PCS_CONTROL, SCRAMBLE_BYPASS_SET(en),
		      SCRAMBLE_BYPASS_MASK);
}

// PCS Lane Control
static void dp_reg_set_lane_map(u32 lane0, u32 lane1, u32 lane2, u32 lane3)
{
	dp_write_mask(SST1, PCS_LANE_CONTROL, LANE0_MAP_SET(lane0),
		      LANE0_MAP_MASK);
	dp_write_mask(SST1, PCS_LANE_CONTROL, LANE1_MAP_SET(lane1),
		      LANE1_MAP_MASK);
	dp_write_mask(SST1, PCS_LANE_CONTROL, LANE2_MAP_SET(lane2),
		      LANE2_MAP_MASK);
	dp_write_mask(SST1, PCS_LANE_CONTROL, LANE3_MAP_SET(lane3),
		      LANE3_MAP_MASK);
}

static void dp_hw_set_lane_map_config(struct dp_hw_config *hw_config)
{
	switch (hw_config->pin_type) {
	case PIN_TYPE_A:
		if (hw_config->orient_type == PLUG_NORMAL)
			dp_reg_set_lane_map(LOGICAL_LANE_3, LOGICAL_LANE_1,
					    LOGICAL_LANE_2, LOGICAL_LANE_0);
		else
			dp_reg_set_lane_map(LOGICAL_LANE_2, LOGICAL_LANE_0,
					    LOGICAL_LANE_3, LOGICAL_LANE_1);
		break;

	case PIN_TYPE_B:
		if (hw_config->orient_type == PLUG_NORMAL)
			dp_reg_set_lane_map(LOGICAL_LANE_3, LOGICAL_LANE_2,
					    LOGICAL_LANE_1, LOGICAL_LANE_0);
		else
			dp_reg_set_lane_map(LOGICAL_LANE_1, LOGICAL_LANE_0,
					    LOGICAL_LANE_2, LOGICAL_LANE_3);
		break;

	case PIN_TYPE_C:
	case PIN_TYPE_D:
	case PIN_TYPE_E:
	case PIN_TYPE_F:
		if (hw_config->orient_type == PLUG_NORMAL) {
			cal_log_info(0, "%s: Type_E & Plug_Normal\n", __func__);
			dp_reg_set_lane_map(LOGICAL_LANE_3, LOGICAL_LANE_2,
					    LOGICAL_LANE_0, LOGICAL_LANE_1);
		} else {
			cal_log_info(0, "%s: Type_E & Plug_Flipped\n",
				     __func__);
			dp_reg_set_lane_map(LOGICAL_LANE_0, LOGICAL_LANE_1,
					    LOGICAL_LANE_3, LOGICAL_LANE_2);
		}
		break;

	default:
		break;
	}
}

// PCS Test Pattern Control
static void dp_reg_set_quality_pattern(u32 en)
{
	dp_write_mask(SST1, PCS_TEST_PATTERN_CONTROL,
		      LINK_QUALITY_PATTERN_SET(en), LINK_QUALITY_PATTERN_MASK);
}

/* HDCP Control Registers */

/* SST1 Control Registers */
// SST1 Main
static void dp_reg_set_audio_mode(enum audio_mode mode)
{
	dp_write_mask(SST1, SST1_MAIN_CONTROL, AUDIO_MODE_SET(mode),
		      AUDIO_MODE_MASK);
}

static void dp_reg_set_video_mode(enum video_mode mode)
{
	dp_write_mask(SST1, SST1_MAIN_CONTROL, VIDEO_MODE_SET(mode),
		      VIDEO_MODE_MASK);
}

static void dp_reg_set_enhanced_mode(u32 en)
{
	dp_write_mask(SST1, SST1_MAIN_CONTROL, ENHANCED_MODE_SET(en),
		      ENHANCED_MODE_MASK);
}

static void dp_reg_set_clear_audio_fifo(void)
{
	dp_write_mask(SST1, SST1_MAIN_FIFO_CONTROL, CLEAR_AUDIO_1_FIFO_SET(1),
		      CLEAR_AUDIO_1_FIFO_MASK);
}

// SST1 Interrupts
static u32 dp_reg_get_video_interrupt(void)
{
	return dp_read(SST1, SST1_INTERRUPT_STATUS_SET0);
}

static void dp_reg_set_video_interrupt_clear(u32 val)
{
	dp_write(SST1, SST1_INTERRUPT_STATUS_SET0, val);
}

static void dp_reg_set_video_interrupt_mask(u32 en, u32 intr_mask)
{
	if (en)
		dp_write_mask(SST1, SST1_INTERRUPT_MASK_SET0, intr_mask,
			      intr_mask);
	else
		dp_write_mask(SST1, SST1_INTERRUPT_MASK_SET0, 0, intr_mask);
}

static void dp_hw_set_video_interrupt(u32 en)
{
	// Mask all video interrupts
	dp_reg_set_video_interrupt_mask(0, ~0);

	if (en)
		dp_reg_set_video_interrupt_mask(
			en, VSC_SDP_TX_INCOMPLETE_MASK |
				    MAPI_FIFO_UNDER_FLOW_MASK | VSYNC_DET_MASK);
}

// SST1 Video M/N Value
static void dp_reg_set_mn_value(u32 mvid, u32 nvid)
{
	dp_write(SST1, SST1_MVID_MASTER_MODE, mvid);
	dp_write(SST1, SST1_NVID_MASTER_MODE, nvid);
}

static void dp_reg_set_mn_config(u32 strm_clk, u32 ls_clk)
{
	dp_write_mask(SST1, SST1_MVID_SFR_CONFIGURE, strm_clk,
		      MNVID_SFR_CONFIG_MASK);
	dp_write_mask(SST1, SST1_NVID_SFR_CONFIGURE, ls_clk,
		      MNVID_SFR_CONFIG_MASK);
}

// SST1 Audio M/N Value
static void dp_reg_set_audio_mn_value(enum audio_sampling_frequency fs,
				      enum dp_link_rate_type rate)
{
	dp_write(SST1, SST1_MAUD_MASTER_MODE, m_aud_master[fs]);
	dp_write(SST1, SST1_NAUD_MASTER_MODE, n_aud_master[rate]);
}

static void dp_reg_set_audio_mn_config(enum audio_mode mode,
				       enum dp_link_rate_type rate,
				       enum audio_sampling_frequency fs)
{
	u32 m_value, n_value;

	if (mode == AUDIO_MODE_ASYNC) {
		m_value = audio_async_m_n[0][rate][fs];
		n_value = audio_async_m_n[1][rate][fs];
	} else {
		m_value = audio_sync_m_n[0][rate][fs];
		n_value = audio_sync_m_n[1][rate][fs];
	}

	dp_write_mask(SST1, SST1_MAUD_SFR_CONFIGURE, m_value,
		      MNAUD_SFR_CONFIG_MASK);
	dp_write_mask(SST1, SST1_NAUD_SFR_CONFIGURE, n_value,
		      MNAUD_SFR_CONFIG_MASK);
}

static u32 dp_get_ls_clk(struct dp_hw_config *hw_config)
{
	/* DP LS(Link Symbol) clocks are 162 / 270 / 540 / 810 MHz in DP Spec */
	if (hw_config->link_rate == LINK_RATE_HBR3)
		return 810000000; // 810 MHz
	else if (hw_config->link_rate == LINK_RATE_HBR2)
		return 540000000; // 540 MHz
	else if (hw_config->link_rate == LINK_RATE_HBR)
		return 270000000; // 270 MHz
	else /* LINK_RATE_RBR or others */
		return 162000000; // 162 MHz
}

static void dp_reg_set_video_clock(struct dp_hw_config *hw_config)
{
	u32 strm_clk = hw_config->vtiming.clock; // KHz Unit
	u32 ls_clk = dp_get_ls_clk(hw_config) / 1000; // KHz Unit

	u32 mvid_master = strm_clk / 2;
	u32 nvid_master = ls_clk;

	dp_reg_set_mn_value(mvid_master, nvid_master);
	dp_reg_set_mn_config(strm_clk, ls_clk);
}

// SST1 Active Symbol SW Control
static void dp_reg_set_active_symbol_config_fec_off(u32 integer, u32 fraction,
						    u32 threshold)
{
	dp_write_mask(SST1, SST1_ACTIVE_SYMBOL_INTEGER_FEC_OFF,
		      ACTIVE_SYMBOL_INTEGER_SET(integer),
		      ACTIVE_SYMBOL_INTEGER_MASK);
	dp_write_mask(SST1, SST1_ACTIVE_SYMBOL_FRACTION_FEC_OFF,
		      ACTIVE_SYMBOL_FRACTION_SET(fraction),
		      ACTIVE_SYMBOL_FRACTION_MASK);
	dp_write_mask(SST1, SST1_ACTIVE_SYMBOL_THRESHOLD_FEC_OFF,
		      ACTIVE_SYMBOL_THRESHOLD_SET(threshold),
		      ACTIVE_SYMBOL_THRESHOLD_MASK);
}

static void dp_reg_set_active_symbol_config_fec_on(u32 integer, u32 fraction,
						   u32 threshold)
{
	dp_write_mask(SST1, SST1_ACTIVE_SYMBOL_INTEGER_FEC_ON,
		      ACTIVE_SYMBOL_INTEGER_SET(integer),
		      ACTIVE_SYMBOL_INTEGER_MASK);
	dp_write_mask(SST1, SST1_ACTIVE_SYMBOL_FRACTION_FEC_ON,
		      ACTIVE_SYMBOL_FRACTION_SET(fraction),
		      ACTIVE_SYMBOL_FRACTION_MASK);
	dp_write_mask(SST1, SST1_ACTIVE_SYMBOL_THRESHOLD_FEC_ON,
		      ACTIVE_SYMBOL_THRESHOLD_SET(threshold),
		      ACTIVE_SYMBOL_THRESHOLD_MASK);
}

static void dp_reg_set_active_symbol_threshold_sel_fec_off(u32 en)
{
	dp_write_mask(SST1, SST1_ACTIVE_SYMBOL_THRESHOLD_SEL_FEC_OFF,
		      ACTIVE_SYMBOL_THRESHOLD_SEL_SET(en),
		      ACTIVE_SYMBOL_THRESHOLD_SEL_MASK);
}

static void dp_reg_set_active_symbol_threshold_sel_fec_on(u32 en)
{
	dp_write_mask(SST1, SST1_ACTIVE_SYMBOL_THRESHOLD_SEL_FEC_ON,
		      ACTIVE_SYMBOL_THRESHOLD_SEL_SET(en),
		      ACTIVE_SYMBOL_THRESHOLD_SEL_MASK);
}

#define MULTIPLIER                                                             \
	10000000000 // It needs to separate integer part and fraction part
static void dp_reg_set_active_symbol(struct dp_hw_config *hw_config)
{
	u32 strm_clk = 0;
	u32 ls_clk = 0;
	u32 bpp = 0; /* BPP (Bit Per Pixel) */
	u32 num_lanes = 0;

	u64 TU_size = 0; /* TU (Transfer Unit) */
	u32 integer_part = 0;
	u32 fraction_part = 0;
	u32 threshold = 0;

	strm_clk = hw_config->vtiming.clock / 1000; // MHz Unit
	ls_clk = dp_get_ls_clk(hw_config) / 1000000; // MHz Unit
	bpp = 18 + (u32)(hw_config->bpc) * 6; // 18 is bpp for BPC_6
	num_lanes = hw_config->num_lanes;

	cal_log_debug(
		0,
		"bpp:%u, stream_clk: %u MHz, ls_clk: %u MHz, num_lanes: %u\n",
		bpp, strm_clk, ls_clk, num_lanes);

	TU_size =
		((strm_clk * bpp * 32) * MULTIPLIER) / (num_lanes * ls_clk * 8);

	if (hw_config->use_fec) {
		/* FEC (Forward Error Correction) */
		TU_size = (TU_size * 1000) / 976;
		cal_log_debug(0, "TU_size: %llu\n", TU_size);

		integer_part = (u32)(TU_size / MULTIPLIER);
		fraction_part =
			(u32)((TU_size - (integer_part * MULTIPLIER)) / 10);

		if (integer_part <= 2)
			threshold = 7;
		else if (integer_part > 2 && integer_part <= 5)
			threshold = 8;
		else if (integer_part > 5)
			threshold = 9;

		cal_log_debug(0, "FEC_On(int: %d, frac: %d, thr: %d)\n",
			      integer_part, fraction_part, threshold);

		dp_reg_set_active_symbol_config_fec_on(
			integer_part, fraction_part, threshold);
		dp_reg_set_active_symbol_threshold_sel_fec_on(1);
	} else {
		/* No FEC */
		cal_log_debug(0, "TU_size: %llu\n", TU_size);

		integer_part = (u32)(TU_size / MULTIPLIER);
		fraction_part =
			(u32)((TU_size - (integer_part * MULTIPLIER)) / 10);

		if (integer_part <= 2)
			threshold = 7;
		else if (integer_part > 2 && integer_part <= 5)
			threshold = 8;
		else if (integer_part > 5)
			threshold = 9;

		cal_log_debug(0, "FEC_Off(int: %d, frac: %d, thr: %d)\n",
			      integer_part, fraction_part, threshold);

		dp_reg_set_active_symbol_config_fec_off(
			integer_part, fraction_part, threshold);
		dp_reg_set_active_symbol_threshold_sel_fec_off(1);
	}
}

// SST1 Video Control
static void dp_reg_set_strm_valid_force(u32 en)
{
	dp_write_mask(SST1, SST1_VIDEO_CONTROL, STRM_VALID_CTRL_SET(en),
		      STRM_VALID_CTRL_MASK);
	dp_write_mask(SST1, SST1_VIDEO_CONTROL, STRM_VALID_FORCE_SET(en),
		      STRM_VALID_FORCE_MASK);
}

static void dp_reg_set_dynamic_range(enum dp_dynamic_range_type type)
{
	dp_write_mask(SST1, SST1_VIDEO_CONTROL, DYNAMIC_RANGE_MODE_SET(type),
		      DYNAMIC_RANGE_MODE_MASK);
}

static void dp_reg_set_bpc(enum bit_depth bpc)
{
	dp_write_mask(SST1, SST1_VIDEO_CONTROL, BPC_SET(bpc), BPC_MASK);
}

static void dp_reg_set_color_format(enum color_format format)
{
	dp_write_mask(SST1, SST1_VIDEO_CONTROL, COLOR_FORMAT_SET(format),
		      COLOR_FORMAT_MASK);
}

static void dp_reg_set_video_polarity(struct video_timing *vtiming)
{
	dp_write_mask(SST1, SST1_VIDEO_CONTROL,
		      VSYNC_POLARITY_SET(vtiming->vsync_polarity),
		      VSYNC_POLARITY_MASK);
	dp_write_mask(SST1, SST1_VIDEO_CONTROL,
		      HSYNC_POLARITY_SET(vtiming->hsync_polarity),
		      HSYNC_POLARITY_MASK);
}

static void dp_reg_set_video_en(u32 en)
{
	dp_write_mask(SST1, SST1_VIDEO_ENABLE, VIDEO_EN_SET(en), VIDEO_EN_MASK);
}

static void dp_reg_set_video_master_timing_gen(u32 en)
{
	dp_write_mask(SST1, SST1_VIDEO_MASTER_TIMING_GEN,
		      VIDEO_MASTER_TIME_GEN_SET(en),
		      VIDEO_MASTER_TIME_GEN_MASK);
}

// SST1 Video Timing
static void dp_reg_set_video_timing(struct video_timing *vtiming)
{
	dp_write(SST1, SST1_VIDEO_HORIZONTAL_TOTAL_PIXELS, vtiming->htotal);
	dp_write(SST1, SST1_VIDEO_VERTICAL_TOTAL_PIXELS, vtiming->vtotal);
	dp_write(SST1, SST1_VIDEO_HORIZONTAL_FRONT_PORCH, vtiming->hfp);
	dp_write(SST1, SST1_VIDEO_HORIZONTAL_BACK_PORCH, vtiming->hbp);
	dp_write(SST1, SST1_VIDEO_HORIZONTAL_ACTIVE, vtiming->hactive);
	dp_write(SST1, SST1_VIDEO_VERTICAL_FRONT_PORCH, vtiming->vfp);
	dp_write(SST1, SST1_VIDEO_VERTICAL_BACK_PORCH, vtiming->vbp);
	dp_write(SST1, SST1_VIDEO_VERTICAL_ACTIVE, vtiming->vactive);
}

// SST1 Video BIST Control
static void dp_reg_set_cts_bist_en(u32 en)
{
	dp_write_mask(SST1, SST1_VIDEO_BIST_CONTROL, CTS_BIST_EN_SET(en),
		      CTS_BIST_EN_MASK);
}

static void dp_reg_set_cts_bist_type(enum bist_pattern_type type)
{
	dp_write_mask(SST1, SST1_VIDEO_BIST_CONTROL, CTS_BIST_TYPE_SET(type),
		      CTS_BIST_TYPE_MASK);
}

static void dp_reg_set_video_bist_en(u32 en)
{
	dp_write_mask(SST1, SST1_VIDEO_BIST_CONTROL, BIST_EN_SET(en),
		      BIST_EN_MASK);
}

static void dp_reg_set_video_bist_width(u32 width)
{
	dp_write_mask(SST1, SST1_VIDEO_BIST_CONTROL, BIST_WIDTH_SET(width),
		      BIST_WIDTH_MASK);
}

static void dp_reg_set_video_bist_type(enum bist_pattern_type type)
{
	dp_write_mask(SST1, SST1_VIDEO_BIST_CONTROL, BIST_TYPE_SET(type),
		      BIST_TYPE_MASK);
}

// SST1 Audio Control
static void dp_reg_set_audio_dma_req_gen(u32 en)
{
	dp_write_mask(SST1, SST1_AUDIO_CONTROL, DMA_REQ_GEN_EN_SET(en),
		      DMA_REQ_GEN_EN_MASK);
}

static void
dp_reg_set_audio_dma_burst_size(enum audio_dma_word_length word_length)
{
	dp_write_mask(SST1, SST1_AUDIO_CONTROL, DMA_BURST_SEL_SET(word_length),
		      DMA_BURST_SEL_MASK);
}

static void dp_reg_set_audio_pack_mode(enum audio_16bit_dma_mode dma_mode)
{
	dp_write_mask(SST1, SST1_AUDIO_CONTROL,
		      AUDIO_BIT_MAPPING_TYPE_SET(dma_mode),
		      AUDIO_BIT_MAPPING_TYPE_MASK);
}

static void dp_reg_set_audio_pcm_size(enum audio_bit_per_channel audio_bit_size)
{
	dp_write_mask(SST1, SST1_AUDIO_CONTROL, PCM_SIZE_SET(audio_bit_size),
		      PCM_SIZE_MASK);
}

static void dp_reg_set_audio_ch_status_same(u32 en)
{
	dp_write_mask(SST1, SST1_AUDIO_CONTROL, AUDIO_CH_STATUS_SAME_SET(en),
		      AUDIO_CH_STATUS_SAME_MASK);
}

static bool dp_reg_is_audio_enabled(void)
{
	return AUDIO_EN_GET(dp_read(SST1, SST1_AUDIO_ENABLE)) ? true : false;
}

static void dp_reg_set_audio_en(u32 en)
{
	dp_write_mask(SST1, SST1_AUDIO_ENABLE, AUDIO_EN_SET(en), AUDIO_EN_MASK);
}

static void dp_reg_set_audio_master_timing_gen(u32 en)
{
	dp_write_mask(SST1, SST1_AUDIO_MASTER_TIMING_GEN,
		      AUDIO_MASTER_TIME_GEN_SET(en),
		      AUDIO_MASTER_TIME_GEN_MASK);
}

// SST1 Audio BIST Control
static void dp_reg_set_audio_bist_en(u32 en)
{
	dp_write_mask(SST1, SST1_AUDIO_BIST_CONTROL, SIN_AMPL_SET(0x0F),
		      SIN_AMPL_MASK);
	dp_write_mask(SST1, SST1_AUDIO_BIST_CONTROL, AUD_BIST_EN_SET(en),
		      AUD_BIST_EN_MASK);
}

static void dp_map_audio_word_size(enum audio_bit_per_channel audio_bpc,
				   u32 *word_max, u32 *word_length)
{
	switch (audio_bpc) {
	case AUDIO_24_BIT:
		*word_max = 1;
		*word_length = 0x05;
		break;

	case AUDIO_16_BIT:
		*word_max = 0;
		*word_length = 0x01;
		break;

	case AUDIO_20_BIT:
		*word_max = 0;
		*word_length = 0x05;
		break;

	default:
		*word_max = 0;
		*word_length = 0x00;
		break;
	}
}

static void
dp_reg_set_audio_bist_ch_status(enum audio_clock_accuracy clock_accuracy,
				enum audio_sampling_frequency fs_freq,
				u32 num_audio_ch,
				enum audio_bit_per_channel audio_bpc)
{
	u32 word_max = 0;
	u32 word_length = 0;

	dp_write_mask(SST1, SST1_AUDIO_BIST_CHANNEL_STATUS_SET0,
		      CLK_ACCUR_SET(clock_accuracy), CLK_ACCUR_MASK);
	dp_write_mask(SST1, SST1_AUDIO_BIST_CHANNEL_STATUS_SET0,
		      FS_FREQ_SET(fs_freq), FS_FREQ_MASK);
	dp_write_mask(SST1, SST1_AUDIO_BIST_CHANNEL_STATUS_SET0,
		      CH_NUM_SET(num_audio_ch), CH_NUM_MASK);
	dp_write_mask(SST1, SST1_AUDIO_BIST_CHANNEL_STATUS_SET0,
		      SOURCE_NUM_SET(num_audio_ch), SOURCE_NUM_MASK);

	dp_map_audio_word_size(audio_bpc, &word_max, &word_length);
	dp_write_mask(SST1, SST1_AUDIO_BIST_CHANNEL_STATUS_SET1,
		      WORD_LENGTH_SET(word_length), WORD_LENGTH_MASK);
	dp_write_mask(SST1, SST1_AUDIO_BIST_CHANNEL_STATUS_SET1,
		      WORD_MAX_SET(word_max), WORD_MAX_MASK);
}

// SST1 Audio Channel Control
static void dp_reg_set_audio_ch(u32 audio_ch_cnt)
{
	dp_write_mask(SST1, SST1_AUDIO_BUFFER_CONTROL,
		      MASTER_AUDIO_CHANNEL_COUNT_SET(audio_ch_cnt - 1),
		      MASTER_AUDIO_CHANNEL_COUNT_MASK);
}

static void dp_reg_set_audio_ch_status(enum audio_clock_accuracy clock_accuracy,
				       enum audio_sampling_frequency fs_freq,
				       u32 num_audio_ch,
				       enum audio_bit_per_channel audio_bpc)
{
	u32 word_max = 0;
	u32 word_length = 0;

	dp_write_mask(SST1, SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_0,
		      CLK_ACCUR_SET(clock_accuracy), CLK_ACCUR_MASK);
	dp_write_mask(SST1, SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_0,
		      FS_FREQ_SET(fs_freq), FS_FREQ_MASK);
	dp_write_mask(SST1, SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_0,
		      CH_NUM_SET(num_audio_ch), CH_NUM_MASK);
	dp_write_mask(SST1, SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_0,
		      SOURCE_NUM_SET(num_audio_ch), SOURCE_NUM_MASK);

	dp_map_audio_word_size(audio_bpc, &word_max, &word_length);
	dp_write_mask(SST1, SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_1,
		      WORD_LENGTH_SET(word_length), WORD_LENGTH_MASK);
	dp_write_mask(SST1, SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_1,
		      WORD_MAX_SET(word_max), WORD_MAX_MASK);

	/* Other channels should be same configurations by dp_reg_set_audio_ch_status_same() */
}

// SST1 InfoFrame Control
static void dp_reg_set_audio_infoframe_update(u32 en)
{
	dp_write_mask(SST1, SST1_INFOFRAME_UPDATE_CONTROL,
		      AUDIO_INFO_UPDATE_SET(en), AUDIO_INFO_UPDATE_MASK);
}

static void dp_reg_set_avi_infoframe_update(u32 en)
{
	dp_write_mask(SST1, SST1_INFOFRAME_UPDATE_CONTROL,
		      AVI_INFO_UPDATE_SET(en), AVI_INFO_UPDATE_MASK);
}

static void dp_reg_set_spd_infoframe_update(u32 en)
{
	dp_write_mask(SST1, SST1_INFOFRAME_UPDATE_CONTROL,
		      SPD_INFO_UPDATE_SET(en), SPD_INFO_UPDATE_MASK);
}

static void dp_reg_set_audio_infoframe_send(u32 en)
{
	dp_write_mask(SST1, SST1_INFOFRAME_SEND_CONTROL,
		      AUDIO_INFO_SEND_SET(en), AUDIO_INFO_SEND_MASK);
}

static void dp_reg_set_avi_infoframe_send(u32 en)
{
	dp_write_mask(SST1, SST1_INFOFRAME_SEND_CONTROL, AVI_INFO_SEND_SET(en),
		      AVI_INFO_SEND_MASK);
}

static void dp_reg_set_spd_infoframe_send(u32 en)
{
	dp_write_mask(SST1, SST1_INFOFRAME_SEND_CONTROL, SPD_INFO_SEND_SET(en),
		      SPD_INFO_SEND_MASK);
}

static void dp_reg_set_spd_infoframe_packet_type(u8 type_code)
{
	dp_write_mask(SST1, SST1_INFOFRAME_SPD_PACKET_TYPE,
		      SPD_TYPE_SET(type_code), SPD_TYPE_MASK);
}

static void dp_reg_set_infoframe_data(u32 offset, u8 *packet_data,
				      u32 max_byte_size)
{
	u32 data_word;
	u32 byte_index = 0, word_index, shift_index;

	for (word_index = 0; word_index < (max_byte_size - 1);
	     word_index += 4) {
		data_word = 0;

		for (shift_index = 0; shift_index < 32; shift_index += 8)
			data_word |= packet_data[byte_index++] << shift_index;

		dp_write(SST1, offset + word_index, data_word);
	}

	data_word = 0;
	for (shift_index = 0; (byte_index < max_byte_size && shift_index < 32);
	     shift_index += 8)
		data_word |= packet_data[byte_index++] << shift_index;
	dp_write(SST1, offset + word_index, data_word);
}

static void dp_reg_set_avi_infoframe_data(u8 *packet_data)
{
	dp_reg_set_infoframe_data(SST1_INFOFRAME_AVI_PACKET_DATA_SET0,
				  packet_data,
				  MAX_AVI_INFOFRAME_DATA_BYTE_SIZE);
}

static void dp_reg_set_audio_infoframe_data(u8 *packet_data)
{
	dp_reg_set_infoframe_data(SST1_INFOFRAME_AUDIO_PACKET_DATA_SET0,
				  packet_data,
				  MAX_AUDIO_INFOFRAME_DATA_BYTE_SIZE);
}

static void dp_reg_set_spd_infoframe_data(u8 *packet_data)
{
	dp_reg_set_infoframe_data(SST1_INFOFRAME_SPD_PACKET_DATA_SET0,
				  packet_data,
				  MAX_SPD_INFOFRAME_DATA_BYTE_SIZE);
}

//---------------------------------------
/* DPCD (DisplayPort Configuration Data) Read/Write Interfaces through AUX channel */
static int dp_reg_set_aux_ch_operation_enable(void)
{
	if (dp_reg_do_aux_transaction())
		return -ETIME;

	if (dp_reg_check_aux_monitors())
		return -EIO;

	return 0;
}

static int dp_write_dpcd(u32 address, u32 length, u8 *data)
{
	int ret;
	int retry_cnt = AUX_RETRY_COUNT;

	while (retry_cnt > 0) {
		dp_reg_aux_ch_buf_clr();
		dp_reg_aux_defer_ctrl(1);
		dp_reg_set_aux_reply_timeout();
		dp_reg_set_aux_ch_command(DPCD_WRITE);
		dp_reg_set_aux_ch_address(address);
		dp_reg_set_aux_ch_length(length);
		dp_reg_aux_ch_send_buf(data, length);
		ret = dp_reg_set_aux_ch_operation_enable();
		if (ret == 0)
			break;

		retry_cnt--;
	}

	return ret;
}

static int dp_read_dpcd(u32 address, u32 length, u8 *data)
{
	int ret;
	int retry_cnt = AUX_RETRY_COUNT;

	while (retry_cnt > 0) {
		dp_reg_set_aux_ch_command(DPCD_READ);
		dp_reg_set_aux_ch_address(address);
		dp_reg_set_aux_ch_length(length);
		dp_reg_aux_ch_buf_clr();
		dp_reg_aux_defer_ctrl(1);
		dp_reg_set_aux_reply_timeout();
		ret = dp_reg_set_aux_ch_operation_enable();
		if (ret == 0)
			break;

		retry_cnt--;
	}

	if (ret == 0)
		dp_reg_aux_ch_received_buf(data, length);

	return ret;
}

int dp_hw_write_dpcd_burst(u32 address, u32 length, u8 *data)
{
	int ret = 0;
	u32 i, buf_length, length_calculation;

	length_calculation = length;
	for (i = 0; i < length; i += AUX_DATA_BUF_COUNT) {
		if (length_calculation >= AUX_DATA_BUF_COUNT) {
			buf_length = AUX_DATA_BUF_COUNT;
			length_calculation -= AUX_DATA_BUF_COUNT;
		} else {
			buf_length = length % AUX_DATA_BUF_COUNT;
			length_calculation = 0;
		}

		ret = dp_write_dpcd(address + i, buf_length, data + i);
		if (ret != 0) {
			cal_log_err(0, "dp_reg_dpcd_write_burst fail\n");
			break;
		}
	}

	return ret;
}

int dp_hw_read_dpcd_burst(u32 address, u32 length, u8 *data)
{
	int ret = 0;
	u32 i, buf_length, length_calculation;

	length_calculation = length;

	for (i = 0; i < length; i += AUX_DATA_BUF_COUNT) {
		if (length_calculation >= AUX_DATA_BUF_COUNT) {
			buf_length = AUX_DATA_BUF_COUNT;
			length_calculation -= AUX_DATA_BUF_COUNT;
		} else {
			buf_length = length % AUX_DATA_BUF_COUNT;
			length_calculation = 0;
		}

		ret = dp_read_dpcd(address + i, buf_length, data + i);
		if (ret != 0) {
			cal_log_err(0, "dp_reg_dpcd_read_burst fail\n");
			break;
		}
	}

	return ret;
}

int dp_hw_read_edid(u8 block_cnt, u32 length, u8 *data)
{
	u32 i, buf_length, length_calculation;
	int ret;
	int retry_cnt = AUX_RETRY_COUNT;
	u8 offset = (block_cnt & 1) * EDID_BLOCK_SIZE;

	while (retry_cnt > 0) {
		dp_reg_aux_ch_buf_clr();
		dp_reg_aux_defer_ctrl(1);
		dp_reg_set_aux_reply_timeout();
		dp_reg_set_aux_ch_address_only_command(false);

		/* for 3,4 block */
		if (block_cnt > 1) {
			u8 segment = 1;

			cal_log_warn(0, "read block%d\n", block_cnt);
			dp_reg_set_aux_ch_command(I2C_WRITE);
			dp_reg_set_aux_ch_address(DDC_SEGMENT_ADDR);
			dp_reg_set_aux_ch_length(1);
			dp_reg_aux_ch_send_buf(&segment, 1);
			ret = dp_reg_set_aux_ch_operation_enable();
			if (ret)
				cal_log_err(0, "sending segment failed\n");
		}

		dp_reg_set_aux_ch_command(I2C_WRITE);
		dp_reg_set_aux_ch_address(EDID_ADDRESS);
		dp_reg_set_aux_ch_length(1);
		dp_reg_aux_ch_send_buf(&offset, 1);
		ret = dp_reg_set_aux_ch_operation_enable();

		cal_log_debug(0, "EDID address command in EDID read\n");

		if (ret == 0) {
			dp_reg_set_aux_ch_command(I2C_READ);
			length_calculation = length;

			for (i = 0; i < length; i += AUX_DATA_BUF_COUNT) {
				if (length_calculation >= AUX_DATA_BUF_COUNT) {
					buf_length = AUX_DATA_BUF_COUNT;
					length_calculation -=
						AUX_DATA_BUF_COUNT;
				} else {
					buf_length =
						length % AUX_DATA_BUF_COUNT;
					length_calculation = 0;
				}

				dp_reg_set_aux_ch_length(buf_length);
				dp_reg_aux_ch_buf_clr();
				ret = dp_reg_set_aux_ch_operation_enable();

				if (ret == 0) {
					dp_reg_aux_ch_received_buf(
						data + ((i /
							 AUX_DATA_BUF_COUNT) *
							AUX_DATA_BUF_COUNT),
						buf_length);
					cal_log_debug(
						0,
						"AUX buffer read count = %d in EDID read\n",
						i);
				} else {
					cal_log_debug(
						0,
						"AUX buffer read fail in EDID read\n");
					break;
				}
			}
		}

		if (ret == 0) {
			dp_reg_set_aux_ch_address_only_command(true);
			ret = dp_reg_set_aux_ch_operation_enable();
			dp_reg_set_aux_ch_address_only_command(false);

			cal_log_debug(
				0, "2nd address only request in EDID read\n");
		}

		if (ret == 0)
			break;

		retry_cnt--;
	}

	return ret;
}

/* DP Hardware Control Interfaces */
int dp_hw_init(struct dp_hw_config *hw_config)
{
	cal_log_info(0, "DP Link Version = 0x%X\n", dp_reg_get_version());

	/* Apply Soft Reset */
	dp_reg_set_swreset();
	cal_log_debug(0, "reset DP Link\n");

	/* Set system clock to OSC */
	dp_reg_set_txclk_sel_mode_by_txclksel();
	dp_reg_set_osc_clk_div(OSC_CLK);
	dp_reg_set_txclk_osc();
	cal_log_debug(0, "set system clk to OSC with %d MHz: Mux(%u)\n",
		      OSC_CLK, dp_reg_get_gfmux_status());

	/*
	 * USBDP PHY Initialization
	 */
	dwc3_exynos_phy_enable(1, 1);
	atomic_inc(&hw_config->usbdp_phy_en_cnt);

	dpphy_reg_init(hw_config);
	cal_log_debug(0, "init USBDP Combo PHY\n");

	/* Wait for PHY PLL Lock */
	dp_reg_wait_phy_pll_lock();
	cal_log_debug(0, "locked PHY PLL\n");

	/* Set system clock to TXCLK */
	dp_reg_set_txclk_txclk();
	cal_log_debug(0, "set system clk to TX: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

	/*
	 * DP Link Initialization
	 */
	/* Set Lane Map Configuration */
	dp_reg_set_lane_count(hw_config->num_lanes);
	dp_reg_set_aux_pn(hw_config->orient_type);
	dp_hw_set_lane_map_config(hw_config);
	cal_log_debug(0, "set lane count & map\n");

	/* Set System Common Functions Enable */
	dp_hw_set_initial_common_funcs();
	cal_log_debug(0, "set common function\n");

	/* Set System SW Functions Enable */
	dp_reg_set_sw_func_en(1);
	cal_log_debug(0, "set sw function\n");

	/* Set Interrupts */
	dp_hw_set_common_interrupt(1);
	cal_log_debug(0, "set interrupts\n");

	return 0;
}

int dp_hw_reinit(struct dp_hw_config *hw_config)
{
	/* Set system clock to OSC */
	dp_reg_set_txclk_osc();
	cal_log_debug(0, "set system clk to OSC: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

	/* USBDP PHY Re-initialization */
	dpphy_reg_init(hw_config);
	cal_log_debug(0, "reconfig USBDP Combo PHY\n");

	/* Wait for PHY PLL Lock */
	dp_reg_wait_phy_pll_lock();
	cal_log_debug(0, "locked PHY PLL\n");

	/* Set system clock to TXCLK */
	dp_reg_set_txclk_txclk();
	cal_log_debug(0, "set system clk to TX: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

	/* Set Lane Map Configuration */
	dp_reg_set_lane_count(hw_config->num_lanes);
	dp_reg_set_aux_pn(hw_config->orient_type);
	dp_hw_set_lane_map_config(hw_config);
	cal_log_debug(0, "set lane count & map\n");

	/* Set System Common Functions Enable */
	dp_reg_set_pcs_func_en(1);
	cal_log_debug(0, "set common function\n");

	/* Set Single-Stream Transport Functions Enable */
	dp_reg_set_enhanced_mode(hw_config->enhanced_mode ? 1 : 0);
	dp_reg_set_sst1_video_func_en(1);
	cal_log_debug(0, "set sst function\n");

	return 0;
}

void dp_hw_deinit(struct dp_hw_config *hw_config)
{
	/* DP Link De-initialization */
	dp_hw_set_common_interrupt(0);
	dp_reg_set_sw_func_en(0);
	dp_reg_set_txclk_osc();
	cal_log_debug(0, "set system clk to OSC: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

	/* USBDP PHY De-initialization */
	dwc3_exynos_phy_enable(1, 0);
	atomic_dec(&hw_config->usbdp_phy_en_cnt);
}

void dp_hw_start(void)
{
	dp_reg_set_sst1_longhop_power(1);
	if (dp_reg_wait_sst1_longhop_power_on())
		cal_log_err(0, "SST LongHop Power is not ON\n");

	dp_hw_set_video_interrupt(1);
	dp_reg_set_video_en(1);
}

void dp_hw_stop(void)
{
	dp_reg_set_sst1_longhop_power(0);
	dp_hw_set_video_interrupt(0);
	dp_reg_set_video_en(0);
}

void dp_hw_set_video_config(struct dp_hw_config *hw_config)
{
	dp_reg_set_dynamic_range(hw_config->range);
	dp_reg_set_bpc(hw_config->bpc);
	dp_reg_set_color_format(COLOR_RGB);
	dp_reg_set_video_polarity(&hw_config->vtiming);
	dp_reg_set_video_timing(&hw_config->vtiming);
	dp_reg_set_video_clock(hw_config);
	dp_reg_set_active_symbol(hw_config);
	dp_reg_set_video_master_timing_gen(1);
	dp_reg_set_video_mode(VIDEO_MODE_MASTER);
}

int dp_hw_set_bist_video_config(struct dp_hw_config *hw_config)
{
	if (!hw_config->bist_mode)
		return -EINVAL;

	dp_reg_set_strm_valid_force(1);

	if (hw_config->bist_type < CTS_COLOR_RAMP) {
		// Normal BIST Mode
		dp_hw_set_video_config(hw_config);

		if (hw_config->bist_type == COLOR_BAR)
			dp_reg_set_video_bist_width(0);
		dp_reg_set_video_bist_type(hw_config->bist_type);
		dp_reg_set_video_bist_en(1);
	} else {
		// CTS BIST Mode
		hw_config->range =
			(hw_config->bist_type == CTS_COLOR_SQUARE_CEA) ?
				CEA_RANGE :
				VESA_RANGE;
		dp_hw_set_video_config(hw_config);

		dp_reg_set_cts_bist_type(hw_config->bist_type - CTS_COLOR_RAMP);
		dp_reg_set_cts_bist_en(1);
	}

	return 0;
}

void dp_hw_get_intr_source(bool *common, bool *str, bool *pcs)
{
	dp_reg_get_interrupt_source(common, str, pcs);
}

u32 dp_hw_get_and_clear_common_intr(void)
{
	u32 val = 0;

	val = dp_reg_get_common_interrupt();
	dp_reg_set_common_interrupt_clear(val);

	return val;
}

u32 dp_hw_get_and_clear_video_intr(void)
{
	u32 val = 0;

	val = dp_reg_get_video_interrupt();
	dp_reg_set_video_interrupt_clear(val);

	return val;
}

void dp_hw_send_audio_infoframe(struct infoframe audio_infoframe)
{
	dp_reg_set_audio_infoframe_data(audio_infoframe.data);
	dp_reg_set_audio_infoframe_update(1);
	dp_reg_set_audio_infoframe_send(1);
}

void dp_hw_send_avi_infoframe(struct infoframe avi_infoframe)
{
	dp_reg_set_avi_infoframe_data(avi_infoframe.data);
	dp_reg_set_avi_infoframe_update(1);
	dp_reg_set_avi_infoframe_send(1);
}

void dp_hw_send_spd_infoframe(struct infoframe spd_infoframe)
{
	dp_reg_set_spd_infoframe_packet_type(spd_infoframe.type_code);
	dp_reg_set_spd_infoframe_data(spd_infoframe.data);
	dp_reg_set_spd_infoframe_update(1);
	dp_reg_set_spd_infoframe_send(1);
}

void dp_hw_set_training_pattern(dp_training_pattern pattern)
{
	dp_reg_set_quality_pattern(0);
	dp_reg_set_training_pattern(pattern);

	if (pattern == NORMAL_DATA || pattern == TRAINING_PATTERN_4)
		dp_reg_set_scramble_bypass(0);
	else
		dp_reg_set_scramble_bypass(1);
}

void dp_hw_set_voltage_and_pre_emphasis(struct dp_hw_config *hw_config,
					u8 *voltage, u8 *pre_emphasis)
{
	switch (hw_config->pin_type) {
	case PIN_TYPE_A:
		if (hw_config->orient_type == PLUG_NORMAL) {
			dpphy_reg_set_lane0_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane1_tx_level(voltage[2],
						     pre_emphasis[2]);
			dpphy_reg_set_lane2_tx_level(voltage[3],
						     pre_emphasis[3]);
			dpphy_reg_set_lane3_tx_level(voltage[0],
						     pre_emphasis[0]);
		} else {
			dpphy_reg_set_lane0_tx_level(voltage[0],
						     pre_emphasis[0]);
			dpphy_reg_set_lane1_tx_level(voltage[3],
						     pre_emphasis[3]);
			dpphy_reg_set_lane2_tx_level(voltage[2],
						     pre_emphasis[2]);
			dpphy_reg_set_lane3_tx_level(voltage[1],
						     pre_emphasis[1]);
		}
		break;

	case PIN_TYPE_B:
		if (hw_config->orient_type == PLUG_NORMAL) {
			dpphy_reg_set_lane2_tx_level(voltage[0],
						     pre_emphasis[0]);
			dpphy_reg_set_lane3_tx_level(voltage[1],
						     pre_emphasis[1]);
		} else {
			dpphy_reg_set_lane0_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane1_tx_level(voltage[0],
						     pre_emphasis[0]);
		}
		break;

	case PIN_TYPE_C:
	case PIN_TYPE_E:
		if (hw_config->orient_type == PLUG_NORMAL) {
			cal_log_info(0, "%s: Type_E & Plug_Normal\n", __func__);
			dpphy_reg_set_lane0_tx_level(voltage[2],
						     pre_emphasis[2]);
			dpphy_reg_set_lane1_tx_level(voltage[3],
						     pre_emphasis[3]);
			dpphy_reg_set_lane2_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane3_tx_level(voltage[0],
						     pre_emphasis[0]);
		} else {
			cal_log_info(0, "%s: Type_E & Plug_Flipped\n",
				     __func__);
			dpphy_reg_set_lane0_tx_level(voltage[0],
						     pre_emphasis[0]);
			dpphy_reg_set_lane1_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane2_tx_level(voltage[3],
						     pre_emphasis[3]);
			dpphy_reg_set_lane3_tx_level(voltage[2],
						     pre_emphasis[2]);
		}
		break;

	case PIN_TYPE_D:
	case PIN_TYPE_F:
		if (hw_config->orient_type == PLUG_NORMAL) {
			dpphy_reg_set_lane2_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane3_tx_level(voltage[0],
						     pre_emphasis[0]);
		} else {
			dpphy_reg_set_lane0_tx_level(voltage[0],
						     pre_emphasis[0]);
			dpphy_reg_set_lane1_tx_level(voltage[1],
						     pre_emphasis[1]);
		}
		break;

	default:
		break;
	}
}

// DP Audio Configuration Interfaces
void dp_hw_init_audio(void)
{
	dp_reg_set_sst1_audio_func_en(1);
	dp_reg_set_sst1_audio_fifo_func_en(1);

	cal_log_info(0, "%s: dp_audio_init_config\n", __func__);
}

void dp_hw_deinit_audio(void)
{
	dp_reg_set_sst1_audio_fifo_func_en(0);
	dp_reg_set_sst1_audio_func_en(0);
}

void dp_hw_start_audio(void)
{
	dp_reg_set_audio_en(1);
	dp_reg_set_audio_master_timing_gen(1);

	cal_log_info(0, "%s: dp_audio is started\n", __func__);
}

void dp_hw_stop_audio(void)
{
	if (dp_reg_is_audio_enabled()) {
		udelay(1000);
		dp_reg_set_audio_master_timing_gen(0);
		dp_reg_set_audio_en(0);
		dp_reg_set_clear_audio_fifo();
		cal_log_info(0, "%s: dp_audio is disabled\n", __func__);
	} else
		cal_log_info(0,
			     "%s: dp_audio is already disabled, AUDIO_EN = 0\n",
			     __func__);
}

void dp_hw_set_audio_config(struct dp_hw_config *hw_config)
{
	dp_reg_set_audio_mode(AUDIO_MODE_ASYNC);
	dp_reg_set_audio_mn_config(AUDIO_MODE_ASYNC, hw_config->link_rate,
				   hw_config->audio_fs);
	dp_reg_set_audio_mn_value(hw_config->audio_fs, hw_config->link_rate);

	dp_reg_set_audio_dma_burst_size(hw_config->audio_word_length);
	dp_reg_set_audio_pcm_size(hw_config->audio_bit);
	dp_reg_set_audio_pack_mode(hw_config->audio_packed_mode);

	dp_reg_set_audio_ch(hw_config->num_audio_ch);
	dp_reg_set_audio_ch_status(NOT_MATCH, hw_config->audio_fs,
				   hw_config->num_audio_ch,
				   hw_config->audio_bit);
	dp_reg_set_audio_ch_status_same(1);
}

int dp_hw_set_bist_audio_config(struct dp_hw_config *hw_config)
{
	if (!hw_config->bist_mode)
		return -EINVAL;

	dp_hw_set_audio_config(hw_config);

	dp_reg_set_audio_bist_ch_status(NOT_MATCH, hw_config->audio_fs,
					hw_config->num_audio_ch,
					hw_config->audio_bit);
	dp_reg_set_audio_bist_en(1);

	cal_log_info(0, "%s: dp_audio is configured for BIST mode\n", __func__);
	return 0;
}

void dp_hw_set_audio_dma(u32 en)
{
	dp_reg_set_audio_dma_req_gen(en);
}

int dp_crc_set_enabled(u32 id, u32 en)
{
	return -EINVAL;
}

int dp_crc_get(u32 id, u32 crc_data[3])
{
	return -EINVAL;
}

int dp_crc_reset(u32 id)
{
	return -EINVAL;
}
