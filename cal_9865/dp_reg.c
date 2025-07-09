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

#include <linux/soc/samsung/exynos-smc.h>

#include "regs-dp.h"
#include "regs-usbdpphy_ctrl.h"
#include "regs-usbdpphy_tca_ctrl.h"

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

#define dp_phy_tca_regs_desc(id)    (&regs_dp[REGS_PHY_TCA][id])
#define dp_phy_tca_read(id, offset) cal_read(dp_phy_tca_regs_desc(id), offset)
#define dp_phy_tca_write(id, offset, val)                                      \
	cal_write(dp_phy_tca_regs_desc(id), offset, val)
#define dp_phy_tca_read_mask(id, offset, mask)                                 \
	cal_read_mask(dp_phy_tca_regs_desc(id), offset, mask)
#define dp_phy_tca_write_mask(id, offset, val, mask)                           \
	cal_write_mask(dp_phy_tca_regs_desc(id), offset, val, mask)

/* DP memory map interface */
void dp_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		       enum dp_regs_type type, unsigned int id)
{
	cal_regs_desc_check(type, id, REGS_DP_TYPE_MAX, SST_MAX);
	cal_regs_desc_set(regs_dp, regs, start, name, type, id);
}

enum lane_usage {
	DP_USE_0_LANES = 0,
	DP_USE_2_LANES = 2,
	DP_USE_4_LANES = 4,
};

/* PHY Tuning Parameters */
enum phy_tune_info {
	EQ_MAIN = 0,
	EQ_PRE = 1,
	EQ_POST = 2,
	EQ_VSWING = 3,
	RBOOST = 4,
};

static u32 phy_eq_hbr0_1[4][4][5] = {
	/* {eq main, eq pre, eq post, eq vswing lvl, rboost en} */
	{
		/* Swing Level_0 */
		{ 21, 0, 0, 5, 0 }, /* Pre-emphasis Level_0 */
		{ 26, 0, 5, 5, 0 }, /* Pre-emphasis Level_1 */
		{ 31, 0, 10, 5, 0 }, /* Pre-emphasis Level_2 */
		{ 41, 0, 20, 5, 0 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_1 */
		{ 31, 0, 0, 5, 0 },
		{ 38, 0, 7, 5, 0 },
		{ 46, 0, 15, 5, 0 },
		{ -1, -1, -1, -1, -1 },
	},
	{
		/* Swing Level_2 */
		{ 43, 0, 0, 5, 0 },
		{ 52, 0, 9, 5, 0 },
		{ -1, -1, -1, -1, -1 },
		{ -1, -1, -1, -1, -1 },
	},
	{
		/* Swing Level_3 */
		{ 62, 0, 0, 5, 0 },
		{ -1, -1, -1, -1, -1 },
		{ -1, -1, -1, -1, -1 },
		{ -1, -1, -1, -1, -1 },
	},
};

static u32 phy_eq_hbr2_3[4][4][5] = {
	/* {eq main, eq pre, eq post, eq vswing lvl, rboost en} */
	{
		/* Swing Level_0 */
		{ 21, 0, 0, 5, 3 }, /* Pre-emphasis Level_0 */
		{ 25, 0, 4, 5, 3 }, /* Pre-emphasis Level_1 */
		{ 29, 0, 8, 5, 3 }, /* Pre-emphasis Level_2 */
		{ 35, 0, 14, 5, 3 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_1 */
		{ 31, 0, 0, 5, 3 },
		{ 37, 0, 6, 5, 3 },
		{ 42, 0, 11, 5, 3 },
		{ -1, -1, -1, -1, -1 },
	},
	{
		/* Swing Level_2 */
		{ 43, 0, 0, 5, 3 },
		{ 51, 0, 8, 5, 3 },
		{ -1, -1, -1, -1, -1 },
		{ -1, -1, -1, -1, -1 },
	},
	{
		/* Swing Level_3 */
		{ 62, 0, 0, 5, 3 },
		{ -1, -1, -1, -1, -1 },
		{ -1, -1, -1, -1, -1 },
		{ -1, -1, -1, -1, -1 },
	},
};

static u32 m_aud_master[7] = {
	32000, 44100, 48000, 88200, 96000, 176000, 192000
};

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

/* USBDP PHY TCA Registers */
static int dpphytca_reg_wait_mode_change(enum lane_usage lane)
{
	unsigned int cnt = 0;
	unsigned int val = 0;

	cal_log_debug(0, "Wait to change from USB to DP%d\n", lane);

	cnt = 2500;
	do {
		val = dp_phy_tca_read_mask(SST1, TCA_REG_TCA_INTR_STS,
					   TCA_REG_XA_ACK_EVT);
		udelay(1000);
		cnt--;
	} while (val == 0 && cnt > 0);

	if (!cnt) {
		cal_log_err(
			0,
			"TCA_TCPC 0x%x, TCA_INTR_STS 0x%x, TCA_CTRLSYNCMODE_DBG0 0x%x, "
			"TCA_REG_TCA_PSTATE 0x%x, TCA_REG_TCA_GEN_STATUS 0x%x\n",
			readl(regs_dp[REGS_PHY_TCA][SST1].regs + TCA_REG_TCA_TCPC),
			readl(regs_dp[REGS_PHY_TCA][SST1].regs + TCA_REG_TCA_INTR_STS),
			readl(regs_dp[REGS_PHY_TCA][SST1].regs + TCA_REG_TCA_CTRLSYNCMODE_DBG0),
			readl(regs_dp[REGS_PHY_TCA][SST1].regs + TCA_REG_TCA_PSTATE),
			readl(regs_dp[REGS_PHY_TCA][SST1].regs + TCA_REG_TCA_GEN_STATUS));

		cal_log_err(0, "Fail to change mode from USB to DP%d\n", lane);
		return -ETIME;
	}

	cal_log_debug(0, "Success to change mode from USB to DP%d\n", lane);
	return 0;
}

static void dpphytca_reg_set_tcpc(u32 mux_control, u32 orientation,
				  u32 low_pwr_en, u32 valid)
{
	u32 val = TCA_REG_TCPC_MUX_CONTROL_SET(mux_control) |
		  TCA_REG_TCPC_CONNECTOR_ORIENTATION_SET(orientation) |
		  TCA_REG_TCPC_LOW_POWER_EN_SET(low_pwr_en) |
		  TCA_REG_TCPC_VALID_SET(valid);
	u32 mask = TCA_REG_TCPC_MUX_CONTROL_MASK |
		   TCA_REG_TCPC_CONNECTOR_ORIENTATION_MASK |
		   TCA_REG_TCPC_LOW_POWER_EN_MASK | TCA_REG_TCPC_VALID_MASK;

	/* Start USB to DP4 : COMMON */
	/* read : 0x00000001 */
	cal_log_debug(0, "TCA_REG_TCA_TCPC(0x1 == 0x%08x)\n",
		      dp_phy_tca_read(SST1, TCA_REG_TCA_TCPC));

	dp_phy_tca_write_mask(SST1, TCA_REG_TCA_TCPC, val, mask);

	cal_log_debug(0, "TCA_REG_TCA_TCPC(0x%08x)\n",
		      dp_phy_tca_read(SST1, TCA_REG_TCA_TCPC));
}

/* USBDP PHY Common Registers */
static void dpphy_reg_set_cr_clk_high(void)
{
	dp_phy_write_mask(SST1, PHY_CR_PARA_CON0, PHY0_CR_PARA_CLK_SET(1),
			  PHY0_CR_PARA_CLK_MASK);
	udelay(1);
}

static void dpphy_reg_set_cr_clk_low(void)
{
	dp_phy_write_mask(SST1, PHY_CR_PARA_CON0, PHY0_CR_PARA_CLK_SET(0),
			  PHY0_CR_PARA_CLK_MASK);
	udelay(1);
}

static bool dpphy_reg_toggle_cr_clk(int cycle_cnt, bool check_ack)
{
	int clk_toggle_cycle;

	for (clk_toggle_cycle = 0; clk_toggle_cycle < cycle_cnt;
	     clk_toggle_cycle++) {
		dpphy_reg_set_cr_clk_high();
		dpphy_reg_set_cr_clk_low();
		if (check_ack) {
			if (dp_phy_read_mask(SST1, PHY_CR_PARA_CON0,
					     PHY0_CR_PARA_ACK_MASK)) {
				return true;
			}
		}
	}

	return !check_ack;
}

static void dpphy_reg_clear_cr_port(void)
{
	dp_phy_write(SST1, PHY_CR_PARA_CON1, 0x0);
	dp_phy_write(SST1, PHY_CR_PARA_CON2, 0x0);
	dp_phy_write(SST1, PHY_CR_PARA_CON0, 0x1);

	do {
		dpphy_reg_toggle_cr_clk(1, false);
		if (dp_phy_read_mask(SST1, PHY_CR_PARA_CON0,
				     PHY0_CR_PARA_ACK_MASK) == 0)
			break;
	} while (1);

	dp_phy_write_mask(SST1, PHY_CR_PARA_CON0, 0, PHY0_CR_PARA_CLK_MASK);
}

static void dpphy_reg_cr_write(u16 addr, u16 data)
{
	u32 clk_cycle = 0;

	dpphy_reg_clear_cr_port();

	dp_phy_write_mask(SST1, PHY_CR_PARA_CON0,
			  PHY0_CR_PARA_ADDR_SET(addr) | PHY0_CR_PARA_CLK_SET(1),
			  PHY0_CR_PARA_ADDR_MASK | PHY0_CR_PARA_CLK_MASK);
	dpphy_reg_set_cr_clk_low();

	dp_phy_write_mask(SST1, PHY_CR_PARA_CON2,
			  PHY0_CR_PARA_WR_DATA_SET(data) |
				  PHY0_CR_PARA_WR_EN_SET(1),
			  PHY0_CR_PARA_WR_DATA_MASK | PHY0_CR_PARA_WR_EN_MASK);

	dpphy_reg_set_cr_clk_high();
	dpphy_reg_set_cr_clk_low();

	dp_phy_write_mask(SST1, PHY_CR_PARA_CON2, PHY0_CR_PARA_WR_EN_SET(0),
			  PHY0_CR_PARA_WR_EN_MASK);

	do {
		dpphy_reg_set_cr_clk_high();
		if (dp_phy_read_mask(SST1, PHY_CR_PARA_CON0,
				     PHY0_CR_PARA_ACK_MASK))
			break;
		dpphy_reg_set_cr_clk_low();
		clk_cycle++;
	} while (clk_cycle < 10);

	if (clk_cycle == 10)
		cal_log_debug(0, "CR write failed to (0x%04x)\n", addr);
	else
		dpphy_reg_set_cr_clk_low();
}

static u16 dpphy_reg_cr_read(u16 addr, int *err)
{
	u32 clk_cycle = 0;

	dpphy_reg_clear_cr_port();

	dp_phy_write_mask(SST1, PHY_CR_PARA_CON0,
			  PHY0_CR_PARA_ADDR_SET(addr) | PHY0_CR_PARA_CLK_SET(1),
			  PHY0_CR_PARA_ADDR_MASK | PHY0_CR_PARA_CLK_MASK);
	dp_phy_write_mask(SST1, PHY_CR_PARA_CON1, PHY0_CR_PARA_RD_EN_SET(1),
			  PHY0_CR_PARA_RD_EN_MASK);

	dpphy_reg_set_cr_clk_low();
	dpphy_reg_set_cr_clk_high();

	dp_phy_write_mask(SST1, PHY_CR_PARA_CON1, PHY0_CR_PARA_RD_EN_SET(0),
			  PHY0_CR_PARA_RD_EN_MASK);

	dpphy_reg_set_cr_clk_low();

	do {
		dpphy_reg_set_cr_clk_high();
		if (dp_phy_read_mask(SST1, PHY_CR_PARA_CON0,
				     PHY0_CR_PARA_ACK_MASK))
			break;
		dpphy_reg_set_cr_clk_low();
		clk_cycle++;
	} while (clk_cycle < 10);

	if (clk_cycle == 10) {
		cal_log_debug(0, "CR read failed to (0x%04x)\n", addr);
		*err = 1;
		return -1;
	} else {
		dpphy_reg_set_cr_clk_low();
		*err = 0;
		return (u16)(PHY0_CR_PARA_RD_DATA_GET(
			dp_phy_read(SST1, PHY_CR_PARA_CON1)));
	}
}

static void dpphy_reg_cr_write_mask(u16 addr, u16 data, u16 mask)
{
	int err = 0;
	u16 old = 0;

	old = dpphy_reg_cr_read(addr, &err);
	cal_log_debug(0, "[CR][%04X][%04X]", addr, old);

	data = (data & mask) | (old & ~mask);
	dpphy_reg_cr_write(addr, data);

	old = dpphy_reg_cr_read(addr, &err);
	cal_log_debug(0, "[CR][%04X][%04X]", addr, old);
}

/*
 * Boost charge pump bias current
 * [10:8] mpllb_ctr_cp_int_ref
 * [2:0] mplla_ctr_cp_int_ref
 */
static void dpphy_reg_enable_cp_current_boost(void)
{
	dpphy_reg_cr_write_mask(0x5c, 0x400, 0x700);
	dpphy_reg_cr_write_mask(0x5c, 0x4, 0x7);
}

/* fix abnormal lane2 signal */
static void dpphy_reg_usb_tune_reset(enum plug_orientation orient)
{
	if (orient == PLUG_NORMAL) {
		dpphy_reg_cr_write_mask(0x1003, (0x0 << 13) | (0x34 << 7),
					(0x1 << 13) | (0x3F << 7));
		dpphy_reg_cr_write_mask(0x1002, (0x0 << 10) | (0xa << 4),
					(0x1 << 10) | (0x3F << 4));
	} else {
		dpphy_reg_cr_write_mask(0x1303, (0x0 << 13) | (0x34 << 7),
					(0x1 << 13) | (0x3F << 7));
		dpphy_reg_cr_write_mask(0x1302, (0x0 << 10) | (0xa << 4),
					(0x1 << 10) | (0x3F << 4));
	}
}

/* USBDP PHY DisplayPort Registers */
static void dpphy_reg_set_aux_enable(u32 en)
{
	dp_phy_write_mask(SST1, DP_AUX_CONFIG0, AUX_PWDNB_SET(en),
			  AUX_PWDNB_MASK);
}

static void dpphy_reg_set_config1(u32 cp_int, u32 cp_int_gs, u32 cp_prop,
				  u32 cp_prop_gs)
{
	u32 val = DP_MPLLB_CP_INT_SET(cp_int) |
		  DP_MPLLB_CP_INT_GS_SET(cp_int_gs) |
		  DP_MPLLB_CP_PROP_SET(cp_prop) |
		  DP_MPLLB_CP_PROP_GS_SET(cp_prop_gs);

	dp_phy_write(SST1, DP_CONFIG1, val);
}

static void dpphy_reg_set_config2(u32 div5_clk_en, u32 div_clk_en,
				  u32 div_multiplier, u32 force_en,
				  u32 force_ack, u32 fracn_cfg_update_en,
				  u32 fracn_en, u32 fracn_den)
{
	u32 val = DP_MPLLB_DIV5_CLK_EN_SET(div5_clk_en) |
		  DP_MPLLB_DIV_CLK_EN_SET(div_clk_en) |
		  DP_MPLLB_DIV_MULTIPLIER_SET(div_multiplier) |
		  DP_MPLLB_FORCE_EN_SET(force_en) |
		  DP_MPLLB_FORCE_ACK_SET(force_ack) |
		  DP_MPLLB_FRACN_CFG_UPDATE_EN_SET(fracn_cfg_update_en) |
		  DP_MPLLB_FRACN_EN_SET(fracn_en) |
		  DP_MPLLB_FRACN_DEN_SET(fracn_den);

	dp_phy_write(SST1, DP_CONFIG2, val);
}

static void dpphy_reg_set_config3(u32 fracn_quot, u32 fracn_rem)
{
	u32 val = DP_MPLLB_FRACN_QUOT_SET(fracn_quot) |
		  DP_MPLLB_FRACN_REM_SET(fracn_rem);

	dp_phy_write(SST1, DP_CONFIG3, val);
}

static void dpphy_reg_set_config4(u32 freq_vco, u32 init_cal_disable,
				  u32 multiplier, u32 pmix_en, u32 ssc_en)
{
	u32 val = DP_MPLLB_FREQ_VCO_SET(freq_vco) |
		  DP_MPLLB_INIT_CAL_DISABLE_SET(init_cal_disable) |
		  DP_MPLLB_MULTIPLIER_SET(multiplier) |
		  DP_MPLLB_PMIX_EN_SET(pmix_en) | DP_MPLLB_SSC_EN_SET(ssc_en);

	dp_phy_write(SST1, DP_CONFIG4, val);
}

static void dpphy_reg_set_config5(u32 ssc_peak)
{
	dp_phy_write_mask(SST1, DP_CONFIG5, DP_MPLLB_SSC_PEAK_SET(ssc_peak),
			  DP_MPLLB_SSC_PEAK_MASK);
}

static void dpphy_reg_set_config6(u32 ssc_stepsize)
{
	dp_phy_write_mask(SST1, DP_CONFIG6,
			  DP_MPLLB_SSC_STEPSIZE_SET(ssc_stepsize),
			  DP_MPLLB_SSC_STEPSIZE_MASK);
}

static void dpphy_reg_set_config7(u32 ssc_up_spread, u32 tx_clk_div, u32 v2i,
				  u32 word_div2_en)
{
	u32 val = DP_MPLLB_SSC_UP_SPREAD_SET(ssc_up_spread) |
		  DP_MPLLB_TX_CLK_DIV_SET(tx_clk_div) | DP_MPLLB_V2I_SET(v2i) |
		  DP_MPLLB_WORD_DIV2_EN_SET(word_div2_en);
	u32 mask = DP_MPLLB_SSC_UP_SPREAD_MASK | DP_MPLLB_TX_CLK_DIV_MASK |
		   DP_MPLLB_V2I_MASK | DP_MPLLB_WORD_DIV2_EN_MASK;

	dp_phy_write_mask(SST1, DP_CONFIG7, val, mask);
}

static void dpphy_reg_set_config7_refclk_en(u32 en)
{
	dp_phy_write_mask(SST1, DP_CONFIG7, DP_REF_CLK_EN_SET(en),
			  DP_REF_CLK_EN_MASK);
}

static void dpphy_reg_set_config8_tx_eq_main(u32 eq_main_val, u32 eq_main_mask)
{
	dp_phy_write_mask(SST1, DP_CONFIG8, eq_main_val, eq_main_mask);
}

static void dpphy_reg_set_config9_tx_eq_post(u32 eq_post_val, u32 eq_post_mask)
{
	dp_phy_write_mask(SST1, DP_CONFIG9, eq_post_val, eq_post_mask);
}

static void dpphy_reg_set_config10_tx_eq_pre(u32 eq_pre_val, u32 eq_pre_mask)
{
	dp_phy_write_mask(SST1, DP_CONFIG10, eq_pre_val, eq_pre_mask);
}

static void dpphy_reg_set_config11_tx_pstate(u32 en, enum lane_usage lane)
{
	u32 val = DP_TX_PSTATE_SET_0LANES;

	/* P2(0b11) is power down */
	if (!en || lane == DP_USE_0_LANES)
		val = DP_TX_PSTATE_SET_0LANES;
	else if (lane == DP_USE_2_LANES)
		val = DP_TX_PSTATE_SET_2LANES;
	else if (lane == DP_USE_4_LANES)
		val = DP_TX_PSTATE_SET_4LANES;

	dp_phy_write_mask(SST1, DP_CONFIG11, val, DP_TX_PSTATE_MASK);
}

static void dpphy_reg_wait_config12_tx_ack(enum lane_usage lane)
{
	u32 ack_val = (lane == DP_USE_4_LANES) ? 0xF : 0x3;
	u32 val = 0;

	if (readl_poll_timeout_atomic(regs_dp[REGS_PHY][SST1].regs + DP_CONFIG12,
				val, !(DP_TX_ACK_GET(val) & ack_val), 10, 2000)) {
		cal_log_err(0, "is timeout. Fail to ack DP TX.\n");
		cal_log_err(0, "val(0x%08x) DP_CONFIG12 read:0x%08x\n", val,
			    readl(regs_dp[REGS_PHY][SST1].regs + DP_CONFIG12));
	}
}

static void dpphy_reg_set_config12_status_update(enum lane_usage lane)
{
	u32 req_val = (lane == DP_USE_4_LANES) ? 0xF : 0x3;
	u32 val = 0;

	/* Set DP TX Request */
	dp_phy_write_mask(SST1, DP_CONFIG12, DP_TX_REQ_SET(req_val), DP_TX_REQ_MASK);

	/* Wait for DP TX Request Clear */
	if (readl_poll_timeout_atomic(regs_dp[REGS_PHY][SST1].regs + DP_CONFIG12,
				val, !(DP_TX_REQ_GET(val) & req_val), 10, 2000)) {
		cal_log_err(0, "is timeout. Fail to request DP TX update.\n");
		cal_log_err(0, "val(0x%08x) DP_CONFIG12 read:0x%08x\n", val,
			    readl(regs_dp[REGS_PHY][SST1].regs + DP_CONFIG12));
	}

	/* Wait for TX Acknowledge to confirm DP TX request is applied */
	dpphy_reg_wait_config12_tx_ack(DP_USE_4_LANES);
}

static void dpphy_reg_set_config12_tx_mpllb_en(u32 en, enum lane_usage lane)
{
	u32 val = DP_TX_MPLL_EN_SET_0LANES;

	if (!en || lane == DP_USE_0_LANES)
		val = DP_TX_MPLL_EN_SET_0LANES;
	else if (lane == DP_USE_2_LANES)
		val = DP_TX_MPLL_EN_SET_2LANES;
	else if (lane == DP_USE_4_LANES)
		val = DP_TX_MPLL_EN_SET_4LANES;

	dp_phy_write_mask(SST1, DP_CONFIG12, val, DP_TX_MPLL_EN_MASK);
}

static void dpphy_reg_set_config12_tx_width(u32 en, enum lane_usage lane)
{
	u32 val = 0;

	/* "0b11" is 20bit DP TX lane width */
	if (!en || lane == DP_USE_0_LANES)
		val = DP_TX_WIDTH_SET_0LANES;
	else if (lane == DP_USE_2_LANES)
		val = DP_TX_WIDTH_SET_2LANES;
	else if (lane == DP_USE_4_LANES)
		val = DP_TX_WIDTH_SET_4LANES;

	dp_phy_write_mask(SST1, DP_CONFIG12, val, DP_TX_WIDTH_MASK);
}

static void dpphy_reg_set_config13_tx_reset(u32 en)
{
	dp_phy_write_mask(SST1, DP_CONFIG13,
			  en ? DP_TX_RESET_ALL : DP_TX_RESET_RELEASE,
			  DP_TX_RESET_MASK);
}

static void dpphy_reg_set_config13_tx_enable(u32 en, enum lane_usage lane)
{
	u32 val = 0;

	/* "0b1" disables DP TX lane, "0b0" enables DP TX lane */
	if (!en || lane == DP_USE_0_LANES)
		val = DP_TX_DISABLE_SET_4LANES;
	else if (lane == DP_USE_2_LANES)
		val = DP_TX_DISABLE_SET_2LANES;
	else if (lane == DP_USE_4_LANES)
		val = DP_TX_DISABLE_SET_0LANES;

	dp_phy_write_mask(SST1, DP_CONFIG13, val, DP_TX_DISABLE_MASK);
}

static void dpphy_reg_set_config17_dcc_byp_ac_cap(u32 en, enum lane_usage lane)
{
	u32 val = 0;

	if (!en || lane == DP_USE_0_LANES)
		val = DP_TX_DCC_BYP_AC_CAP_SET_0LANES;
	else if (lane == DP_USE_2_LANES)
		val = DP_TX_DCC_BYP_AC_CAP_SET_2LANES;
	else if (lane == DP_USE_4_LANES)
		val = DP_TX_DCC_BYP_AC_CAP_SET_4LANES;

	dp_phy_write_mask(SST1, DP_CONFIG17, val, DP_TX_DCC_BYP_AC_CAP_MASK);
}

static void dpphy_reg_set_config19_dpalt_disable_ack(u32 en)
{
	dp_phy_write_mask(SST1, DP_CONFIG19, DPALT_DISABLE_ACK_SET(en),
			  DPALT_DISABLE_ACK_MASK);
}

/* USBDP PHY TRSV Registers */
static void dpphy_reg_set_lane_tx_level(enum dp_link_rate_type link_rate,
					u32 lane_num, u32 amp_lvl,
					u32 pre_emp_lvl)
{
	u32 val_eq;
	u16 val_rboost, addr_rboost;
	u32 eq_main_val, eq_main_mask, eq_pre_val, eq_pre_mask, eq_post_val,
		eq_post_mask;
	int i = 0;

	for (i = EQ_MAIN; i <= RBOOST; i++) {
		if (link_rate <= LINK_RATE_HBR)
			val_eq = phy_eq_hbr0_1[amp_lvl][pre_emp_lvl][i];
		else
			val_eq = phy_eq_hbr2_3[amp_lvl][pre_emp_lvl][i];

		if (val_eq == -1)
			continue;

		if (i == EQ_MAIN) {
			eq_main_val = ((val_eq & 0x3F) << (8 * lane_num));
			eq_main_mask = (0x3F << (8 * lane_num));
			dpphy_reg_set_config8_tx_eq_main(eq_main_val,
							 eq_main_mask);
		} else if (i == EQ_PRE) {
			eq_pre_val = ((val_eq & 0x3F) << (8 * lane_num));
			eq_pre_mask = (0x3F << (8 * lane_num));
			dpphy_reg_set_config10_tx_eq_pre(eq_pre_val,
							 eq_pre_mask);
		} else if (i == EQ_POST) {
			eq_post_val = ((val_eq & 0x3F) << (8 * lane_num));
			eq_post_mask = (0x3F << (8 * lane_num));
			dpphy_reg_set_config9_tx_eq_post(eq_post_val,
							 eq_post_mask);
		} else if (i == EQ_VSWING) {
			dpphy_reg_cr_write((u16)0x22, (u16)0xD0);
		} else if (i == RBOOST) {
			if (link_rate > LINK_RATE_HBR)
				val_rboost = (u16)0x70;
			else
				val_rboost = (u16)0x40;

			addr_rboost = (u16)(0x1005 + (0x100 * lane_num));

			dpphy_reg_cr_write(addr_rboost, val_rboost);
		}
	}

	/* For lane control */
	dpphy_reg_cr_write((u16)0x1002, (u16)0x180);
	dpphy_reg_cr_write((u16)0x10EB, (u16)0x0);
}

/* USBDP PHY functions */
static void dpphy_reg_reset_tx_lanes(void)
{
	// Assert all TX lanes reset
	dpphy_reg_set_config13_tx_reset(1);

	// Disable all TX lanes' power state: P2
	dpphy_reg_set_config11_tx_pstate(0, DP_USE_4_LANES);
	cal_log_debug(0, "power off all lanes.\n");

	// De-Assert all TX lanes reset
	dpphy_reg_set_config13_tx_reset(0);

	// Disable all TX lanes
	dpphy_reg_set_config13_tx_enable(0, DP_USE_4_LANES);
	cal_log_debug(0, "disable all lanes.\n");

	// Wait TX_ACK De-Assert
	dpphy_reg_wait_config12_tx_ack(DP_USE_4_LANES);

	// Assert DP Alt-mode Disable ACK
	dpphy_reg_set_config19_dpalt_disable_ack(1);
}

static void dpphy_reg_reset_mpllb(u32 en, enum lane_usage num_lane)
{
	if (en) {
		// Assert USBDP PHY MPLLB reset
		dpphy_reg_set_config12_tx_mpllb_en(0, num_lane);
		dpphy_reg_set_config11_tx_pstate(0, num_lane);
		cal_log_debug(0, "disable mpllb and power off for %d lanes\n", num_lane);
	} else {
		// De-assert USBDP PHY MPLLB reset
		dpphy_reg_set_config12_tx_mpllb_en(1, num_lane);
		dpphy_reg_set_config11_tx_pstate(1, num_lane);
		cal_log_debug(0, "enable mpllb and power on for %d lanes\n", num_lane);
	}

	// Request to update TX status
	dpphy_reg_set_config12_status_update(num_lane);
	cal_log_debug(0, "status update for %d lanes.\n", num_lane);
}

static void dpphy_reg_set_mpllb(struct dp_hw_config *hw_config, bool reconfig)
{
	enum dp_link_rate_type link_rate = hw_config->link_rate;

	// CONFIG01
	u32 mpllb_cp_int = (u32)0xe, mpllb_cp_int_gs = 0, mpllb_cp_prop = 0,
	    mpllb_cp_prop_gs = (u32)0x7f;
	// CONFIG02
	u32 mpllb_div5_clk_en = 1, mpllb_div_cl_en = 0,
	    mpllb_div_multiplier = 0, mpllb_force_en = 0;
	u32 mpllb_force_ack = 0, mpllb_fracn_cfg_update_en = 1,
	    mpllb_fracn_en = 1, mpllb_fracn_den = 1;
	// CONFIG03
	u32 mpllb_fracn_quot = 0, mpllb_fracn_rem = 0;
	// CONFIG04
	u32 mpllb_freq_vco = 0, mpllb_init_cal_disable = 0,
	    mpllb_multiplier = 0, mpllb_pmix_en = 1, mpllb_ssc_en = 0;
	// CONFIG05
	u32 mpllb_ssc_peak = 0;
	// CONFIG06
	u32 mpllb_ssc_stepsize = 0;
	// CONFIG07
	u32 mpllb_ssc_up_spread = 0, mpllb_tx_clk_div = 0, mpllb_v2i = 0,
	    mpllb_word_div2_en = 0;
	u32 ref_clk_en = 1;

	if (hw_config->use_ssc) {
		mpllb_ssc_en = 1;
		cal_log_debug(0, "configure for SSC\n");
	}

	if (hw_config->dp_emul) {
		mpllb_force_en = 1;
		cal_log_debug(0, "configure for DP Emulation Mode\n");
	}

	cal_log_debug(0, "link_rate(%d)\n", link_rate);

	switch (link_rate) {
	case LINK_RATE_HBR3:
		mpllb_cp_int_gs = 0x43;
		mpllb_cp_prop = 0x19;
		mpllb_fracn_quot = 0xF000;
		mpllb_freq_vco = 0x2;
		mpllb_multiplier = 0x184;
		mpllb_tx_clk_div = 0x0;
		mpllb_v2i = 0x3;
		if (mpllb_ssc_en) {
			/* Max down-spread (-0.5%) for HBR3 SSC */
			mpllb_ssc_peak = 0x10E00;
			mpllb_ssc_stepsize = 0x1C59A;
		}
		break;
	case LINK_RATE_HBR2:
		mpllb_cp_int_gs = 0x43;
		mpllb_cp_prop = 0x14;
		mpllb_fracn_quot = 0xA000;
		mpllb_freq_vco = 0x3;
		mpllb_multiplier = 0xF8;
		mpllb_tx_clk_div = 0x0;
		mpllb_v2i = 0x3;
		if (mpllb_ssc_en) {
			/* Max down-spread (-0.5%) for HBR2 SSC */
			mpllb_ssc_peak = 0xB400;
			mpllb_ssc_stepsize = 0x12E66;
		}
		break;
	case LINK_RATE_HBR:
		mpllb_cp_int_gs = 0x43;
		mpllb_cp_prop = 0x14;
		mpllb_fracn_quot = 0xA000;
		mpllb_freq_vco = 0x3;
		mpllb_multiplier = 0xF8;
		mpllb_tx_clk_div = 0x1;
		mpllb_v2i = 0x3;
		if (mpllb_ssc_en) {
			/* Max down-spread (-0.5%) for HBR SSC */
			mpllb_ssc_peak = 0xB400;
			mpllb_ssc_stepsize = 0x12E66;
		}
		break;
	case LINK_RATE_RBR:
	default:
		mpllb_cp_int_gs = 0x41;
		mpllb_cp_prop = 0x1C;
		mpllb_fracn_quot = 0xC000;
		mpllb_freq_vco = 0x3;
		mpllb_multiplier = 0x130;
		mpllb_tx_clk_div = 0x2;
		mpllb_v2i = 0x2;
		if (mpllb_ssc_en) {
			/* Max down-spread (-0.5%) for RBR SSC */
			mpllb_ssc_peak = 0xD800;
			mpllb_ssc_stepsize = 0x16AE1;
		}
		break;
	}

	// Configure Master PLL-B for DP
	dpphy_reg_set_config1(mpllb_cp_int, mpllb_cp_int_gs, mpllb_cp_prop,
			      mpllb_cp_prop_gs);
	dpphy_reg_set_config2(mpllb_div5_clk_en, mpllb_div_cl_en,
			      mpllb_div_multiplier, mpllb_force_en,
			      mpllb_force_ack, mpllb_fracn_cfg_update_en,
			      mpllb_fracn_en, mpllb_fracn_den);
	dpphy_reg_set_config3(mpllb_fracn_quot, mpllb_fracn_rem);
	dpphy_reg_set_config4(mpllb_freq_vco, mpllb_init_cal_disable,
			      mpllb_multiplier, mpllb_pmix_en, mpllb_ssc_en);
	dpphy_reg_set_config5(mpllb_ssc_peak);
	dpphy_reg_set_config6(mpllb_ssc_stepsize);
	dpphy_reg_set_config7(mpllb_ssc_up_spread, mpllb_tx_clk_div, mpllb_v2i,
			      mpllb_word_div2_en);

	// Configure Reference Clock for DP
	dpphy_reg_set_config7_refclk_en(ref_clk_en);

	/* Enable the bypassing AC Capacitor for all lanes */
	dpphy_reg_set_config17_dcc_byp_ac_cap(1, DP_USE_4_LANES);
}

static enum lane_usage dp_get_and_inform_lanes(struct dp_hw_config *hw_config)
{
	enum lane_usage num_lane = DP_USE_0_LANES;
	u32 pin_flag = 0;

	switch (hw_config->pin_type) {
	case PIN_TYPE_C:
		pin_flag = 1;
		fallthrough;
	case PIN_TYPE_E:
		/* 4 Lanes Mode Configuration */
		cal_log_debug(0, "PIN_TYPE_%s\n", pin_flag ? "C" : "E");
		num_lane = DP_USE_4_LANES;
		break;

	case PIN_TYPE_D:
		pin_flag = 1;
		fallthrough;
	case PIN_TYPE_F:
		/* 2 Lanes Mode Configuration */
		cal_log_debug(0, "PIN_TYPE_%s\n", pin_flag ? "D" : "F");
		num_lane = DP_USE_2_LANES;
		break;

	default:
		/*
		 * DP should get the Pin type CDEF from PDIC notifier
		 * So, Error handler code implemented to here
		 */
		cal_log_err(0, "UNKNOWN DP PIN TYPE\n");
		break;
	}

	return num_lane;
}

static int dpphy_reg_switch_to_dp(enum lane_usage num_lane)
{
	u32 tcpc_mux_con = NO_CONNECTION, tcpc_orientation = NORMAL;
	u32 tcpc_low_pwr_en = STANDARD_OPERATION, tcpc_valid = VALID_I;

	if (num_lane == DP_USE_4_LANES)
		tcpc_mux_con = DP_ALT4;
	else if (num_lane == DP_USE_2_LANES)
		tcpc_mux_con = USB31_DP_ALT2;
	else
		cal_log_err(0, "abnormal lanes\n");

	/* Start USB to DP4 : COMMON */
	udelay(1000);

	/*
	 * Both Samsung DP Link and Synopsys DP PHY can support pin map and orientation.
	 * To avoid unexpected configuration conflict, DP Link use only default pin map and orientation.
	 */
	cal_log_debug(0, "Start USB to DP%d\n", num_lane);
	dpphytca_reg_set_tcpc(tcpc_mux_con, tcpc_orientation, tcpc_low_pwr_en,
			      tcpc_valid);
	return dpphytca_reg_wait_mode_change(num_lane);
}

static void dpphy_reg_set_lanes(struct dp_hw_config *hw_config, enum lane_usage num_lane)
{
	// Enable TX lanes
	dpphy_reg_set_config13_tx_enable(1, num_lane);
	cal_log_debug(0, "enable %d lanes.\n", num_lane);

	// De-assert DP Alt-mode Disable ACK
	dpphy_reg_set_config19_dpalt_disable_ack(0);

	// Update TX Width & Enable TX MPLLB
	dpphy_reg_set_config12_tx_width(1, num_lane);
	dpphy_reg_set_config12_tx_mpllb_en(1, num_lane);
	cal_log_debug(0, "enable mpllb for %d lanes.\n", num_lane);

	dpphy_reg_set_config12_status_update(num_lane);
	cal_log_debug(0, "status update for %d lanes.\n", num_lane);
}

static int dpphy_reg_init(struct dp_hw_config *hw_config, bool reconfig)
{
	int ret;
	enum lane_usage num_lane = dp_get_and_inform_lanes(hw_config);

	//dpphy_reg_usbdrd_qch_en(1);
	cal_log_debug(0, "dpphy_reg_usbdrd_qch_en is skipped.\n");

	if (reconfig)
		dpphy_reg_reset_mpllb(1, num_lane);

	/* Enable AUX channel */
	dpphy_reg_set_aux_enable(1);
	cal_log_debug(0, "enable DP AUX.\n");

	/* Reset All DP TX lanes */
	dpphy_reg_reset_tx_lanes();
	cal_log_debug(0, "reset All DP TX lanes.\n");

	/* CP(Charge Pump) Bias Boosting X2 */
	if (hw_config->phy_boost && !reconfig)
		dpphy_reg_enable_cp_current_boost();

	/* Set Master PLL-B for DP as Link_BW */
	dpphy_reg_set_mpllb(hw_config, reconfig);
	cal_log_debug(0, "set MPLLB as link_bw.\n");

	/* Switch from USB to DP */
	ret = dpphy_reg_switch_to_dp(num_lane);
	if (ret)
		return ret;
	cal_log_debug(0, "switch from USB to DP.\n");

	/* Set DP TX lanes */
	dpphy_reg_set_lanes(hw_config, num_lane);
	cal_log_debug(0, "set DP TX lanes.\n");

	if (reconfig)
		dpphy_reg_reset_mpllb(0, num_lane);

	return 0;
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

	udelay(1);

	// After software reset sequence is done, CPU needs to write this bit to 0.
	dp_write_mask(SST1, SYSTEM_SW_RESET_CONTROL, SW_RESET_RELEASE,
		      SW_RESET_MASK);

	if (readl_poll_timeout_atomic(regs_dp[REGS_LINK][SST1].regs +
					      SYSTEM_SW_RESET_CONTROL,
				      val, !SW_RESET_GET(val), 10, 2000))
		cal_log_err(0, "is timeout.\n");
}

// System Clock Control Configuration
static u32 dp_reg_get_gfmux_status(void)
{
	return GFMUX_STATUS_TXCLK_GET(dp_read_mask(SST1, SYSTEM_CLK_CONTROL,
						   GFMUX_STATUS_TXCLK_MASK));
}

static void dp_reg_set_txclk_osc(void)
{
	dp_write_mask(SST1, SYSTEM_CLK_CONTROL,
		      GFCLKMUX_SEL_10_OSC | GFCLKMUX_SEL_20_OSC,
		      GFCLKMUX_SEL_10_MASK | GFCLKMUX_SEL_20_MASK);
}

static void dp_reg_set_txclk_txclk(void)
{
	dp_write_mask(SST1, SYSTEM_CLK_CONTROL,
		      GFCLKMUX_SEL_10_TX | GFCLKMUX_SEL_20_TX,
		      GFCLKMUX_SEL_10_MASK | GFCLKMUX_SEL_20_MASK);
}

static void dp_reg_set_oscclk_qch_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_OSCLK_QCH_FUNCTION_ENABLE,
		      OSCCLK_QCH_FUNC_EN_SET(en), OSCCLK_QCH_FUNC_EN_MASK);
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
	dp_reg_set_pcs_func_en(1);
	dp_reg_set_aux_func_en(1);
}

// System SST1 Function Enable Configuration
static int dp_reg_wait_sst1_longhop_power_on(void)
{
	u32 val;

	if (readl_poll_timeout_atomic(
		    regs_dp[REGS_LINK][SST1].regs + SYSTEM_SST1_FUNCTION_ENABLE,
		    val, SST1_LH_PWR_ON_STATUS_GET(val), 10, 200)) {
		cal_log_err(0, "is timeout.\n");
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
static void dp_reg_set_hpd_force(void)
{
	u32 val = HPD_FORCE_EN_SET(1) | HPD_FORCE_SET(1) | HPD_EVENT_CTRL_EN_BY_HOST;
	u32 mask = HPD_FORCE_EN_MASK | HPD_FORCE_MASK | HPD_EVENT_CTRL_EN_MASK;

	dp_write_mask(SST1, SYSTEM_HPD_CONTROL, val, mask);
}

// System PLL Lock Control Configuration
static int dp_reg_wait_phy_pll_lock(void)
{
	u32 val;

	if (readl_poll_timeout_atomic(regs_dp[REGS_LINK][SST1].regs +
					      SYSTEM_PLL_LOCK_CONTROL,
				      val, PLL_LOCK_STATUS_GET(val), 10, 200)) {
		cal_log_err(0, "is timeout.\n");
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
static void dp_reg_set_aux_reply_timeout_and_retries(void)
{
	/*
	 * 1800 us timeout and no hardware retries.
	 * Additional 1400 us wait will be done in software
	 * to comply with DP CTS test 4.2.1.1 requirement.
	 */
	dp_write_mask(SST1, AUX_CONTROL,
		      AUX_REPLY_TIMER_MODE_1800US | AUX_RETRY_TIMER_SET(0),
		      AUX_REPLY_TIMER_MODE_MASK | AUX_RETRY_TIMER_MASK);
}

static void dp_reg_set_aux_pn(enum plug_orientation orient, bool aux_auto_orientation)
{
	u32 val;

	/*
	 * When aux_auto_orientation is true, then the AUX pin is set by
	 * the USB stack so set to normal unconditionally.
	 */
	if (aux_auto_orientation)
		val = AUX_PN_NORMAL;
	else
		val = (orient == PLUG_FLIPPED) ? AUX_PN_INVERT : AUX_PN_NORMAL;

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
		cal_log_err(0, "is timeout.\n");
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

		if (AUX_CMD_STATUS_GET(val0) == AUX_CMD_STATUS_TIMEOUT_ERROR) {
			cal_log_err(0, "DP Sink didn't reply in 1.8ms. Retry after 1.4ms.\n");
			usleep_range(1400, 1410);
			return -ETIME;
		} else {
			usleep_range(400, 410);
			return -EIO;
		}
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
static void dp_reg_set_fec_ready(u32 en)
{
	dp_write_mask(SST1, PCS_CONTROL, FEC_READY_SET(en), FEC_READY_MASK);
}

static void dp_reg_set_fec_func_en(u32 en)
{
	dp_write_mask(SST1, PCS_CONTROL, FEC_FUNC_EN_SET(en), FEC_FUNC_EN_MASK);
}

static void dp_reg_set_training_pattern(dp_training_pattern pattern)
{
	dp_write_mask(SST1, PCS_CONTROL, LINK_TRAINING_PATTERN_SET(pattern),
		      LINK_TRAINING_PATTERN_MASK);
}

static void dp_reg_set_bit_swap(u32 en)
{
	dp_write_mask(SST1, PCS_CONTROL, BIT_SWAP_SET(en), BIT_SWAP_MASK);
}

static void dp_reg_set_scramble_bypass(dp_scrambling scram)
{
	dp_write_mask(SST1, PCS_CONTROL, SCRAMBLE_BYPASS_SET(scram),
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
	/*
	 * Both Samsung DP Link and Synopsys DP PHY can support pin map and orientation.
	 * To avoid unexpected configuration conflict, DP Link use only default pin map and orientation.
	 */
	switch (hw_config->pin_type) {
	case PIN_TYPE_C:
	case PIN_TYPE_E:
		/* 4 Lanes Mode Configuration */
		dpphy_reg_usb_tune_reset(hw_config->orient_type); // TBD
		break;
	case PIN_TYPE_D:
	case PIN_TYPE_F:
	default:
		break;
	}

	dp_reg_set_lane_map(LOGICAL_LANE_0, LOGICAL_LANE_1, LOGICAL_LANE_2,
			    LOGICAL_LANE_3);
}

// PCS Test Pattern Control
static void dp_reg_set_quality_pattern(dp_qual_pattern pattern)
{
	dp_write_mask(SST1, PCS_TEST_PATTERN_CONTROL,
		      LINK_QUALITY_PATTERN_SET(pattern), LINK_QUALITY_PATTERN_MASK);
}

static void dp_reg_set_custom_pattern(void)
{
	dp_write(SST1, PCS_TEST_PATTERN_SET0, 0x3E0F83E0);  // 00111110 00001111 10000011 11100000
	dp_write(SST1, PCS_TEST_PATTERN_SET1, 0x0F83E0F8);  // 00001111 10000011 11100000 11111000
	dp_write(SST1, PCS_TEST_PATTERN_SET2, 0x0000F83E);  // 11111000 00111110
}

static void dp_reg_set_hbr2_scrambler_reset(u32 uResetCount)
{
	dp_write_mask(SST1, PCS_HBR2_EYE_SR_CONTROL,
		      HBR2_EYE_SR_COUNT_SET(uResetCount), HBR2_EYE_SR_COUNT_MASK);
}

// Synopsys Data Path Control
static void dp_reg_set_snps_tx_clk_en(enum lane_usage lane)
{
	u32 val = 0;

	if (lane == DP_USE_2_LANES)
		val = SNPS_TX0_CLK_EN | SNPS_TX1_CLK_EN;
	else if (lane == DP_USE_4_LANES)
		val = SNPS_TX0_CLK_EN | SNPS_TX1_CLK_EN | SNPS_TX2_CLK_EN |
		      SNPS_TX3_CLK_EN;

	dp_write_mask(SST1, PCS_SNPS_PHY_DATAPATH_CONTROL, val,
		      SNPS_TX_CLK_EN_MASK);
}

static void dp_reg_set_snps_tx_clk_rdy(enum lane_usage lane)
{
	u32 val = 0;

	if (lane == DP_USE_2_LANES)
		val = SNPS_TX0_CLK_RDY | SNPS_TX1_CLK_RDY;
	else if (lane == DP_USE_4_LANES)
		val = SNPS_TX0_CLK_RDY | SNPS_TX1_CLK_RDY | SNPS_TX2_CLK_RDY |
		      SNPS_TX3_CLK_RDY;

	dp_write_mask(SST1, PCS_SNPS_PHY_DATAPATH_CONTROL, val,
		      SNPS_TX_CLK_RDY_MASK);
}

static void dp_reg_set_snps_tx_data_en(enum lane_usage lane)
{
	u32 val = 0;

	if (lane == DP_USE_2_LANES)
		val = SNPS_TX0_DATA_EN | SNPS_TX1_DATA_EN;
	else if (lane == DP_USE_4_LANES)
		val = SNPS_TX0_DATA_EN | SNPS_TX1_DATA_EN | SNPS_TX2_DATA_EN |
		      SNPS_TX3_DATA_EN;

	dp_write_mask(SST1, PCS_SNPS_PHY_DATAPATH_CONTROL, val,
		      SNPS_TX_DATA_EN_MASK);
}

static void dp_hw_set_data_path(struct dp_hw_config *hw_config)
{
	enum lane_usage num_lane = DP_USE_0_LANES;

	switch (hw_config->pin_type) {
	case PIN_TYPE_C:
	case PIN_TYPE_E:
		/* 4 Lanes Mode Configuration */
		num_lane = DP_USE_4_LANES;
		break;
	case PIN_TYPE_D:
	case PIN_TYPE_F:
		/* 2 Lanes Mode Configuration */
		num_lane = DP_USE_2_LANES;
		break;
	default:
		break;
	}

	// SNPS PHY Clock Ready
	dp_reg_set_snps_tx_clk_rdy(num_lane);

	// SNPS PHY Clock Enable
	dp_reg_set_snps_tx_clk_en(num_lane);

	// Power on TX lanes
	dpphy_reg_set_config11_tx_pstate(1, num_lane);
	cal_log_debug(0, "power on %d lanes.\n", num_lane);

	dpphy_reg_set_config12_status_update(num_lane);
	cal_log_debug(0, "status update for %d lanes.\n", num_lane);

	// Adjust DCC Bypassing AC Capacitor
	if (hw_config->link_rate > LINK_RATE_RBR) {
		/*
		 * Disable the bypassing AC capacitor for HBRx mode only.
		 * RBR mode is using the bypassing AC capacitor as default.
		 */
		dpphy_reg_set_config17_dcc_byp_ac_cap(0, num_lane);
	}

	// SNPS PHY Data Enable
	dp_reg_set_snps_tx_data_en(num_lane);
}

/* HDCP Control Registers */
// HDCP 1.3
static bool dp_reg_get_hdcp13_aksv_valid(void)
{
	return AKSV_VALID_GET(dp_read(SST1, HDCP13_STATUS)) ? true : false;
}

static void dp_reg_set_hdcp13_store_an(void)
{
	dp_write_mask(SST1, HDCP13_CONTROL_0, SW_STORE_AN_STOP_PRNG, SW_STORE_AN_MASK);

	/* Save the 64 bit random number to AN during PRNG Stop */

	dp_write_mask(SST1, HDCP13_CONTROL_0, SW_STORE_AN_START_PRNG, SW_STORE_AN_MASK);
}

static void dp_reg_set_hdcp13_repeater(u32 en)
{
	dp_write_mask(SST1, HDCP13_CONTROL_0, SW_RX_REPEATER_SET(en), SW_RX_REPEATER_MASK);
}

static void dp_reg_set_hdcp13_encryption_enable(u32 en)
{
	dp_write_mask(SST1, HDCP13_CONTROL_0,
		      SW_AUTH_OK_SET(en) | HDCP13_ENC_EN_SET(en),
		      SW_AUTH_OK_MASK | HDCP13_ENC_EN_MASK);
}

static void dp_reg_get_hdcp13_aksv(u32 *aksv0, u32 *aksv1)
{
	*aksv0 = dp_read(SST1, HDCP13_AKSV_0);
	*aksv1 = dp_read(SST1, HDCP13_AKSV_1);
}

static void dp_reg_get_hdcp13_an(u32 *an0, u32 *an1)
{
	*an0 = dp_read(SST1, HDCP13_AN_0);
	*an1 = dp_read(SST1, HDCP13_AN_1);
}

static void dp_reg_set_hdcp13_bksv(u32 bksv0, u32 bksv1)
{
	dp_write(SST1, HDCP13_BKSV_0, bksv0);
	dp_write(SST1, HDCP13_BKSV_1, bksv1);
}

static void dp_reg_get_hdcp13_r0(u32 *r0)
{
	*r0 = dp_read(SST1, HDCP13_R0);
}

static void dp_reg_get_hdcp13_am0(u32 *am0, u32 *am1)
{
	*am0 = dp_read(SST1, HDCP13_AM0_0);
	*am1 = dp_read(SST1, HDCP13_AM0_1);
}

static u32 dp_reg_get_hdcp13_key_valid(void)
{
	return KEY_VALID_SYNC_IN_I2C_CLK_GET(dp_read(SST1, HDCP13_KEY_VALID_STATUS));
}

// HDCP 2.2
static void dp_reg_set_hdcp22_system_enable(u32 en)
{
	dp_write_mask(SST1, HDCP22_SYS_EN, SYSTEM_ENABLE_SET(en), SYSTEM_ENABLE_MASK);
}

static void dp_reg_set_hdcp22_encryption_enable(u32 en)
{
	dp_write_mask(SST1, HDCP22_CONTROL, HDCP22_ENC_EN_SET(en), HDCP22_ENC_EN_MASK);
}

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
static void dp_reg_set_video_mn_value(u32 mvid_master, u32 nvid_master)
{
	dp_write(SST1, SST1_MVID_MASTER_MODE, mvid_master);
	dp_write(SST1, SST1_NVID_MASTER_MODE, nvid_master);
}

static void dp_reg_set_video_mn_config(u32 mvid, u32 nvid)
{
	dp_write_mask(SST1, SST1_MVID_SFR_CONFIGURE, mvid, MNVID_SFR_CONFIG_MASK);
	dp_write_mask(SST1, SST1_NVID_SFR_CONFIGURE, nvid, MNVID_SFR_CONFIG_MASK);
}

// SST1 Audio M/N Value
static void dp_reg_set_audio_mn_value(u32 maud_master, u32 naud_master)
{
	dp_write(SST1, SST1_MAUD_MASTER_MODE, maud_master);
	dp_write(SST1, SST1_NAUD_MASTER_MODE, naud_master);
}

static void dp_reg_set_audio_mn_config(u32 maud, u32 naud)
{
	dp_write_mask(SST1, SST1_MAUD_SFR_CONFIGURE, maud, MNAUD_SFR_CONFIG_MASK);
	dp_write_mask(SST1, SST1_NAUD_SFR_CONFIGURE, naud, MNAUD_SFR_CONFIG_MASK);
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
	u32 strm_clk = hw_config->vtiming.clock;  /* KHz unit */
	u32 ls_clk = dp_get_ls_clk(hw_config);  /* Hz unit */

	/*
	 * Adjust ls_clk when SSC is enabled.
	 * Max downspread is 0.5%, so use 0.25% average adjustment.
	 */
	if (hw_config->use_ssc)
		ls_clk = (ls_clk / 10000) * 9975;

	ls_clk = ls_clk / 1000;  /* KHz unit */

	/* Synopsys PHY needs mvid_master = stream clock / 4 */
	dp_reg_set_video_mn_value(strm_clk / 4, ls_clk);
	dp_reg_set_video_mn_config(strm_clk, ls_clk);
}

static void dp_reg_set_audio_clock(struct dp_hw_config *hw_config, enum audio_mode mode)
{
	u32 audio_fs = m_aud_master[hw_config->audio_fs];  /* Hz unit */
	u32 ls_clk = dp_get_ls_clk(hw_config);  /* Hz unit */
	u32 m_value, n_value;

	if (mode == AUDIO_MODE_ASYNC) {
		m_value = audio_async_m_n[0][hw_config->link_rate][hw_config->audio_fs];
		n_value = audio_async_m_n[1][hw_config->link_rate][hw_config->audio_fs];
	} else {
		m_value = audio_sync_m_n[0][hw_config->link_rate][hw_config->audio_fs];
		n_value = audio_sync_m_n[1][hw_config->link_rate][hw_config->audio_fs];
	}

	/*
	 * Adjust ls_clk when SSC is enabled.
	 * Max downspread is 0.5%, so use 0.25% average adjustment.
	 */
	if (hw_config->use_ssc)
		ls_clk = (ls_clk / 10000) * 9975;

	/* Synopsys PHY needs naud_master = LS clock / 2 */
	dp_reg_set_audio_mn_value(audio_fs, ls_clk / 2);
	dp_reg_set_audio_mn_config(m_value, n_value);
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

	return dp_reg_check_aux_monitors();
}

static int dp_write_dpcd(u32 address, u32 length, u8 *data)
{
	int ret;
	int retry_cnt = AUX_RETRY_COUNT;

	while (retry_cnt > 0) {
		dp_reg_aux_ch_buf_clr();
		dp_reg_aux_defer_ctrl(1);
		dp_reg_set_aux_reply_timeout_and_retries();
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
		dp_reg_set_aux_reply_timeout_and_retries();
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
	u8 segment = block_cnt / 2;
	u8 offset = (block_cnt & 1) * EDID_BLOCK_SIZE;

	while (retry_cnt > 0) {
		dp_reg_aux_ch_buf_clr();
		dp_reg_aux_defer_ctrl(1);
		dp_reg_set_aux_reply_timeout_and_retries();
		dp_reg_set_aux_ch_address_only_command(false);

		dp_reg_set_aux_ch_command(I2C_WRITE);
		dp_reg_set_aux_ch_address(DDC_SEGMENT_ADDR);
		dp_reg_set_aux_ch_length(1);
		dp_reg_aux_ch_send_buf(&segment, 1);
		ret = dp_reg_set_aux_ch_operation_enable();
		if (ret)
			cal_log_err(0, "sending segment failed\n");

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
			dp_reg_set_aux_ch_command(I2C_READ_NO_MOT);
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
	int ret;

	cal_log_debug(0, "DP Link Version = 0x%X\n", dp_reg_get_version());

	/* Apply Soft Reset */
	dp_reg_set_swreset();
	dp_reg_set_oscclk_qch_func_en(1);
	cal_log_debug(0, "reset DP Link\n");

	/* Set system clock to OSC */
	dp_reg_set_osc_clk_div(OSC_CLK);
	dp_reg_set_txclk_osc();
	cal_log_debug(0, "set system clk to OSC with %d MHz: Mux(%u)\n",
		      OSC_CLK, dp_reg_get_gfmux_status());

	/*
	 * USBDP PHY Initialization
	 */
	dwc3_exynos_phy_enable(1, 1);
	atomic_inc(&hw_config->usbdp_phy_en_cnt);

	ret = dpphy_reg_init(hw_config, false);
	if (ret)
		return ret;
	cal_log_debug(0, "init USBDP Combo PHY\n");

	/* Wait for PHY PLL Lock */
	ret = dp_reg_wait_phy_pll_lock();
	if (ret)
		return ret;
	cal_log_debug(0, "locked PHY PLL\n");

	/* Set system clock to TXCLK */
	dp_reg_set_txclk_txclk();
	cal_log_debug(0, "set system clk to TX: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

	/* Set Data Path */
	dp_hw_set_data_path(hw_config);
	cal_log_debug(0, "post-init USBDP Combo PHY\n");

	/*
	 * DP Link Initialization
	 */
	/* Set Lane Map Configuration */
	dp_reg_set_lane_count(hw_config->num_lanes);
	dp_reg_set_aux_pn(hw_config->orient_type, hw_config->aux_auto_orientation);
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
	int ret;

	/* Set system clock to OSC */
	dp_reg_set_txclk_osc();
	cal_log_debug(0, "set system clk to OSC: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

	/* USBDP PHY Re-initialization */
	ret = dpphy_reg_init(hw_config, true);
	if (ret)
		return ret;
	cal_log_debug(0, "reconfig USBDP Combo PHY\n");

	/* Wait for PHY PLL Lock */
	ret = dp_reg_wait_phy_pll_lock();
	if (ret)
		return ret;
	cal_log_debug(0, "locked PHY PLL\n");

	/* Set system clock to TXCLK */
	dp_reg_set_txclk_txclk();
	cal_log_debug(0, "set system clk to TX: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

	/* Set Data Path */
	dp_hw_set_data_path(hw_config);
	cal_log_debug(0, "post-init USBDP Combo PHY\n");

	/* Set Lane Map Configuration */
	dp_reg_set_lane_count(hw_config->num_lanes);
	dp_reg_set_aux_pn(hw_config->orient_type, hw_config->aux_auto_orientation);
	dp_hw_set_lane_map_config(hw_config);
	cal_log_debug(0, "set lane count & map\n");

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
	dp_reg_set_oscclk_qch_func_en(0);

	/* USBDP PHY De-initialization */
	dpphy_reg_reset_tx_lanes();
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
	dp_reg_set_video_en(0);
	dp_hw_set_video_interrupt(0);
	dp_reg_set_sst1_longhop_power(0);
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

void dp_hw_set_fec(bool en)
{
	dp_reg_set_fec_func_en(en ? 1 : 0);
	dp_reg_set_fec_ready(en ? 1 : 0);
}

void dp_hw_set_training_pattern(dp_training_pattern pattern)
{
	dp_reg_set_quality_pattern(DISABLE_PATTERN);
	dp_reg_set_training_pattern(pattern);
	dp_reg_set_bit_swap(1);

	if (pattern == NORMAL_DATA || pattern == TRAINING_PATTERN_4)
		dp_reg_set_scramble_bypass(ENABLE_SCRAM);
	else
		dp_reg_set_scramble_bypass(DISABLE_SCRAM);
}

void dp_hw_set_quality_pattern(dp_qual_pattern pattern, dp_scrambling scramble)
{
	dp_reg_set_training_pattern(NORMAL_DATA);
	if (pattern == CUSTOM_80BIT)
		dp_reg_set_custom_pattern();
	else if (pattern == HBR2_COMPLIANCE)
		dp_reg_set_hbr2_scrambler_reset(252*2);
	dp_reg_set_quality_pattern(pattern);
	dp_reg_set_bit_swap(1);
	dp_reg_set_scramble_bypass(scramble);
}

void dp_hw_set_voltage_and_pre_emphasis(struct dp_hw_config *hw_config,
					u8 *voltage, u8 *pre_emphasis)
{
	enum dp_link_rate_type link_rate = hw_config->link_rate;

	switch (hw_config->pin_type) {
	case PIN_TYPE_A:
	case PIN_TYPE_C:
	case PIN_TYPE_E:
		dpphy_reg_set_lane_tx_level(link_rate, 0, voltage[0],
					    pre_emphasis[0]);
		dpphy_reg_set_lane_tx_level(link_rate, 1, voltage[1],
					    pre_emphasis[1]);
		dpphy_reg_set_lane_tx_level(link_rate, 2, voltage[2],
					    pre_emphasis[2]);
		dpphy_reg_set_lane_tx_level(link_rate, 3, voltage[3],
					    pre_emphasis[3]);
		break;

	case PIN_TYPE_B:
	case PIN_TYPE_D:
	case PIN_TYPE_F:
		dpphy_reg_set_lane_tx_level(link_rate, 0, voltage[0],
					    pre_emphasis[0]);
		dpphy_reg_set_lane_tx_level(link_rate, 1, voltage[1],
					    pre_emphasis[1]);
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

	cal_log_debug(0, "%s: dp_audio_init_config\n", __func__);
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

	cal_log_debug(0, "%s: dp_audio is started\n", __func__);
}

void dp_hw_stop_audio(void)
{
	if (dp_reg_is_audio_enabled()) {
		udelay(1000);
		dp_reg_set_audio_master_timing_gen(0);
		dp_reg_set_audio_en(0);
		dp_reg_set_clear_audio_fifo();
		cal_log_debug(0, "%s: dp_audio is disabled\n", __func__);
	} else
		cal_log_debug(
			0, "%s: dp_audio is already disabled, AUDIO_EN = 0\n",
			__func__);
}

void dp_hw_set_audio_config(struct dp_hw_config *hw_config)
{
	enum audio_mode aud_mode = AUDIO_MODE_ASYNC;

	dp_reg_set_audio_mode(aud_mode);
	dp_reg_set_audio_clock(hw_config, aud_mode);

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

	cal_log_debug(0, "%s: dp_audio is configured for BIST mode\n",
		      __func__);
	return 0;
}

void dp_hw_set_audio_dma(u32 en)
{
	dp_reg_set_audio_dma_req_gen(en);
}

// DP HDCP Configuration Interfaces
bool dp_hw_get_hdcp13_key_valid(void)
{
	return dp_reg_get_hdcp13_key_valid() ? true : false;
}

void dp_hw_set_hdcp13_repeater(u8 is_repeater)
{
	dp_reg_set_hdcp13_repeater(is_repeater);
}

void dp_hw_set_hdcp13_bksv(u8 *bksv)
{
	u32 bksv0 = 0, bksv1 = 0;
	int i;

	for (i = 0; i < 4; i++)
		bksv0 |= (bksv[i] << (i * 8));
	bksv1 |= bksv[4];

	dp_reg_set_hdcp13_bksv(bksv0, bksv1);
}

void dp_hw_get_hdcp13_an(u8 *an)
{
	u32 an0 = 0, an1 = 0;
	int i;

	dp_reg_set_hdcp13_store_an();
	dp_reg_get_hdcp13_an(&an0, &an1);

	for (i = 0; i < 4; i++)	// Byte 0, 1, 2, 3
		an[i] = (u8)((an0 >> (i * 8)) & 0xFF);
	for (; i < 8; i++)	// Byte 4, 5, 6, 7
		an[i] = (u8)((an1 >> ((i - 4) * 8)) & 0xFF);
}

bool dp_hw_get_hdcp13_aksv(u8 *aksv)
{
	u32 aksv0 = 0, aksv1 = 0;
	int i;

	dp_reg_get_hdcp13_aksv(&aksv0, &aksv1);

	for (i = 0; i < 4; i++)
		aksv[i] = (u8)((aksv0 >> (i * 8)) & 0xFF);
	aksv[i] = (u8)(aksv1 & 0xFF);

	return dp_reg_get_hdcp13_aksv_valid();
}

void dp_hw_get_hdcp13_r0(u32 *r0)
{
	dp_reg_get_hdcp13_r0(r0);
}

void dp_hw_get_hdcp13_am0(u8 *am)
{
	u32 am0 = 0, am1 = 0;
	int i;

	dp_reg_get_hdcp13_am0(&am0, &am1);

	for (i = 0; i < 4; i++)	// Byte 0, 1, 2, 3
		am[i] = (u8)((am0 >> (i * 8)) & 0xFF);
	for (; i < 8; i++)	// Byte 4, 5, 6, 7
		am[i] = (u8)((am1 >> ((i - 4) * 8)) & 0xFF);
}

void dp_hw_set_hdcp13_function(u32 en)
{
	dp_reg_set_hpd_force();
	dp_reg_set_hdcp13_func_en(en);
}

void dp_hw_set_hdcp13_encryption(u32 en)
{
	dp_reg_set_hdcp13_encryption_enable(en);
}

void dp_hw_set_hdcp22_function(u32 en)
{
	dp_reg_set_hdcp22_system_enable(en);
	dp_reg_set_hdcp22_func_en(en);
}

void dp_hw_set_hdcp22_encryption(u32 en)
{
	dp_reg_set_hdcp22_encryption_enable(en);
}

static int dp_smc_read(u32 dp_id, u32 offset, u32 *read_result)
{
	struct cal_regs_desc *rdesc = dp_regs_desc(dp_id);
	unsigned long smc_result = 0;
	unsigned long smc_read_result = 0;

	if (read_result == NULL)
		return -EINVAL;
	smc_result = exynos_smc4(SMC_DRM_DPU_CRC_SEC, rdesc->start + offset, 0, 0, &smc_read_result,
				 0, 0);
	if (smc_result != DRMDRV_OK) {
		pr_debug("%s: exynos_smc_readvalue failed (smc_result=%lu)\n", __func__,
			 smc_result);
		return -EIO;
	}
	*read_result = (u32)smc_read_result;
	return 0;
}

static int dp_smc_read_mask(u32 dp_id, u32 offset, u32 mask, u32 *read_result)
{
	int res = dp_smc_read(dp_id, offset, read_result);

	if (res != 0)
		return res;
	*read_result &= mask;
	return 0;
}

static int dp_smc_write(u32 dp_id, u32 offset, u32 val)
{
	struct cal_regs_desc *rdesc = dp_regs_desc(dp_id);
	unsigned long smc_result = 0;

	smc_result = exynos_smc(SMC_DRM_DPU_CRC_SEC, rdesc->start + offset, 1, val);
	if (smc_result != DRMDRV_OK) {
		pr_debug("%s: exynos_smc failed (smc_result=%lu)\n", __func__, smc_result);
		return -EIO;
	}
	return 0;
}

static int dp_smc_write_mask(u32 dp_id, u32 offset, u32 val, u32 mask)
{
	u32 read_result = 0;
	int smc_res = dp_smc_read(dp_id, offset, &read_result);

	if (smc_res != 0)
		return smc_res;

	val = (val & mask) | (read_result & ~mask);
	return dp_smc_write(dp_id, offset, val);
}

int dp_crc_set_enabled(u32 id, u32 en)
{
	int res = 0;
	u32 wval = 0;

	wval = en ? (IF_CRC_SW_COMPARE | IF_CRC_EN) : 0;
	res = dp_smc_write_mask(id, SST1_STREAM_IF_CRC_CONTROL_1, wval,
				IF_CRC_SW_COMPARE | IF_CRC_EN);
	if (res != 0) {
		pr_debug("%s: dp_smc_write_mask failed: %d\n", __func__, res);
		return res;
	}
	wval = en ? SEC_CRC_ENABLE : 0;
	res = dp_smc_write_mask(id, SST1_SEC_CRC_CONTROL_4, wval, SEC_CRC_ENABLE);
	if (res != 0) {
		pr_debug("%s: dp_smc_write_mask failed: %d\n", __func__, res);
		return res;
	}
	return res;
}

int dp_crc_get(u32 id, u32 crc_data[3])
{
	int res = 0;

	res = dp_smc_read_mask(id, SST1_SEC_CRC_CONTROL_1, SEC_CRC_RGB_RESULT, &crc_data[0]);
	if (res != 0) {
		pr_debug("%s: dp_smc_read_mask failed: %d\n", __func__, res);
		return res;
	}
	res = dp_smc_read_mask(id, SST1_SEC_CRC_CONTROL_2, SEC_CRC_RGB_RESULT, &crc_data[1]);
	if (res != 0) {
		pr_debug("%s: dp_smc_read_mask failed: %d\n", __func__, res);
		return res;
	}
	res = dp_smc_read_mask(id, SST1_SEC_CRC_CONTROL_3, SEC_CRC_RGB_RESULT, &crc_data[2]);
	if (res != 0) {
		pr_debug("%s: dp_smc_read_mask failed: %d\n", __func__, res);
		return res;
	}

	return res;
}

int dp_crc_reset(u32 id)
{
	int res = 0;
	u32 wval = 0;

	wval = IF_CRC_CLEAR;
	res = dp_smc_write_mask(id, SST1_STREAM_IF_CRC_CONTROL_1, wval, wval);
	if (res != 0) {
		pr_debug("%s: dp_smc_write_mask failed: %d\n", __func__, res);
		return res;
	}
	wval = CLEAR_SEC_CRC_LFSR | SEC_CRC_CLEAR;
	res = dp_smc_write_mask(id, SST1_SEC_CRC_CONTROL_4, wval, wval);
	if (res != 0) {
		pr_debug("%s: dp_smc_write_mask failed: %d\n", __func__, res);
		return res;
	}
	return res;
}
