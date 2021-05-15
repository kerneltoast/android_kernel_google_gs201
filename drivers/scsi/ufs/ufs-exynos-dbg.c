// SPDX-License-Identifier: GPL-2.0-only
//
// UFS Host Controller driver for Exynos specific extensions
//
// Copyright (C) 2016 Samsung Electronics Co., Ltd.
//
// Authors:
//	Kiwoong <kwmad.kim@samsung.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.

#include <linux/smc.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/module.h>

#include "ufshcd.h"
#include "ufs-vs-mmio.h"
#include "ufs-vs-regs.h"

enum {
	LOG_STD_HCI_SFR = 0xFFFFFFF0,
	LOG_VS_HCI_SFR,
	LOG_FMP_SFR,
	LOG_UNIPRO_SFR,
	LOG_PMA_SFR,
	LOG_INVALID,
};

enum {
	SFR_VAL_H_0_FIRST = 0,
	SFR_VAL_H_0,
	SFR_VAL_NUM,
};

enum {
	ATTR_VAL_H_0_L_0_FIRST = 0,
	ATTR_VAL_H_0_L_1_FIRST,
	ATTR_VAL_H_0_L_0,
	ATTR_VAL_H_0_L_1,
	ATTR_VAL_NUM,
};

enum {
	DBG_ATTR_UNIPRO = 0xEFFFFFF0,
	DBG_ATTR_PCS_CMN,
	DBG_ATTR_PCS_TX,
	DBG_ATTR_PCS_RX,
	DBG_ATTR_INVALID,
};

struct exynos_ufs_sfr_log {
	const char *name;
	const u32 offset;
	u32 val[SFR_VAL_NUM];
};

struct exynos_ufs_attr_log {
	const u32 mib;
	const u32 offset;
	/* 0: lane0, 1: lane1 */
	u32 val[ATTR_VAL_NUM];
};

/* Structure for ufs cmd logging */
#define MAX_CMD_LOGS    32

struct cmd_data {
	unsigned int tag;
	unsigned int sct;
	unsigned long lba;
	u64 start_time;
	u64 end_time;
	u64 outstanding_reqs;
	int retries;
	unsigned char op;
};

struct ufs_cmd_info {
	u32 total;
	u32 last;
	struct cmd_data data[MAX_CMD_LOGS];
	struct cmd_data *pdata[32];	/* Currently, 32 slots */
};

static const char *ufs_sfr_field_name[3] = {
	"NAME",	"OFFSET", "VALUE",
};

static const char *ufs_attr_field_name[4] = {
	"MIB",	"OFFSET(SFR)", "VALUE(lane #0)", "VALUE(lane #1)",
};

#define ATTR_LANE_OFFSET		16
#define ATTR_TYPE_MASK(addr)		(((addr) >> ATTR_LANE_OFFSET) & 0xF)
#define ATTR_SET(addr, type)		((addr) |	\
		((type & 0xF) << ATTR_LANE_OFFSET))

#define ATTR_NUM_MAX_LANES		2

/* CPORT printing enable: 1 , disable:0 */
#define UFS_CPORT_PRINT         1

#define CPORT_TX_BUF_SIZE       0x100
#define CPORT_RX_BUF_SIZE       0x100
#define CPORT_LOG_PTR_SIZE      0x100
#define CPORT_UTRL_SIZE         0x400
#define CPORT_UCD_SIZE          0x400
#define CPORT_UTMRL_SIZE        0xc0
#define CPORT_BUF_SIZE          (CPORT_TX_BUF_SIZE + CPORT_RX_BUF_SIZE + \
		CPORT_LOG_PTR_SIZE +  CPORT_UTRL_SIZE + \
		CPORT_UCD_SIZE +  CPORT_UTMRL_SIZE)

#define CPORT_TX_BUF_PTR        0x0
#define CPORT_RX_BUF_PTR        (CPORT_TX_BUF_PTR + CPORT_TX_BUF_SIZE)
#define CPORT_LOG_PTR_OFFSET    (CPORT_RX_BUF_PTR + CPORT_RX_BUF_SIZE)
#define CPORT_UTRL_PTR          (CPORT_LOG_PTR_OFFSET + CPORT_LOG_PTR_SIZE)
#define CPORT_UCD_PTR           (CPORT_UTRL_PTR + CPORT_UTRL_SIZE)
#define CPORT_UTMRL_PTR         (CPORT_UCD_PTR + CPORT_UCD_SIZE)

enum {
	TX_LANE_0 = 0,
	TX_LANE_1 = 1,
	TX_LANE_2 = 2,
	TX_LANE_3 = 3,
	RX_LANE_0 = 4,
	RX_LANE_1 = 5,
	RX_LANE_2 = 6,
	RX_LANE_3 = 7,
};

struct ufs_log_cport {
	u32 ptr;
	u8 buf[CPORT_BUF_SIZE];
} ufs_log_cport;

#define DBG_NUM_OF_HOSTS	1
struct ufs_dbg_mgr {
	struct ufs_vs_handle *handle;
	int active;
	u64 first_time;
	u64 time;
	u32 lanes;

	/* cport */
	struct ufs_log_cport log_cport;

	/* cmd log */
	struct ufs_cmd_info cmd_info;
	struct cmd_data cmd_log;		/* temp buffer to put */
	spinlock_t cmd_lock;
};

static struct ufs_dbg_mgr ufs_dbg[DBG_NUM_OF_HOSTS];
static int ufs_dbg_mgr_idx;

/* hardware, pma */
#define PHY_PMA_COMN_ADDR(reg)			(reg)
#define PHY_PMA_TRSV_ADDR(reg, lane)		((reg) + (0x800 * (lane)))

/* hardware, pcs */
#define UNIP_COMP_AXI_AUX_FIELD			0x040
#define __WSTRB					(0xF << 24)
#define __SEL_IDX(L)				((L) & 0xFFFF)

static struct exynos_ufs_sfr_log ufs_log_sfr[] = {
	{"STD HCI SFR",	LOG_STD_HCI_SFR},

	{"INTERRUPT STATUS",	0x20},
	{"INTERRUPT ENABLE",	0x24},
	{"CONTROLLER STATUS",	0x30},
	{"CONTROLLER ENABLE",	0x34},
	{"UECPA",		0x38},
	{"UECDL",		0x3c},
	{"UECN",		0x40},
	{"UECT",		0x44},
	{"UECDME",		0x48},
	{"UTP TRANSF REQ INT AGG CNTRL",	0x4C},
	{"UTP TRANSF REQ LIST BASE L",	0x50},
	{"UTP TRANSF REQ LIST BASE H",	0x54},
	{"UTP TRANSF REQ DOOR BELL",	0x58},
	{"UTP TRANSF REQ LIST CLEAR",	0x5C},
	{"UTP TRANSF REQ LIST RUN STOP",	0x60},
	{"UTP TRANSF REQ LIST CNR",	0x64},
	{"UTP TASK REQ LIST BASE L",	0x70},
	{"UTP TASK REQ LIST BASE H",	0x74},
	{"UTP TASK REQ DOOR BELL",	0x78},
	{"UTP TASK REQ LIST CLEAR",	0x7C},
	{"UTP TASK REQ LIST RUN STOP",	0x80},
	{"UIC COMMAND",	0x90},
	{"UIC COMMAND ARG1",	0x94},
	{"UIC COMMAND ARG2",	0x98},
	{"UIC COMMAND ARG3",	0x9C},
	{"MH_IS",    0x00d0},
	{"MH_IE",    0x00d4},
	{"DBR_DUPLICATION_INFO",    0x00e8},
	{"SMU_CDB_INVALID_INFO",    0x00ec},
	{"SMU_UNMAP_INVALID_INFO",    0x00f4},
	{"CCAP",    0x0100},

	{"VS HCI SFR",	LOG_VS_HCI_SFR},

	{"TXPRDT ENTRY SIZE",	HCI_TXPRDT_ENTRY_SIZE},
	{"RXPRDT ENTRY SIZE",	HCI_RXPRDT_ENTRY_SIZE},
	{"TO CNT DIV VAL",	HCI_TO_CNT_DIV_VAL},
	{"1US TO CNT VAL",	HCI_1US_TO_CNT_VAL},
	{"INVALID UPIU CTRL",	HCI_INVALID_UPIU_CTRL},
	{"INVALID UPIU BADDR",	HCI_INVALID_UPIU_BADDR},
	{"INVALID UPIU UBADDR",	HCI_INVALID_UPIU_UBADDR},
	{"INVALID UTMR OFFSET ADDR",	HCI_INVALID_UTMR_OFFSET_ADDR},
	{"INVALID UTR OFFSET ADDR",	HCI_INVALID_UTR_OFFSET_ADDR},
	{"INVALID DIN OFFSET ADDR",	HCI_INVALID_DIN_OFFSET_ADDR},
	{"VENDOR SPECIFIC IS",	HCI_VENDOR_SPECIFIC_IS},
	{"VENDOR SPECIFIC IE",	HCI_VENDOR_SPECIFIC_IE},
	{"UTRL NEXUS TYPE",	HCI_UTRL_NEXUS_TYPE},
	{"UTMRL NEXUS TYPE",	HCI_UTMRL_NEXUS_TYPE},
	{"SW RST",	HCI_SW_RST},
	{"RX UPIU MATCH ERROR CODE",	HCI_RX_UPIU_MATCH_ERROR_CODE},
	{"DATA REORDER",	HCI_DATA_REORDER},
	{"AXIDMA RWDATA BURST LEN",	HCI_AXIDMA_RWDATA_BURST_LEN},
	{"WRITE DMA CTRL",	HCI_WRITE_DMA_CTRL},
	{"V2P1 CTRL",	HCI_UFSHCI_V2P1_CTRL},
	{"CLKSTOP CTRL",	HCI_CLKSTOP_CTRL},
	{"FORCE HCS",	HCI_FORCE_HCS},
	{"DMA0 MONITOR STATE",	HCI_DMA0_MONITOR_STATE},
	{"DMA0 MONITOR CNT",	HCI_DMA0_MONITOR_CNT},
	{"DMA1 MONITOR STATE",	HCI_DMA1_MONITOR_STATE},
	{"DMA1 MONITOR CNT",	HCI_DMA1_MONITOR_CNT},
	{"DMA0 DOORBELL DEBUG",	HCI_DMA0_DOORBELL_DEBUG},
	{"DMA1 DOORBELL DEBUG",	HCI_DMA1_DOORBELL_DEBUG},

	{"AXI DMA IF CTRL",	HCI_UFS_AXI_DMA_IF_CTRL},
	{"UFS ACG DISABLE",	HCI_UFS_ACG_DISABLE},
	{"MPHY REFCLK SEL",	HCI_MPHY_REFCLK_SEL},

	{"SMU RD ABORT MATCH INFO",	HCI_SMU_RD_ABORT_MATCH_INFO},
	{"SMU WR ABORT MATCH INFO",	HCI_SMU_WR_ABORT_MATCH_INFO},

	{"DBR DUPLICATION INFO",	HCI_DBR_DUPLICATION_INFO},
	{"INVALID PRDT CTRL",	HCI_INVALID_PRDT_CTRL},

	{"FMP SFR",	LOG_FMP_SFR},

	{"UFSPRCTRL",	UFSPRCTRL},
	{"UFSPRSTAT",	UFSPRSTAT},
	{"UFSPRSECURITY",	UFSPRSECURITY},
	{"UFSPWCTRL",	UFSPWCTRL},
	{"UFSPWSTAT",	UFSPWSTAT},
	{"UFSPSBEGIN0",	UFSPSBEGIN0},
	{"UFSPSEND0",	UFSPSEND0},
	{"UFSPSLUN0",	UFSPSLUN0},
	{"UFSPSCTRL0",	UFSPSCTRL0},
	{"UFSPSBEGIN1",	UFSPSBEGIN1},
	{"UFSPSEND1",	UFSPSEND1},
	{"UFSPSLUN1",	UFSPSLUN1},
	{"UFSPSCTRL1",	UFSPSCTRL1},
	{"UFSPSBEGIN2",	UFSPSBEGIN2},
	{"UFSPSEND2",	UFSPSEND2},
	{"UFSPSLUN2",	UFSPSLUN2},
	{"UFSPSCTRL2",	UFSPSCTRL2},
	{"UFSPSBEGIN3",	UFSPSBEGIN3},
	{"UFSPSEND3",	UFSPSEND3},
	{"UFSPSLUN3",	UFSPSLUN3},
	{"UFSPSCTRL3",	UFSPSCTRL3},
	{"UFSPSBEGIN4",	UFSPSBEGIN4},
	{"UFSPSLUN4",	UFSPSLUN4},
	{"UFSPSCTRL4",	UFSPSCTRL4},
	{"UFSPSBEGIN5",	UFSPSBEGIN5},
	{"UFSPSEND5",	UFSPSEND5},
	{"UFSPSLUN5",	UFSPSLUN5},
	{"UFSPSCTRL5",	UFSPSCTRL5},
	{"UFSPSBEGIN6",	UFSPSBEGIN6},
	{"UFSPSEND6",	UFSPSEND6},
	{"UFSPSLUN6",	UFSPSLUN6},
	{"UFSPSCTRL6",	UFSPSCTRL6},
	{"UFSPSBEGIN7",	UFSPSBEGIN7},
	{"UFSPSEND7",	UFSPSEND7},
	{"UFSPSLUN7",	UFSPSLUN7},
	{"UFSPSCTRL7",	UFSPSCTRL7},

	{"UNIPRO SFR",	LOG_UNIPRO_SFR},

	{"DME_LINKSTARTUP_CNF_RESULT",	UNIP_DME_LINKSTARTUP_CNF_RESULT},
	{"DME_HIBERN8_ENTER_CNF_RESULT",
		UNIP_DME_HIBERN8_ENTER_CNF_RESULT},
	{"DME_HIBERN8_ENTER_IND_RESULT",
		UNIP_DME_HIBERN8_ENTER_IND_RESULT},
	{"DME_HIBERN8_EXIT_CNF_RESULT",	UNIP_DME_HIBERN8_EXIT_CNF_RESULT},
	{"DME_HIBERN8_EXIT_IND_RESULT",	UNIP_DME_HIBERN8_EXIT_IND_RESULT},
	{"DME_PWR_IND_RESULT",	UNIP_DME_PWR_IND_RESULT},
	{"DME_INTR_STATUS_LSB",	UNIP_DME_INTR_STATUS_LSB},
	{"DME_INTR_STATUS_MSB",	UNIP_DME_INTR_STATUS_MSB},
	{"DME_INTR_ERROR_CODE",	UNIP_DME_INTR_ERROR_CODE},
	{"DME_DISCARD_PORT_ID",	UNIP_DME_DISCARD_PORT_ID},
	{"DME_DBG_OPTION_SUITE",	UNIP_DME_DBG_OPTION_SUITE},
	{"DME_DBG_CTRL_FSM",	UNIP_DME_DBG_CTRL_FSM},
	{"DME_DBG_FLAG_STATUS",	UNIP_DME_DBG_FLAG_STATUS},
	{"DME_DBG_LINKCFG_FSM",	UNIP_DME_DBG_LINKCFG_FSM},

	{"PMA SFR",	LOG_PMA_SFR},

	{"CMN_REG4F", 0x013c},
	{"CMN_REG50", 0x0140},
	{"CMN_REG51", 0x0144},
	{"TRSV_REG30B", 0x0c2c},
	{"TRSV_REG30D", 0x0c34},
	{"TRSV_REG30E", 0x0c38},
	{"TRSV_REG30F", 0x0c3c},
	{"TRSV_REG310", 0x0c40},
	{"TRSV_REG311", 0x0c44},
	{"TRSV_REG313", 0x0c4c},
	{"TRSV_REG314", 0x0c50},
	{"TRSV_REG316", 0x0c58},
	{"TRSV_REG317", 0x0c5c},
	{"TRSV_REG319", 0x0c64},
	{"TRSV_REG323", 0x0c8c},
	{"TRSV_REG337", 0x0cdc},
	{"TRSV_REG338", 0x0ce0},
	{"TRSV_REG339", 0x0ce4},
	{"TRSV_REG50B", 0x142C},
	{"TRSV_REG50D", 0x1434},
	{"TRSV_REG50E", 0x1438},
	{"TRSV_REG50F", 0x143C},
	{"TRSV_REG510", 0x1440},
	{"TRSV_REG511", 0x1444},
	{"TRSV_REG513", 0x144C},
	{"TRSV_REG514", 0x1450},
	{"TRSV_REG516", 0x1458},
	{"TRSV_REG517", 0x145C},
	{"TRSV_REG519", 0x1464},
	{"TRSV_REG523", 0x148C},
	{"TRSV_REG537", 0x14DC},
	{"TRSV_REG538", 0x14E0},
	{"TRSV_REG539", 0x14E4},

	{0, 0},
};

static struct exynos_ufs_attr_log ufs_log_attr[] = {
	{DBG_ATTR_UNIPRO, 0},
	/* DME */
	{0xD015, 0x7854},
	{0xD019, 0x7864},
	{0xD01A, 0x7868},
	{0xD01D, 0x7874},
	{0xD01E, 0x7878},
	{0xD03B, 0x78EC},
	{0xD0C0, 0x7B00},
	{0xD0C1, 0x7B04},
	{0xD0C8, 0x7B20},
	{0xD100, 0x7C00},
	{0xD140, 0x7D00},
	{0xD145, 0x7D14},
	{0xD146, 0x7D18},
	{0xD184, 0x7E10},
	{0xD185, 0x7E14},
	{0xD188, 0x7E20},
	{0xD189, 0x7E24},
	{0xD18A, 0x7E28},
	{0xD18B, 0x7E2C},
	{0xD18C, 0x7E30},
	{0xD18D, 0x7E34},
	{0xD18E, 0x7E38},
	{0xD18F, 0x7E3C},

	/* TL */
	{0x4020, 0x6080},
	{0xC001, 0x6804},
	{0xC024, 0x6890},
	{0xC026, 0x6898},

	/* NL */
	{0xB011, 0x5844},

	/* DL */
	{0x2047, 0x411C},
	{0x2067, 0x419C},
	{0xA000, 0x4800},
	{0xA005, 0x4814},
	{0xA007, 0x481C},
	{0xA010, 0x4840},
	{0xA020, 0x4880},
	{0xA021, 0x4884},
	{0xA022, 0x4888},
	{0xA023, 0x488C},
	{0xA024, 0x4890},
	{0xA025, 0x4894},
	{0xA026, 0x4898},
	{0xA027, 0x489C},
	{0xA100, 0x4C00},
	{0xA101, 0x4C04},
	{0xA102, 0x4C08},
	{0xA103, 0x4C0C},
	{0xA114, 0x4C50},
	{0xA115, 0x4C54},
	{0xA116, 0x4C58},
	{0xA120, 0x4C80},
	{0xA121, 0x4C84},
	{0xA122, 0x4C88},

	/* PA */
	{0x1520, 0x3080},
	{0x1540, 0x3100},
	{0x1543, 0x310C},
	{0x155C, 0x3170},
	{0x155D, 0x3174},
	{0x155F, 0x317C},
	{0x1560, 0x3180},
	{0x1561, 0x3184},
	{0x1564, 0x3190},
	{0x1567, 0x319C},
	{0x1568, 0x31A0},
	{0x1569, 0x31A4},
	{0x156A, 0x31A8},
	{0x1571, 0x31C4},
	{0x1580, 0x3200},
	{0x1581, 0x3204},
	{0x1582, 0x3208},
	{0x1583, 0x320C},
	{0x1584, 0x3210},
	{0x1585, 0x3214},
	{0x1590, 0x3240},
	{0x1591, 0x3244},
	{0x15A1, 0x3284},
	{0x15A2, 0x3288},
	{0x15A3, 0x328C},
	{0x15A4, 0x3290},
	{0x15A7, 0x329C},
	{0x15A8, 0x32A0},
	{0x15A9, 0x32A4},
	{0x15C0, 0x3300},
	{0x15C1, 0x3304},
	{0x15D2, 0x3348},
	{0x15D3, 0x334C},
	{0x15D4, 0x3350},
	{0x15D5, 0x3354},
	{0x9500, 0x3800},
	{0x9501, 0x3804},
	{0x9502, 0x3808},
	{0x9504, 0x3810},
	{0x9564, 0x3990},
	{0x956A, 0x39A8},
	{0x956D, 0x39B4},
	{0x9570, 0x39C0},
	{0x9595, 0x3A54},
	{0x9596, 0x3A58},
	{0x9597, 0x3A5C},
	{0x95C0, 0x3B00},
	{0x95C1, 0x3B04},
	{0x0044, 0x0044},

	/* MPHY PCS TX */
	{DBG_ATTR_PCS_TX, 0},
	{0x21, 0x2084},
	{0x22, 0x2088},
	{0x23, 0x208C},
	{0x24, 0x2090},
	{0x28, 0x20A0},
	{0x29, 0x20A4},
	{0x2A, 0x20A8},
	{0x2B, 0x20AC},
	{0x2C, 0x20B0},
	{0x2D, 0x20B4},
	{0x33, 0x20CC},
	{0x35, 0x20D4},
	{0x36, 0x20D8},
	{0x41, 0x2104},

	{DBG_ATTR_PCS_RX, 0},
	{0xA1, 0x2284},
	{0xA2, 0x2288},
	{0xA3, 0x228C},
	{0xA4, 0x2290},
	{0xA7, 0x229C},
	{0xC1, 0x2304},
	{0x06, 0x2018},
	{0x12, 0x2048},
	{0x19, 0x2064},
	{0x23, 0x208C},
	{0x24, 0x2090},
	{0x5D, 0x2174},

	{0, 0},
};

static void __ufs_get_sfr(struct ufs_dbg_mgr *mgr,
			  struct exynos_ufs_sfr_log *cfg)
{
	struct ufs_vs_handle *handle = mgr->handle;
	int sel_api = 0;
	u32 reg;
	u32 *pval;

	while (cfg) {
		if (!cfg->name)
			break;

		if (cfg->offset >= LOG_STD_HCI_SFR) {
			/* Select an API to get SFRs */
			sel_api = cfg->offset;
			if (sel_api == LOG_PMA_SFR) {
				/* Enable MPHY APB */
				reg = hci_readl(handle, HCI_CLKSTOP_CTRL);
				hci_writel(handle, reg & ~MPHY_APBCLK_STOP,
					   HCI_CLKSTOP_CTRL);
			}
			cfg++;
			continue;
		}

		/* Fetch value */
		pval = &cfg->val[SFR_VAL_H_0];
		if (sel_api == LOG_STD_HCI_SFR)
			*pval = std_readl(handle, cfg->offset);
		else if (sel_api == LOG_VS_HCI_SFR)
			*pval = hci_readl(handle, cfg->offset);
#ifdef CONFIG_EXYNOS_SMC_LOGGING
		else if (sel_api == LOG_FMP_SFR)
			*pval = exynos_smc(SMC_CMD_FMP_SMU_DUMP,
					   0, 0, cfg->offset);
#endif
		else if (sel_api == LOG_UNIPRO_SFR)
			*pval = unipro_readl(handle, cfg->offset);
		else if (sel_api == LOG_PMA_SFR)
			*pval = pma_readl(handle, cfg->offset);
		else
			*pval = 0xFFFFFFFF;

		/* Keep the first contexts permanently */
		if (mgr->first_time == 0ULL)
			cfg->val[SFR_VAL_H_0_FIRST] = *pval;

		/* Next SFR */
		cfg++;
	}

	/* Disable MPHY APB */
	hci_writel(handle, reg | MPHY_APBCLK_STOP, HCI_CLKSTOP_CTRL);
}

static inline u32 __ufs_set_pcs_read(struct ufs_vs_handle *handle, u32 lane,
				     struct exynos_ufs_attr_log *cfg)
{
	unipro_writel(handle, __WSTRB | __SEL_IDX(lane),
		      UNIP_COMP_AXI_AUX_FIELD);
	return unipro_readl(handle, cfg->offset);
}

static void __ufs_get_attr(struct ufs_dbg_mgr *mgr,
			   struct exynos_ufs_attr_log *cfg)
{
	struct ufs_vs_handle *handle = mgr->handle;
	int sel_api = 0;
	u32 val;
	u32 *pval;
	int i;

	while (cfg) {
		if (cfg->mib == 0)
			break;

		/* Fetch result and value */
		pval = &cfg->val[ATTR_VAL_H_0_L_0];
		if (cfg->mib >= DBG_ATTR_UNIPRO) {
			/* Select an API to get attributes */
			sel_api = cfg->mib;
			cfg++;
			continue;
		}

		if (sel_api == DBG_ATTR_UNIPRO) {
			*pval = unipro_readl(handle, cfg->offset);
		} else if (sel_api == DBG_ATTR_PCS_CMN) {
			*pval = unipro_readl(handle, cfg->offset);
		} else if (sel_api == DBG_ATTR_PCS_TX) {
			for (i = 0 ; i < ATTR_NUM_MAX_LANES ; i++) {
				if (i >= mgr->lanes)
					val = 0xFFFFFFFF;
				else
					__ufs_set_pcs_read(handle,
							   TX_LANE_0 + i, cfg);
				*(pval + 1) = val;
			}
		} else if (sel_api == DBG_ATTR_PCS_RX) {
			for (i = 0 ; i < ATTR_NUM_MAX_LANES ; i++) {
				if (i >= mgr->lanes)
					val = 0xFFFFFFFF;
				else
					__ufs_set_pcs_read(handle,
							   RX_LANE_0 + i, cfg);
				*(pval + 1) = val;
			}
		} else {
			//TODO:
			;
		}

		/* Keep the first contexts permanently */
		if (mgr->first_time == 0ULL) {
			cfg->val[ATTR_VAL_H_0_L_0_FIRST] = *pval;
			cfg->val[ATTR_VAL_H_0_L_1_FIRST] = *(pval + 1);
		}

		/* Next attribute */
		cfg++;
	}
}

static void __ufs_print_sfr(struct ufs_vs_handle *handle,
			    struct device *dev,
			    struct exynos_ufs_sfr_log *cfg)
{
	dev_err(dev, ":---------------------------------------------------\n");
	dev_err(dev, ":\t\tREGISTER\n");
	dev_err(dev, ":---------------------------------------------------\n");
	dev_err(dev, ":%-30s\t%-012s\t%-014s\n", ufs_sfr_field_name[0],
		ufs_sfr_field_name[1], ufs_sfr_field_name[2]);

	while (cfg) {
		if (!cfg->name)
			break;

		/* show */
		if (cfg->offset >= LOG_STD_HCI_SFR)
			dev_err(dev, "\n");
		dev_err(dev, ":%-30s\t0x%-012x\t0x%-014x\n",
			cfg->name, cfg->offset, cfg->val[SFR_VAL_H_0]);
		if (cfg->offset >= LOG_STD_HCI_SFR)
			dev_err(dev, "\n");

		/* Next SFR */
		cfg++;
	}
}

static void __ufs_print_attr(struct ufs_vs_handle *handle,
			     struct device *dev,
			     struct exynos_ufs_attr_log *cfg)
{
	dev_err(dev, ":---------------------------------------------------\n");
	dev_err(dev, ":\t\tATTRIBUTE\n");
	dev_err(dev, ":---------------------------------------------------\n");
	dev_err(dev, ":%-30s\t%-12s%-14s\t%-14s\n",
		ufs_attr_field_name[0],
		ufs_attr_field_name[1],
		ufs_attr_field_name[2],
		ufs_attr_field_name[3]);

	while (cfg) {
		if (!cfg->mib)
			break;

		/* show */
		if (cfg->offset >= DBG_ATTR_UNIPRO)
			dev_err(dev, "\n");
		dev_err(dev, ":0x%-27x\t0x%-012x\t0x%-014x\t0x%-014x\n",
			cfg->mib, cfg->offset,
			cfg->val[ATTR_VAL_H_0_L_0],
			cfg->val[ATTR_VAL_H_0_L_1]);
		if (cfg->offset >= DBG_ATTR_UNIPRO)
			dev_err(dev, "\n");

		/* Next SFR */
		cfg++;
	}
}

static void __print_cport(struct device *dev, int tag_printed,
			  u32 tag, u32 *ptr)
{
	if (tag_printed)
		dev_err(dev, "%08x %08x %08x %08x tag# %u\n",
			be32_to_cpu(*(ptr + 0)), be32_to_cpu(*(ptr + 1)),
			be32_to_cpu(*(ptr + 2)), be32_to_cpu(*(ptr + 3)),
			tag);
	else
		dev_err(dev, "%08x %08x %08x %08x\n",
			be32_to_cpu(*(ptr + 0)), be32_to_cpu(*(ptr + 1)),
			be32_to_cpu(*(ptr + 2)), be32_to_cpu(*(ptr + 3)));
}

static void __ufs_print_cport(struct ufs_dbg_mgr *mgr, struct device *dev)
{
	struct ufs_vs_handle *handle = mgr->handle;
	struct ufs_log_cport *log_cport = &mgr->log_cport;
	u32 *buf_ptr;
	u8 *buf_ptr_out;
	u32 offset;
	u32 size = 0;
	u32 cur_ptr = 0;
	u32 tag;
	u32 idx;
	int tag_printed;

	/*
	 * CPort logger
	 *
	 * [ log type 0 ]
	 * First 4 double words
	 *
	 * [ log type 1 ]
	 * 6 double words, for Rx
	 * 8 double words, for Tx
	 *
	 * [ log type 2 ]
	 * 4 double words, for Command UPIU, DATA OUT/IN UPIU and RTT.
	 * 2 double words, otherwise.
	 *
	 */
	log_cport->ptr = cport_readl(handle, CPORT_LOG_PTR_OFFSET);
	buf_ptr = (u32 *)&log_cport->buf[0];
	size = 0;
	offset = 0;

	while (size < CPORT_BUF_SIZE) {
		*buf_ptr = cport_readl(handle, offset);
		size += 4;
		buf_ptr += 1;
		offset += 4;
	}

	/* memory barrier for ufs cport dump */
	mb();

	dev_err(dev, "cport logging finished\n");

	/* Print data */
	buf_ptr_out = &log_cport->buf[0];
	cur_ptr = 0;

#ifdef UFS_CPORT_PRINT
	dev_err(dev, ":---------------------------------------------------\n");
	dev_err(dev, ":\t\tCPORT\n");
	dev_err(dev, ":---------------------------------------------------\n");
	while (cur_ptr < CPORT_BUF_SIZE) {
		switch (cur_ptr) {
		case CPORT_TX_BUF_PTR:
			dev_err(dev, ":---------------------------------------------------\n");
			dev_err(dev, ":\t\tTX BUF (%d)\n",
				((log_cport->ptr >> 0) & 0x3F) / 2);
			dev_err(dev, ":---------------------------------------------------\n");
			break;
		case  CPORT_RX_BUF_PTR:
			dev_err(dev, ":---------------------------------------------------\n");
			dev_err(dev, ":\t\tRX BUF (%d)\n",
				((log_cport->ptr >> 8) & 0x3F) / 2);
			dev_err(dev, ":---------------------------------------------------\n");
			break;
		case CPORT_LOG_PTR_OFFSET:
			dev_err(dev, ":---------------------------------------------------\n");
			dev_err(dev, ":\t\tCPORT LOG PTR\n");
			dev_err(dev, ":---------------------------------------------------\n");
			break;
		case CPORT_UTRL_PTR:
			dev_err(dev, ":---------------------------------------------------\n");
			dev_err(dev, ":\t\tUTRL\n");
			dev_err(dev, ":---------------------------------------------------\n");
			tag = -1;
			idx = 0;
			break;
		case CPORT_UCD_PTR:
			dev_err(dev, ":---------------------------------------------------\n");
			dev_err(dev, ":\t\tUCD\n");
			dev_err(dev, ":---------------------------------------------------\n");
			tag = -1;
			idx = 0;
			break;
		case CPORT_UTMRL_PTR:
			dev_err(dev, ":---------------------------------------------------\n");
			dev_err(dev, ": \t\tUTMRL\n");
			dev_err(dev, ":---------------------------------------------------\n");
			break;
		default:
			break;
		}

		if (cur_ptr == CPORT_LOG_PTR_OFFSET) {
			dev_err(dev, "%02x%02x%02x%02x ",
				*(buf_ptr_out + 0x3), *(buf_ptr_out + 0x2),
				*(buf_ptr_out + 0x1), *(buf_ptr_out + 0x0));
			buf_ptr_out += 0x100;
			cur_ptr += 0x100;
		} else {
			tag_printed = 0;
			if (cur_ptr >= CPORT_UTRL_PTR &&
			    cur_ptr < CPORT_UTMRL_PTR) {
				if (idx++ % 2 == 0) {
					tag_printed = 1;
					tag++;
				}
			}
			__print_cport(dev, tag_printed,
				      tag, (u32 *)buf_ptr_out);
			buf_ptr_out += 0x10;
			cur_ptr += 0x10;
		}
	}
#endif
}

static void __ufs_print_cmd_log(struct ufs_dbg_mgr *mgr, struct device *dev)
{
	struct ufs_cmd_info *cmd_info = &mgr->cmd_info;
	struct cmd_data *data = cmd_info->data;
	u32 i;
	u32 last;
	u32 max = MAX_CMD_LOGS;
	unsigned long flags;
	u32 total;

	spin_lock_irqsave(&mgr->cmd_lock, flags);
	total = cmd_info->total;
	if (cmd_info->total < max)
		max = cmd_info->total;
	last = (cmd_info->last + MAX_CMD_LOGS - 1) % MAX_CMD_LOGS;
	spin_unlock_irqrestore(&mgr->cmd_lock, flags);

	dev_err(dev, ":---------------------------------------------------\n");
	dev_err(dev, ":\t\tSCSI CMD(%u)\n", total - 1);
	dev_err(dev, ":---------------------------------------------------\n");
	dev_err(dev, ":OP, TAG, LBA, SCT, RETRIES, STIME, ETIME, REQS\n\n");

	for (i = 0 ; i < max ; i++, data++) {
		dev_err(dev, ":0x%02x, %02d, 0x%08lx, 0x%04x, %d, %llu, %llu, 0x%lx %s",
			data->op,
			data->tag,
			data->lba,
			data->sct,
			data->retries,
			data->start_time,
			data->end_time,
			data->outstanding_reqs,
			((last == i) ? "<--" : " "));
		if (last == i)
			dev_err(dev, "\n");
	}
}

static void __ufs_put_cmd_log(struct ufs_dbg_mgr *mgr,
			      struct cmd_data *cmd_data)
{
	struct ufs_cmd_info *cmd_info = &mgr->cmd_info;
	unsigned long flags;
	struct cmd_data *pdata;

	spin_lock_irqsave(&mgr->cmd_lock, flags);
	pdata = &cmd_info->data[cmd_info->last];
	++cmd_info->total;
	cmd_info->last = (++cmd_info->last) % MAX_CMD_LOGS;
	spin_unlock_irqrestore(&mgr->cmd_lock, flags);

	pdata->op = cmd_data->op;
	pdata->tag = cmd_data->tag;
	pdata->lba = cmd_data->lba;
	pdata->sct = cmd_data->sct;
	pdata->retries = cmd_data->retries;
	pdata->start_time = cmd_data->start_time;
	pdata->end_time = 0;
	pdata->outstanding_reqs = cmd_data->outstanding_reqs;
	cmd_info->pdata[cmd_data->tag] = pdata;
}

/*
 * EXTERNAL FUNCTIONS
 *
 * There are two classes that are to initialize data structures for debug
 * and to define actual behavior.
 */
void exynos_ufs_dump_info(struct ufs_vs_handle *handle, struct device *dev)
{
	struct ufs_dbg_mgr *mgr = (struct ufs_dbg_mgr *)handle->private;

	if (mgr->active == 0)
		goto out;

	mgr->time = cpu_clock(raw_smp_processor_id());

	__ufs_get_sfr(mgr, ufs_log_sfr);
	__ufs_get_attr(mgr, ufs_log_attr);

	__ufs_print_sfr(handle, dev, ufs_log_sfr);
	__ufs_print_attr(handle, dev, ufs_log_attr);

	__ufs_print_cport(mgr, dev);
	__ufs_print_cmd_log(mgr, dev);

	if (mgr->first_time == 0ULL)
		mgr->first_time = mgr->time;
out:
	return;
}

void exynos_ufs_cmd_log_start(struct ufs_vs_handle *handle,
			      struct ufs_hba *hba, struct scsi_cmnd *cmd)
{
	struct ufs_dbg_mgr *mgr = (struct ufs_dbg_mgr *)handle->private;
	int cpu = raw_smp_processor_id();
	struct cmd_data *cmd_log = &mgr->cmd_log;	/* temp buffer to put */
	unsigned long lba = (cmd->cmnd[2] << 24) |
					(cmd->cmnd[3] << 16) |
					(cmd->cmnd[4] << 8) |
					(cmd->cmnd[5] << 0);
	unsigned int sct = (cmd->cmnd[7] << 8) |
					(cmd->cmnd[8] << 0);

	if (mgr->active == 0)
		return;

	cmd_log->start_time = cpu_clock(cpu);
	cmd_log->op = cmd->cmnd[0];
	cmd_log->tag = cmd->request->tag;
	/* This function runtime is protected by spinlock from outside */
	cmd_log->outstanding_reqs = hba->outstanding_reqs;

	/* unmap */
	if (cmd->cmnd[0] != UNMAP)
		cmd_log->lba = lba;

	cmd_log->sct = sct;
	cmd_log->retries = cmd->allowed;

	__ufs_put_cmd_log(mgr, cmd_log);
}

void exynos_ufs_cmd_log_end(struct ufs_vs_handle *handle,
			    struct ufs_hba *hba, int tag)
{
	struct ufs_dbg_mgr *mgr = (struct ufs_dbg_mgr *)handle->private;
	struct ufs_cmd_info *cmd_info = &mgr->cmd_info;
	int cpu = raw_smp_processor_id();

	if (mgr->active == 0)
		return;

	cmd_info->pdata[tag]->end_time = cpu_clock(cpu);
}

int exynos_ufs_dbg_set_lanes(struct ufs_vs_handle *handle,
			     struct device *dev, u32 lanes)
{
	struct ufs_dbg_mgr *mgr = (struct ufs_dbg_mgr *)handle->private;
	int ret = 0;

	mgr->lanes = lanes;

	if (mgr->lanes > ATTR_NUM_MAX_LANES) {
		pr_err("%s: input lanes is too big: %u > %d\n",
		       __func__, mgr->lanes, ATTR_NUM_MAX_LANES);
		ret = -1;
	}

	return ret;
}

int exynos_ufs_init_dbg(struct ufs_vs_handle *handle, struct device *dev)
{
	struct ufs_dbg_mgr *mgr;
	int ret = -1;

	if (ufs_dbg_mgr_idx >= DBG_NUM_OF_HOSTS)
		goto out;

	mgr = &ufs_dbg[ufs_dbg_mgr_idx++];
	handle->private = (void *)mgr;
	mgr->handle = handle;
	mgr->active = 1;

	/* print hardware specific definitions */
	dev_info(dev, "%s:\n", __func__);
	dev_info(dev, "UNIP_COMP_AXI_AUX_FIELD = 0x%08x\n", UNIP_COMP_AXI_AUX_FIELD);
	dev_info(dev, "__WSTRB = 0x%08x\n", __WSTRB);

	/* cmd log */
	spin_lock_init(&mgr->cmd_lock);
	ret = 0;
out:
	return ret;
}
MODULE_AUTHOR("Kiwoong Kim <kwmad.kim@samsung.com>");
MODULE_DESCRIPTION("Exynos UFS debug information");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
