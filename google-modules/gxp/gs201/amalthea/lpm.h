/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Amalthea LPM chip-dependent settings.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __AMALTHEA_LPM_H__
#define __AMALTHEA_LPM_H__

#include <linux/types.h>

enum gxp_lpm_psm {
	LPM_PSM_CORE0,
	LPM_PSM_CORE1,
	LPM_PSM_CORE2,
	LPM_PSM_CORE3,
	LPM_PSM_TOP,
	LPM_NUM_PSMS,
};

#define CORE_TO_PSM(core) (LPM_PSM_CORE0 + (core))

enum lpm_psm_csrs {
	LPM_REG_ENABLE_STATE_0 = 0x080,
	LPM_REG_ENABLE_STATE_1 = 0x180,
	LPM_REG_ENABLE_STATE_2 = 0x280,
	LPM_REG_ENABLE_STATE_3 = 0x380,
};

/* offset from GXP_LPM_BASE */
enum lpm_psm_base {
	GXP_REG_LPM_PSM_0 = 0x1000,
	GXP_REG_LPM_PSM_1 = 0x2000,
	GXP_REG_LPM_PSM_2 = 0x3000,
	GXP_REG_LPM_PSM_3 = 0x4000,
	GXP_REG_LPM_PSM_4 = 0x5000,
};

#define LPM_STATE_TABLE_SIZE (LPM_REG_ENABLE_STATE_1 - LPM_REG_ENABLE_STATE_0)

/* LPM address space starts at lpm_version register */
#define GXP_LPM_BASE GXP_REG_LPM_VERSION
#define GXP_LPM_PSM_0_BASE GXP_REG_LPM_PSM_0
#define GXP_LPM_PSM_SIZE (GXP_REG_LPM_PSM_1 - GXP_REG_LPM_PSM_0)

/* LPM Registers */
#define LPM_VERSION_OFFSET 0x0
#define TRIGGER_CSR_START_OFFSET 0x4
#define IMEM_START_OFFSET 0x8
#define LPM_CONFIG_OFFSET 0xC
#define PSM_DESCRIPTOR_OFFSET 0x10
#define EVENTS_EN_OFFSET 0x100
#define EVENTS_INV_OFFSET 0x140
#define FUNCTION_SELECT_OFFSET 0x180
#define TRIGGER_STATUS_OFFSET 0x184
#define EVENT_STATUS_OFFSET 0x188
#define OPS_OFFSET 0x800
#define PSM_DESCRIPTOR_BASE(_x_) ((_x_) << 2)
#define PSM_DESCRIPTOR_COUNT 5
#define EVENTS_EN_BASE(_x_) ((_x_) << 2)
#define EVENTS_EN_COUNT 16
#define EVENTS_INV_BASE(_x_) ((_x_) << 2)
#define EVENTS_INV_COUNT 16
#define OPS_BASE(_x_) ((_x_) << 2)
#define OPS_COUNT 128
#define PSM_COUNT 5
#define PSM_STATE_TABLE_BASE(_x_) ((_x_) << 8)
#define PSM_STATE_TABLE_COUNT 6
#define PSM_TRANS_BASE(_x_) ((_x_) << 5)
#define PSM_TRANS_COUNT 4
#define PSM_DMEM_BASE(_x_) ((_x_) << 2)
#define PSM_DATA_COUNT 32
#define PSM_NEXT_STATE_OFFSET 0x0
#define PSM_SEQ_ADDR_OFFSET 0x4
#define PSM_TIMER_VAL_OFFSET 0x8
#define PSM_TIMER_EN_OFFSET 0xC
#define PSM_TRIGGER_NUM_OFFSET 0x10
#define PSM_TRIGGER_EN_OFFSET 0x14
#define PSM_ENABLE_STATE_OFFSET 0x80
#define PSM_DATA_OFFSET 0x600
#define PSM_CFG_OFFSET 0x680
#define PSM_START_OFFSET 0x684
#define PSM_STATUS_OFFSET 0x688
#define PSM_DEBUG_CFG_OFFSET 0x68C
#define PSM_BREAK_ADDR_OFFSET 0x694
#define PSM_GPIN_LO_RD_OFFSET 0x6A0
#define PSM_GPIN_HI_RD_OFFSET 0x6A4
#define PSM_GPOUT_LO_WRT_OFFSET 0x6A8
#define PSM_GPOUT_LO_RD_OFFSET 0x6B0
#define PSM_GPOUT_HI_RD_OFFSET 0x6B4
#define PSM_DEBUG_STATUS_OFFSET 0x6B8

static inline u32 gxp_lpm_psm_get_status_offset(enum gxp_lpm_psm psm)
{
	if (psm >= LPM_NUM_PSMS)
		return 0;
	return GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + PSM_STATUS_OFFSET;
}

static inline u32 gxp_lpm_psm_get_start_offset(enum gxp_lpm_psm psm)
{
	if (psm >= LPM_NUM_PSMS)
		return 0;
	return GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + PSM_START_OFFSET;
}

static inline u32 gxp_lpm_psm_get_cfg_offset(enum gxp_lpm_psm psm)
{
	if (psm >= LPM_NUM_PSMS)
		return 0;
	return GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + PSM_CFG_OFFSET;
}

static inline u32 gxp_lpm_psm_get_state_offset(enum gxp_lpm_psm psm, uint state)
{
	uint reg_offset;

	if (psm >= LPM_NUM_PSMS || state > 3)
		return 0;

	switch (state) {
	case 0:
		reg_offset = LPM_REG_ENABLE_STATE_0;
		break;
	case 1:
		reg_offset = LPM_REG_ENABLE_STATE_1;
		break;
	case 2:
		reg_offset = LPM_REG_ENABLE_STATE_2;
		break;
	case 3:
		reg_offset = LPM_REG_ENABLE_STATE_3;
		break;
	}
	return GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + reg_offset;
}

static inline u32 gxp_lpm_psm_get_debug_cfg_offset(enum gxp_lpm_psm psm)
{
	if (psm >= LPM_NUM_PSMS)
		return 0;
	return GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + PSM_DEBUG_CFG_OFFSET;
}

static inline u32 gxp_lpm_psm_get_gpin_lo_rd_offset(enum gxp_lpm_psm psm)
{
	if (psm >= LPM_NUM_PSMS)
		return 0;
	return GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + PSM_GPIN_LO_RD_OFFSET;
}

static inline u32 gxp_lpm_psm_get_gpout_lo_wrt_offset(enum gxp_lpm_psm psm)
{
	if (psm >= LPM_NUM_PSMS)
		return 0;
	return GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + PSM_GPOUT_LO_WRT_OFFSET;
}

static inline u32 gxp_lpm_psm_get_gpout_lo_rd_offset(enum gxp_lpm_psm psm)
{
	if (psm >= LPM_NUM_PSMS)
		return 0;
	return GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + PSM_GPOUT_LO_RD_OFFSET;
}

#endif /* __AMALTHEA_LPM_H__ */
