/*
 * drivers/media/platform/exynos/mfc/mfc_core_hw_reg_api.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_HW_REG_API_H
#define __MFC_CORE_HW_REG_API_H __FILE__

#include "mfc_core_reg_api.h"

#include "mfc_common.h"

#include "mfc_utils.h"

#define mfc_core_get_int_reason()	(MFC_CORE_READL(MFC_REG_RISC2HOST_CMD)		\
						& MFC_REG_RISC2HOST_CMD_MASK)
#define mfc_core_clear_int()				\
		do {							\
			MFC_CORE_WRITEL(0, MFC_REG_RISC2HOST_CMD);	\
			MFC_CORE_WRITEL(0, MFC_REG_RISC2HOST_INT);	\
		} while (0)

#define mfc_core_clear_int_only()				\
		do {							\
			MFC_CORE_WRITEL(0, MFC_REG_RISC2HOST_INT);	\
		} while (0)

static inline int mfc_core_wait_fw_status(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;
	unsigned int status;
	unsigned long timeout;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->wait_fw_status)) {
		status = MFC_CORE_READL(MFC_REG_FIRMWARE_STATUS_INFO);
		if (status & 0x1)
			return 0;

		timeout = jiffies + msecs_to_jiffies(MFC_BW_TIMEOUT);
		do {
			if (time_after(jiffies, timeout)) {
				mfc_core_err("Timeout while waiting MFC F/W done\n");
				return -EIO;
			}
			status = MFC_CORE_READL(MFC_REG_FIRMWARE_STATUS_INFO);
		} while ((status & 0x1) == 0);
	}

	return 0;
}

static inline int mfc_core_wait_nal_q_status(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;
	unsigned int status;
	unsigned long timeout;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->wait_nalq_status)) {
		status = MFC_CORE_READL(MFC_REG_FIRMWARE_STATUS_INFO);
		if (status & 0x2)
			return 0;

		timeout = jiffies + msecs_to_jiffies(MFC_BW_TIMEOUT);
		do {
			if (time_after(jiffies, timeout)) {
				mfc_core_err("Timeout while waiting NALQ status\n");
				return -EIO;
			}
			status = MFC_CORE_READL(MFC_REG_FIRMWARE_STATUS_INFO);
		} while ((status & 0x2) == 0);
	}

	return 0;
}

static inline int mfc_core_wait_pending(struct mfc_core *core)
{
	unsigned int status;
	unsigned long timeout;

	/* Check F/W wait status */
	timeout = jiffies + msecs_to_jiffies(MFC_BW_TIMEOUT);
	do {
		if (time_after(jiffies, timeout)) {
			mfc_core_err("Timeout while waiting MFC F/W done\n");
			return -EIO;
		}
		status = MFC_CORE_READL(MFC_REG_FIRMWARE_STATUS_INFO);
	} while ((status & 0x1) == 0);

	/* Check H/W pending status */
	timeout = jiffies + msecs_to_jiffies(MFC_BW_TIMEOUT);
	do {
		if (time_after(jiffies, timeout)) {
			mfc_core_err("Timeout while pendng clear\n");
			mfc_core_err("MFC access pending R: %#x, BUS: %#x\n",
					MFC_CORE_READL(MFC_REG_MFC_RPEND),
					MFC_CORE_READL(MFC_REG_MFC_BUS_STATUS));
			return -EIO;
		}
		status = MFC_CORE_READL(MFC_REG_MFC_RPEND);
	} while (status != 0);

	MFC_TRACE_CORE("** pending wait done\n");

	return 0;
}

static inline int mfc_core_stop_bus(struct mfc_core *core)
{
	unsigned int status;
	unsigned long timeout;

	/* Reset */
	MFC_CORE_WRITEL(0x1, MFC_REG_MFC_BUS_RESET_CTRL);

	timeout = jiffies + msecs_to_jiffies(MFC_BW_TIMEOUT);
	/* Check bus status */
	do {
		if (time_after(jiffies, timeout)) {
			mfc_core_err("Timeout while resetting MFC.\n");
			return -EIO;
		}
		status = MFC_CORE_READL(MFC_REG_MFC_BUS_RESET_CTRL);
	} while ((status & 0x2) == 0);

	return 0;
}

static inline void mfc_core_start_bus(struct mfc_core *core)
{
	int val;

	val = MFC_CORE_READL(MFC_REG_MFC_BUS_RESET_CTRL);
	val &= ~(0x1);
	MFC_CORE_WRITEL(val, MFC_REG_MFC_BUS_RESET_CTRL);
}

static inline void mfc_core_risc_on(struct mfc_core *core)
{
	mfc_core_clean_dev_int_flags(core);

	MFC_CORE_WRITEL(0x1, MFC_REG_RISC_ON);
	MFC_CORE_WRITEL(0x0, MFC_REG_MFC_OFF);
	mfc_core_debug(1, "RISC_ON\n");
	MFC_TRACE_CORE(">> RISC ON\n");
}

static inline void mfc_core_risc_off(struct mfc_core *core)
{
	unsigned int status;
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(MFC_BW_TIMEOUT);
	/* Check pending status */
	do {
		if (time_after(jiffies, timeout)) {
			mfc_core_err("Timeout while pendng clear\n");
			mfc_core_err("MFC access pending state: %#x\n", status);
			mfc_core_err("MFC access pending R: %#x, W: %#x\n",
					MFC_CORE_READL(MFC_REG_MFC_RPEND),
					MFC_CORE_READL(MFC_REG_MFC_WPEND));
			break;
		}
		status = MFC_CORE_READL(MFC_REG_MFC_BUS_STATUS);
	} while (status != 0);

	MFC_CORE_WRITEL(0x0, MFC_REG_RISC_ON);
}

static inline void mfc_core_mfc_off(struct mfc_core *core)
{
	mfc_core_info("MFC h/w state: %d\n",
			MFC_CORE_READL(MFC_REG_MFC_STATE) & 0x7);
	MFC_CORE_WRITEL(0x1, MFC_REG_MFC_OFF);
}

static inline void mfc_core_enable_all_clocks(struct mfc_core *core)
{
	/* Enable all FW clock gating */
	MFC_CORE_WRITEL(0xFFFFFFFF, MFC_REG_MFC_FW_CLOCK);
}

void mfc_core_reset_mfc(struct mfc_core *core);
void mfc_core_set_risc_base_addr(struct mfc_core *core,
				enum mfc_buf_usage_type buf_type);
void mfc_core_cmd_host2risc(struct mfc_core *core, int cmd);
int mfc_core_check_risc2host(struct mfc_core *core);
void mfc_core_set_gdc_votf(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_set_dpu_votf(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_clear_votf(struct mfc_core *core);

#endif /* __MFC_CORE_HW_REG_API_H */
