// SPDX-License-Identifier: GPL-2.0-only
/*
 * cal_9855/dpp_regs.c
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Jaehoe Yang <jaehoe.yang@samsung.com>
 * Jiun Yu <jiun.yu@samsung.com>
 *
 * Register access functions for Samsung EXYNOS Display Pre-Processor driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <dpp_cal.h>

#include "regs-dpp.h"
#include "dpp_cal_internal.h"

/****************** RCD CAL functions ******************/
static void rcd_reg_set_sw_reset(u32 id)
{
	dma_write_mask(id, RCD_ENABLE, ~0, RCD_SRESET);
}

static int rcd_reg_wait_sw_reset_status(u32 id)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(dma_regs_desc(id)->regs + RCD_ENABLE,
			val, !(val & RCD_SRESET), 10, 2000); /* timeout 2ms */
	if (ret) {
		cal_log_err(id, "[idma] timeout sw-reset\n");
		return ret;
	}

	return 0;
}

static void rcd_reg_set_irq_mask_all(u32 id, bool en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, RCD_IRQ, val, RCD_ALL_IRQ_MASK);
}

static void rcd_reg_set_irq_enable(u32 id)
{
	dma_write_mask(id, RCD_IRQ, ~0, RCD_IRQ_ENABLE);
}

static void rcd_reg_set_in_qos_lut(u32 id, u32 lut_id, u32 qos_t)
{
	u32 reg_id;

	if (lut_id == 0)
		reg_id = RCD_QOS_LUT_LOW;
	else
		reg_id = RCD_QOS_LUT_HIGH;
	dma_write(id, reg_id, qos_t);
}

static void rcd_reg_set_sram_clk_gate_en(u32 id, bool en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, RCD_DYNAMIC_GATING_EN, val, RCD_SRAM_CG_EN);
}

static void rcd_reg_set_dynamic_gating_en_all(u32 id, bool en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, RCD_DYNAMIC_GATING_EN, val, RCD_DG_EN_ALL);
}

static void rcd_reg_set_ic_max(u32 id, u32 ic_max)
{
	dma_write_mask(id, RCD_IN_CTRL_0, RCD_IC_MAX(ic_max),
			RCD_IC_MAX_MASK);
}

static void rcd_reg_clear_irq(u32 id, u32 irq)
{
	dma_write_mask(id, RCD_IRQ, ~0, irq);
}

static void rcd_reg_set_base_addr(u32 id, struct dpp_params_info *p,
		const unsigned long attr)
{
	dma_write(id, RCD_BASEADDR_P0, p->addr[0]);
}

static void rcd_reg_set_coordinates(u32 id, struct decon_frame *src)
{
	dma_write(id, RCD_SRC_OFFSET,
		  RCD_SRC_OFFSET_Y(src->y) | RCD_SRC_OFFSET_X(src->x));
	dma_write(id, RCD_SRC_WIDTH, src->f_w);
	dma_write(id, RCD_SRC_HEIGHT, src->f_h);
	dma_write(id, RCD_IMG_SIZE,
		  RCD_IMG_HEIGHT(src->h) | RCD_IMG_WIDTH(src->w));
}

static void rcd_reg_set_block_mode(u32 id, bool en, int x, int y, u32 w, u32 h)
{
	if (!en) {
		dma_write_mask(id, RCD_IN_CTRL_0, 0, RCD_BLOCK_EN);
		return;
	}

	dma_write(id, RCD_BLOCK_OFFSET,
			RCD_BLK_OFFSET_Y(y) | RCD_BLK_OFFSET_X(x));
	dma_write(id, RCD_BLOCK_SIZE, RCD_BLK_HEIGHT(h) | RCD_BLK_WIDTH(w));
	dma_write(id, RCD_BLOCK_VALUE, 255 << 24);
	dma_write_mask(id, RCD_IN_CTRL_0, ~0, RCD_BLOCK_EN);

	cal_log_debug(id, "block x(%d) y(%d) w(%d) h(%d)\n", x, y, w, h);
}

static void rcd_reg_set_deadlock(u32 id, bool en, u32 dl_num)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, RCD_DEADLOCK_CTRL, val, RCD_DEADLOCK_NUM_EN);
	dma_write_mask(id, RCD_DEADLOCK_CTRL, RCD_DEADLOCK_NUM(dl_num),
				RCD_DEADLOCK_NUM_MASK);
}

static void cgc_reg_print_irqs_msg(u32 id, u32 irqs)
{
	if (irqs & CGC_READ_SLAVE_ERROR)
		cal_log_err(id, "CGC DMA read error irq occur\n");

	if (irqs & CGC_STATUS_DEADLOCK_IRQ)
		cal_log_err(id, "CGC DMA deadlock irq occur\n");

	if (irqs & CGC_CONFIG_ERR_IRQ)
		cal_log_err(id, "CGC DMA cfg err irq occur\n");
}

static void cgc_reg_clear_irq(u32 id, u32 irq)
{
	dma_write_mask(id, CGC_IRQ, ~0, irq);
}

static void cgc_reg_set_irq_en(u32 id, bool en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, CGC_IRQ, val, CGC_IRQ_ENABLE_MASK);
}

static void cgc_reg_set_irq_mask_all(u32 id, bool en)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, CGC_IRQ, val, CGC_ALL_IRQ_MASK);
}

static void cgc_reg_set_base_addr(u32 id, dma_addr_t addr)
{
	dma_write(id, CGC_BASE_ADDR_SET_0, addr);
}

static void cgc_reg_set_deadlock(u32 id, bool en, u32 dl_num)
{
	u32 val = en ? ~0 : 0;

	dma_write_mask(id, CGC_DEADLOCK_CTRL, val, CGC_DEADLOCK_NUM_EN);
	dma_write_mask(id, CGC_DEADLOCK_CTRL, CGC_DEADLOCK_NUM(dl_num),
				CGC_DEADLOCK_NUM_MASK);
}

/******************** INTERNAL RCD CAL APIs ********************/
void rcd_reg_init(u32 id)
{
	if (dma_read(id, RCD_IRQ) & RCD_DEADLOCK_IRQ) {
		rcd_reg_set_sw_reset(id);
		rcd_reg_wait_sw_reset_status(id);
	}
	rcd_reg_set_irq_mask_all(id, 0);
	rcd_reg_set_irq_enable(id);
	rcd_reg_set_in_qos_lut(id, 0, 0x44444444);
	rcd_reg_set_in_qos_lut(id, 1, 0x44444444);
	rcd_reg_set_sram_clk_gate_en(id, 0);
	rcd_reg_set_dynamic_gating_en_all(id, 0);
	rcd_reg_set_ic_max(id, 0x40);
}

int rcd_reg_deinit(u32 id, bool reset, const unsigned long attr)
{
	rcd_reg_clear_irq(id, RCD_ALL_IRQ_CLEAR);
	rcd_reg_set_irq_mask_all(id, 1);
	if (reset) {
		rcd_reg_set_sw_reset(id);
		if (rcd_reg_wait_sw_reset_status(id))
			return -1;
	}

	return 0;
}

void rcd_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr)
{
	rcd_reg_set_coordinates(id, &p->src);
	rcd_reg_set_base_addr(id, p, attr);
	rcd_reg_set_block_mode(id, p->is_block, p->block.x, p->block.y,
			p->block.w, p->block.h);
	rcd_reg_set_deadlock(id, 1, p->rcv_num * 51);

	dma_write_mask(id, RCD_ENABLE, 1, RCD_SFR_UPDATE_FORCE);
}

u32 cgc_reg_get_irq_and_clear_internal(u32 id)
{
	u32 val;

	val = dma_read(id, CGC_IRQ);
	cgc_reg_print_irqs_msg(id, val);
	cgc_reg_clear_irq(id, val);

	return val;
}

void cgc_reg_set_cgc_start_internal(u32 id)
{
	dma_write_mask(id, CGC_ENABLE, CGC_START_SET_0, CGC_START_SET_0_MASK);
}

void cgc_reg_set_config_internal(u32 id, bool en, dma_addr_t addr)
{
	if (!en) {
		cgc_reg_set_irq_en(id, 0);
		cgc_reg_set_irq_mask_all(id, 1);
		return;
	}
	cgc_reg_set_irq_en(id, 1);
	cgc_reg_set_irq_mask_all(id, 0);
	cgc_reg_set_base_addr(id, addr);
	cgc_reg_set_deadlock(id, 1, 0x7FFFFFFF);
}
