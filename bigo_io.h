/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#ifndef _BIGO_IO_H_
#define _BIGO_IO_H_

#include <linux/interrupt.h>

#include "bigo_priv.h"

#define BIGO_REG_PRODUCT 0x00
#define BIGO_REG_CFG 0x04
#define BIGO_REG_STAT 0x08
#define BIGO_REG_LAST_RD_AXI_ADDR 0x1C
#define BIGO_REG_LAST_WR_AXI_ADDR 0x20

#define BIGO_CFG_H264 BIT(2)
#define BIGO_CFG_VP9D BIT(3)
#define BIGO_CFG_VP9E BIT(4)
#define BIGO_CFG_AV1 BIT(5)

#define BIGO_STAT_ENABLE BIT(0)
#define BIGO_STAT_MODE BIT(1)
#define BIGO_STAT_IRQ_TIMEOUT BIT(2)
#define BIGO_STAT_IRQ_BUS_ERROR BIT(3)
#define BIGO_STAT_IRQ_FRAME_READY BIT(4)
#define BIGO_STAT_IRQ_DEC_ERROR BIT(5)
#define BIGO_STAT_IRQ BIT(6)
#define BIGO_STAT_AXI_RD_OVERFLOW BIT(9)
#define BIGO_STAT_AXI_WR_OVERFLOW BIT(10)
#define BIGO_STAT_AXI_RD_PENDING BIT(19)
#define BIGO_STAT_AXI_WR_PENDING BIT(20)
#define BIGO_STAT_CODING_MODE GENMASK(22, 21)
#define BIG_STAT_AXI_OVERFLOW_ID GENMASK(30, 23)

#define BIGO_STAT_IRQMASK GENMASK(6, 2)

#define BIGO_DISABLE_TIMEOUT_MS 10
/*
 * 1. This timeout should be more than the max time HW takes to
 *    process max resolution frame at min frequency (usecase:
 *    non-realtime scenarios)
 * 2. This timeout is not for catching bigocean hardware hangs
 *    because if hardware is really hung, it should trigger an IRQ with
 *    BIGO_STAT_IRQ_TIMEOUT_BIT so HW hang should be caught there.
 * 3. This timeout is to catch any other issues with the system.
 */
#define JOB_COMPLETE_TIMEOUT_MS 1000

int bigo_init_io(struct bigo_core *core, irq_handler_t handler);
u32 bigo_core_readl(struct bigo_core *core, ptrdiff_t offset);
void bigo_core_writel(struct bigo_core *core, ptrdiff_t offset, u32 val);
void bigo_push_regs(struct bigo_core *core, void *regs);
void bigo_pull_regs(struct bigo_core *core, void *regs);
void bigo_core_enable(struct bigo_core *core);
void bigo_core_disable(struct bigo_core *core);
bool bigo_core_is_enabled(struct bigo_core *core);
int bigo_wait_disabled(struct bigo_core *core, int timeout_ms);
u32 bigo_check_status(struct bigo_core *core);

#endif //_BIGO_REGS_H_
