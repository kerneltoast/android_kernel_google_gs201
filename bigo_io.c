// SPDX-License-Identifier: GPL-2.0-only
/*
 * I/O methods for communication with BigOcean
 *
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/module.h>

#include "bigo_io.h"

int bigo_init_io(struct bigo_core *core, irq_handler_t handler)
{
	struct platform_device *pdev = to_platform_device(core->dev);
	int rc;

	rc = devm_request_irq(&pdev->dev, core->irq, handler, IRQF_SHARED,
			      dev_name(&pdev->dev), core);
	if (rc < 0)
		pr_err("failed to request irq: %d\n", rc);
	return rc;
}

u32 bigo_core_readl(struct bigo_core *core, ptrdiff_t offset)
{
	return readl(core->base + offset);
}

void bigo_core_writel(struct bigo_core *core, ptrdiff_t offset, u32 val)
{
	writel(val, core->base + offset);
}

void bigo_push_regs(struct bigo_core *core, void *regs)
{
	memcpy_toio(core->base, regs, core->regs_size);
}

void bigo_pull_regs(struct bigo_core *core, void *regs)
{
	memcpy_fromio(regs, core->base, core->regs_size);
}

void bigo_core_enable(struct bigo_core *core)
{
	u32 bigo_stat = bigo_core_readl(core, BIGO_REG_STAT);

	bigo_stat |= BIGO_STAT_ENABLE;
	bigo_core_writel(core, BIGO_REG_STAT, bigo_stat);
	pr_debug("core enable\n");
}

void bigo_core_disable(struct bigo_core *core)
{
	u32 bigo_stat = bigo_core_readl(core, BIGO_REG_STAT);

	bigo_stat &= ~BIGO_STAT_ENABLE;
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	bigo_stat &= ~BIGO_STAT_MODE;
	bigo_stat &= ~BIGO_STAT_CODING_MODE;
#endif
	bigo_core_writel(core, BIGO_REG_STAT, bigo_stat);
	pr_debug("core disable\n");
}

inline bool bigo_core_is_enabled(struct bigo_core *core)
{
	return bigo_core_readl(core, BIGO_REG_STAT) & BIGO_STAT_ENABLE;
}

inline int bigo_wait_disabled(struct bigo_core *core, int timeout_ms)
{
	int i;

	for (i = 0; i < timeout_ms; ++i) {
		if (!bigo_core_is_enabled(core))
			break;
		usleep_range(900, 1100);
	}
	if (i >= timeout_ms) {
		pr_err("Failed to disable the core in %d ms\n", i);
		return -ETIMEDOUT;
	}

	return 0;
}

u32 bigo_check_status(struct bigo_core *core)
{
	u32 status;
	unsigned long flags;

	spin_lock_irqsave(&core->status_lock, flags);
	status = core->stat_with_irq;
	spin_unlock_irqrestore(&core->status_lock, flags);

	if (status & BIGO_STAT_IRQ_TIMEOUT)
		pr_err("hw timedout: 0x%x\n", status);
	if (status & BIGO_STAT_IRQ_BUS_ERROR)
		pr_err("bus error: 0x%x\n", status);
	if (status & BIGO_STAT_IRQ_DEC_ERROR)
		pr_err("decoding error: 0x%x\n", status);
	if (status & BIGO_STAT_AXI_RD_OVERFLOW) {
		pr_err("axi read overflow, status: 0x%x, id: %lu\n", status,
		       status & BIG_STAT_AXI_OVERFLOW_ID);
	}
	if (status & BIGO_STAT_AXI_WR_OVERFLOW) {
		pr_err("axi write undererflow, status: 0x%x, id: %lu\n", status,
		       status & BIG_STAT_AXI_OVERFLOW_ID);
	}
	if (status & BIGO_STAT_AXI_RD_PENDING)
		pr_err("axi read pending: 0x%x\n", status);
	if (status & BIGO_STAT_AXI_WR_PENDING)
		pr_err("axi write pending: 0x%x\n", status);

	return status;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinay Kalia <vinaykalia@google.com>");
