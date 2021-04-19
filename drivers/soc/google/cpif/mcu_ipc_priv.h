/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2014-2020, Samsung Electronics.
 *
 */

#ifndef __MCU_IPC_PRIV_H__
#define __MCU_IPC_PRIV_H__

#include <dt-bindings/soc/google/exynos-cpif.h>

/* Registers */
#define EXYNOS_MCU_IPC_MCUCTLR			0x0
#define EXYNOS_MCU_IPC_INTGR0			0x8
#define EXYNOS_MCU_IPC_INTCR0			0xc
#define EXYNOS_MCU_IPC_INTMR0			0x10
#define EXYNOS_MCU_IPC_INTSR0			0x14
#define EXYNOS_MCU_IPC_INTMSR0			0x18

/* Bits definition */
#define MCU_IPC_MCUCTLR_MSWRST	(0)

/* */
#define MAX_CP_MBOX_HANDLER		16
struct cp_mbox_handler {
	void *data;
	irq_handler_t handler;
};

struct cp_mbox_irq_sfr {
	u32 gr;
	u32 cr;
	u32 mr;
	u32 sr;
	u32 msr;
	u32 mask;
	u32 shift;
};

struct cp_mbox_irq_data {
	char *name;
	u32 idx;
	bool enable;

	struct cp_mbox_irq_sfr sfr_rx;
	struct cp_mbox_irq_sfr sfr_tx;

	int irq;
	int affinity;
	u32 registered_irq;
	unsigned long unmasked_irq;

	struct cp_mbox_handler hd[MAX_CP_MBOX_HANDLER];
};

struct cp_mbox_drv_data {
	void __iomem *ioaddr;

	struct device *dev;

	u32 num_shared_reg;
	u32 shared_reg_offset;
	bool use_sw_reset_reg;

	struct cp_mbox_irq_data irq_data[MAX_CP_MBOX_IRQ_IDX];

	spinlock_t reg_lock;

	int irq;
};

/* */
static struct cp_mbox_drv_data mbox_data;

static inline void mcu_ipc_write(u32 val, u32 reg)
{
	writel(val, mbox_data.ioaddr + reg);
}

static inline u32 mcu_ipc_read(u32 reg)
{
	return readl(mbox_data.ioaddr + reg);
}

#endif /* __MCU_IPC_PRIV_H__ */
