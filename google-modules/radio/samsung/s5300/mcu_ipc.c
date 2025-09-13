// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014-2020, Samsung Electronics.
 *
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/bitops.h>

#include "mcu_ipc.h"
#include "mcu_ipc_priv.h"
#include "modem_utils.h"

/* IRQ handler */
static irqreturn_t cp_mbox_irq_handler(int irq, void *data)
{
	struct cp_mbox_irq_data *irq_data = NULL;
	u32 irq_stat;
	int i;

	irq_data = (struct cp_mbox_irq_data *)data;
	if (!irq_data) {
		mif_err_limited("irq_data is null\n");
		return IRQ_HANDLED;
	}

	if (!irq_data->enable) {
		mif_err_limited("irq_data %d is disabled\n", irq_data->idx);
		return IRQ_HANDLED;
	}

	/*
	 * Check raised interrupts
	 * Only clear and handle unmasked interrupts
	 */
	spin_lock(&mbox_data.reg_lock);

	irq_stat = mcu_ipc_read(irq_data->sfr_rx.sr) & irq_data->sfr_rx.mask;
	irq_stat &= ~(mcu_ipc_read(irq_data->sfr_rx.mr)) & irq_data->sfr_rx.mask;
	mcu_ipc_write(irq_stat, irq_data->sfr_rx.cr);

	spin_unlock(&mbox_data.reg_lock);

	/* Call handlers */
	for (i = 0; i < MAX_CP_MBOX_HANDLER; i++) {
		if (irq_stat & (1 << (i + irq_data->sfr_rx.shift))) {
			if ((1 << (i + irq_data->sfr_rx.shift)) & irq_data->registered_irq) {
				irq_data->hd[i].handler(i, irq_data->hd[i].data);
			} else {
				mif_err_limited("unregistered:%d %d 0x%08x 0x%08lx 0x%08x\n",
						irq_data->idx, i, irq_stat,
						irq_data->unmasked_irq << irq_data->sfr_rx.shift,
						mcu_ipc_read(irq_data->sfr_rx.mr));
			}

			irq_stat &= ~(1 << (i + irq_data->sfr_rx.shift));
		}

		if (!irq_stat)
			break;
	}

	return IRQ_HANDLED;
}

/* Register / Unregister */
int cp_mbox_register_handler(u32 idx, u32 int_num, irq_handler_t handler, void *data)
{
	struct cp_mbox_irq_data *irq_data = &mbox_data.irq_data[idx];
	unsigned long flags;

	if (!handler) {
		mif_err_limited("handler is null\n");
		return -EINVAL;
	}
	if (int_num >= MAX_CP_MBOX_HANDLER) {
		mif_err_limited("int_num error:%d\n", int_num);
		return -EINVAL;
	}
	if (!irq_data->enable) {
		mif_err_limited("irq_data %d is disabled\n", irq_data->idx);
		return -EACCES;
	}

	spin_lock_irqsave(&mbox_data.reg_lock, flags);

	irq_data->hd[int_num].data = data;
	irq_data->hd[int_num].handler = handler;
	irq_data->registered_irq |= 1 << (int_num + irq_data->sfr_rx.shift);
	set_bit(int_num, &irq_data->unmasked_irq);

	spin_unlock_irqrestore(&mbox_data.reg_lock, flags);

	cp_mbox_enable_handler(irq_data->idx, int_num);
	mif_info("idx:%d num:%d intmr0:0x%08x\n",
		irq_data->idx, int_num, mcu_ipc_read(irq_data->sfr_rx.mr));

	return 0;
}
EXPORT_SYMBOL(cp_mbox_register_handler);

int cp_mbox_unregister_handler(u32 idx, u32 int_num, irq_handler_t handler)
{
	struct cp_mbox_irq_data *irq_data = &mbox_data.irq_data[idx];
	unsigned long flags;

	if (!handler) {
		mif_err_limited("handler is null\n");
		return -EINVAL;
	}
	if (irq_data->hd[int_num].handler != handler) {
		mif_err_limited("int_num error:%d\n", int_num);
		return -EINVAL;
	}
	if (!irq_data->enable) {
		mif_err_limited("irq_data %d is disabled\n", irq_data->idx);
		return -EACCES;
	}

	cp_mbox_disable_handler(irq_data->idx, int_num);
	mif_info("idx:%d num:%d intmr0:0x%08x\n",
		irq_data->idx, int_num, mcu_ipc_read(irq_data->sfr_rx.mr));

	spin_lock_irqsave(&mbox_data.reg_lock, flags);

	irq_data->hd[int_num].data = NULL;
	irq_data->hd[int_num].handler = NULL;
	irq_data->registered_irq &= ~(1 << (int_num + irq_data->sfr_rx.shift));
	clear_bit(int_num, &irq_data->unmasked_irq);

	spin_unlock_irqrestore(&mbox_data.reg_lock, flags);

	return 0;
}
EXPORT_SYMBOL(cp_mbox_unregister_handler);

/* Handler : Enable / Disable */
int cp_mbox_enable_handler(u32 idx, u32 int_num)
{
	struct cp_mbox_irq_data *irq_data = &mbox_data.irq_data[idx];
	unsigned long flags;
	unsigned long tmp;

	/* The irq should have been registered. */
	if (!(irq_data->registered_irq & BIT(int_num + irq_data->sfr_rx.shift))) {
		mif_err_limited("int_num is not registered 0x%x %d\n",
				irq_data->registered_irq, int_num);
		return -EINVAL;
	}
	if (!irq_data->enable) {
		mif_err_limited("irq_data %d is disabled\n", irq_data->idx);
		return -EACCES;
	}

	spin_lock_irqsave(&mbox_data.reg_lock, flags);

	tmp = mcu_ipc_read(irq_data->sfr_rx.mr);

	/* Clear the mask if it was set. */
	if (test_and_clear_bit(int_num + irq_data->sfr_rx.shift, &tmp))
		mcu_ipc_write(tmp, irq_data->sfr_rx.mr);

	/* Mark the irq as unmasked */
	set_bit(int_num, &irq_data->unmasked_irq);

	spin_unlock_irqrestore(&mbox_data.reg_lock, flags);

	return 0;
}
EXPORT_SYMBOL(cp_mbox_enable_handler);

int cp_mbox_disable_handler(u32 idx, u32 int_num)
{
	struct cp_mbox_irq_data *irq_data = &mbox_data.irq_data[idx];
	unsigned long flags;
	unsigned long irq_mask;

	/* The interrupt must have been registered. */
	if (!(irq_data->registered_irq & BIT(int_num + irq_data->sfr_rx.shift))) {
		mif_err_limited("int_num is not registered 0x%x %d\n",
				irq_data->registered_irq, int_num);
		return -EINVAL;
	}
	if (!irq_data->enable) {
		mif_err_limited("irq_data %d is disabled\n", irq_data->idx);
		return -EACCES;
	}

	/* Set the mask */
	spin_lock_irqsave(&mbox_data.reg_lock, flags);

	irq_mask = mcu_ipc_read(irq_data->sfr_rx.mr);

	/* Set the mask if it was not already set */
	if (!test_and_set_bit(int_num + irq_data->sfr_rx.shift, &irq_mask)) {
		mcu_ipc_write(irq_mask, irq_data->sfr_rx.mr);
		udelay(5);

		/* Reset the status bit to signal interrupt needs handling */
		mcu_ipc_write(BIT(int_num + irq_data->sfr_rx.shift), irq_data->sfr_rx.gr);
		udelay(5);
	}

	/* Remove the irq from the umasked irqs */
	clear_bit(int_num, &irq_data->unmasked_irq);

	spin_unlock_irqrestore(&mbox_data.reg_lock, flags);

	return 0;
}
EXPORT_SYMBOL(cp_mbox_disable_handler);

/*
 * This function is used to check the state of the mailbox interrupt
 * when the interrupt after the interrupt has been masked. This can be
 * used to check if a new interrupt has been set after being masked. A
 * masked interrupt will have its status set but will not generate a hard
 * interrupt. This function will check and clear the status.
 */
int cp_mbox_check_handler(u32 idx, u32 int_num)
{
	struct cp_mbox_irq_data *irq_data = &mbox_data.irq_data[idx];
	unsigned long flags;
	u32 irq_stat;

	/* Interrupt must have been registered. */
	if (!(irq_data->registered_irq & BIT(int_num + irq_data->sfr_rx.shift))) {
		mif_err_limited("int_num is not registered 0x%x %d\n",
				irq_data->registered_irq, int_num);
		return -EINVAL;
	}
	if (!irq_data->enable) {
		mif_err_limited("irq_data %d is disabled\n", irq_data->idx);
		return -EACCES;
	}

	spin_lock_irqsave(&mbox_data.reg_lock, flags);

	/* Interrupt must have been masked. */
	if (test_bit(int_num, &irq_data->unmasked_irq)) {
		spin_unlock_irqrestore(&mbox_data.reg_lock, flags);
		mif_err_limited("Mailbox interrupt (idx: %d, num: %d) is unmasked!\n",
				irq_data->idx, int_num);
		return -EINVAL;
	}

	/* Check and clear the interrupt status bit. */
	irq_stat = mcu_ipc_read(irq_data->sfr_rx.sr) & BIT(int_num + irq_data->sfr_rx.shift);
	if (irq_stat)
		mcu_ipc_write(irq_stat, irq_data->sfr_rx.cr);

	spin_unlock_irqrestore(&mbox_data.reg_lock, flags);

	return irq_stat != 0;
}
EXPORT_SYMBOL(cp_mbox_check_handler);

/* Set AP2CP interrupt */
void cp_mbox_set_interrupt(u32 idx, u32 int_num)
{
	struct cp_mbox_irq_data *irq_data = &mbox_data.irq_data[idx];

	mcu_ipc_write((0x1 << int_num) << irq_data->sfr_tx.shift, irq_data->sfr_tx.gr);
}
EXPORT_SYMBOL(cp_mbox_set_interrupt);

/* Shared register : Get / Set / Extract / Update / Dump */
static bool is_valid_sr(u32 sr_num)
{
	if (!mbox_data.num_shared_reg) {
		mif_err("num_shared_reg is 0\n");
		return false;
	}

	if (sr_num > mbox_data.num_shared_reg) {
		mif_err("num_shared_reg is %d:%d\n",
			sr_num, mbox_data.num_shared_reg);
		return false;
	}

	return true;
}

u32 cp_mbox_get_sr(u32 sr_num)
{
	if (!is_valid_sr(sr_num))
		return 0;

	return mcu_ipc_read(mbox_data.shared_reg_offset + (4 * sr_num));
}
EXPORT_SYMBOL(cp_mbox_get_sr);

u32 cp_mbox_extract_sr(u32 sr_num, u32 mask, u32 pos)
{
	if (!is_valid_sr(sr_num))
		return 0;

	return (cp_mbox_get_sr(sr_num) >> pos) & mask;
}
EXPORT_SYMBOL(cp_mbox_extract_sr);

void cp_mbox_set_sr(u32 sr_num, u32 msg)
{
	if (!is_valid_sr(sr_num))
		return;

	mcu_ipc_write(msg, mbox_data.shared_reg_offset + (4 * sr_num));
}
EXPORT_SYMBOL(cp_mbox_set_sr);

void cp_mbox_update_sr(u32 sr_num, u32 msg, u32 mask, u32 pos)
{
	u32 val;
	unsigned long flags;

	if (!is_valid_sr(sr_num))
		return;

	spin_lock_irqsave(&mbox_data.reg_lock, flags);

	val = cp_mbox_get_sr(sr_num);
	val &= ~(mask << pos);
	val |= (msg & mask) << pos;
	cp_mbox_set_sr(sr_num, val);

	spin_unlock_irqrestore(&mbox_data.reg_lock, flags);
}
EXPORT_SYMBOL(cp_mbox_update_sr);

void cp_mbox_dump_sr(void)
{
	unsigned long flags;
	u32 i, value;

	spin_lock_irqsave(&mbox_data.reg_lock, flags);

	for (i = 0; i < mbox_data.num_shared_reg; i++) {
		value = mcu_ipc_read(mbox_data.shared_reg_offset + (4 * i));
		mif_info("mbox dump: 0x%02x: 0x%04x\n", i, value);
	}

	spin_unlock_irqrestore(&mbox_data.reg_lock, flags);
}
EXPORT_SYMBOL(cp_mbox_dump_sr);

/* Reset */
void cp_mbox_reset(void)
{
	u32 reg_val;
	int i;

	mif_info("Reset mailbox registers\n");

	if (mbox_data.use_sw_reset_reg) {
		reg_val = mcu_ipc_read(EXYNOS_MCU_IPC_MCUCTLR);
		reg_val |= (0x1 << MCU_IPC_MCUCTLR_MSWRST);

		mcu_ipc_write(reg_val, EXYNOS_MCU_IPC_MCUCTLR);
		udelay(5);
	}

	for (i = 0; i < MAX_CP_MBOX_IRQ_IDX; i++) {
		struct cp_mbox_irq_data *irq_data = NULL;

		irq_data = &mbox_data.irq_data[i];
		if (!irq_data || !irq_data->name)
			break;

		mcu_ipc_write(~(irq_data->unmasked_irq) << irq_data->sfr_rx.shift,
				irq_data->sfr_rx.mr);
		mif_info("idx:%d intmr0:0x%08x\n", irq_data->idx,
				mcu_ipc_read(irq_data->sfr_rx.mr));

		mcu_ipc_write(irq_data->sfr_rx.mask, irq_data->sfr_rx.cr);
	}
}
EXPORT_SYMBOL(cp_mbox_reset);

/* IRQ affinity */
int cp_mbox_get_affinity(u32 idx)
{
	struct cp_mbox_irq_data *irq_data = &mbox_data.irq_data[idx];

	if (!irq_data) {
		mif_err("irq_data %d is null\n", idx);
		return -EINVAL;
	}

	if (!irq_data->enable) {
		mif_err_limited("irq_data %d is disabled\n", irq_data->idx);
		return -EACCES;
	}

	return irq_data->affinity;
}
EXPORT_SYMBOL(cp_mbox_get_affinity);

int cp_mbox_set_affinity(u32 idx, int affinity)
{
	struct cp_mbox_irq_data *irq_data = &mbox_data.irq_data[idx];
	int num_cpu;

	if (!irq_data) {
		mif_err("irq_data %d is null\n", idx);
		return -EINVAL;
	}

	if (!irq_data->enable) {
		mif_err_limited("irq_data %d is disabled\n", irq_data->idx);
		return -EACCES;
	}

#if defined(CONFIG_VENDOR_NR_CPUS)
	num_cpu = CONFIG_VENDOR_NR_CPUS;
#else
	num_cpu = 8;
#endif
	if (affinity >= num_cpu) {
		mif_err("idx:%d affinity:%d error. cpu max:%d\n",
			idx, affinity, num_cpu);
		return -EINVAL;
	}

	mif_debug("idx:%d affinity:0x%x\n", idx, affinity);
	irq_data->affinity = affinity;

	return irq_set_affinity_hint(irq_data->irq, cpumask_of(affinity));
}
EXPORT_SYMBOL(cp_mbox_set_affinity);

/* Probe */
static int cp_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *irq_np = NULL;
	struct device_node *irq_child_np = NULL;
	u32 count = 0;
	int irq;
	int err = 0;
	u32 idx = 0;
	u32 offset[4] = {};

	mif_info("+++\n");

	if (!dev->of_node) {
		mif_err("dev->of_node is null\n");
		err = -ENODEV;
		goto fail;
	}

	/* DMA mask */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	/* Region */
	mbox_data.ioaddr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mbox_data.ioaddr)) {
		mif_err("failed to request memory resource\n");
		err = PTR_ERR(mbox_data.ioaddr);
		goto fail;
	}

	mbox_data.dev = &pdev->dev;
	spin_lock_init(&mbox_data.reg_lock);

	/* Shared register */
	mif_dt_read_u32(dev->of_node, "num_shared_reg", mbox_data.num_shared_reg);
	mif_dt_read_u32(dev->of_node, "shared_reg_offset", mbox_data.shared_reg_offset);
	mif_info("num_shared_reg:%d shared_reg_offset:0x%x\n",
		mbox_data.num_shared_reg, mbox_data.shared_reg_offset);

	/* SW reset reg */
	mif_dt_read_bool(dev->of_node, "use_sw_reset_reg", mbox_data.use_sw_reset_reg);
	mif_info("use_sw_reset_reg:%d\n", mbox_data.use_sw_reset_reg);

	/* Interrupt */
	irq_np = of_get_child_by_name(dev->of_node, "cp_mailbox_irqs");
	if (!irq_np) {
		mif_err("of_get_child_by_name() error:irq_np\n");
		err = -EINVAL;
		goto fail;
	}
	for_each_child_of_node(irq_np, irq_child_np) {
		struct cp_mbox_irq_data *irq_data = NULL;

		if (count >= MAX_CP_MBOX_IRQ_IDX) {
			mif_err("count is full:%d\n", count);
			err = -ENOMEM;
			goto fail;
		}

		/* IRQ index */
		mif_dt_read_u32(irq_child_np, "cp_irq,idx", idx);
		irq_data = &mbox_data.irq_data[idx];
		if (!irq_data) {
			mif_err("irq_data %d is null\n", idx);
			err = -EINVAL;
			goto fail;
		}
		irq_data->idx = idx;

		/* Enable */
		mif_dt_read_bool(irq_child_np, "cp_irq,enable", irq_data->enable);
		if (!irq_data->enable) {
			mif_err("irq_data %d is disabled\n", idx);
			count++;
			continue;
		}

		/* Name */
		mif_dt_read_string(irq_child_np, "cp_irq,name", irq_data->name);

		/* SFR */
		of_property_read_u32_array(irq_child_np, "cp_irq,sfr", offset, 4);
		irq_data->sfr_rx.gr = EXYNOS_MCU_IPC_INTGR0 + offset[0];
		irq_data->sfr_rx.cr = EXYNOS_MCU_IPC_INTCR0 + offset[0];
		irq_data->sfr_rx.mr = EXYNOS_MCU_IPC_INTMR0 + offset[0];
		irq_data->sfr_rx.sr = EXYNOS_MCU_IPC_INTSR0 + offset[0];
		irq_data->sfr_rx.msr = EXYNOS_MCU_IPC_INTMSR0 + offset[0];
		irq_data->sfr_rx.shift = offset[1];
		irq_data->sfr_rx.mask = 0xFFFF << offset[1];

		irq_data->sfr_tx.gr = EXYNOS_MCU_IPC_INTGR0 + offset[2];
		irq_data->sfr_tx.cr = EXYNOS_MCU_IPC_INTCR0 + offset[2];
		irq_data->sfr_tx.mr = EXYNOS_MCU_IPC_INTMR0 + offset[2];
		irq_data->sfr_tx.sr = EXYNOS_MCU_IPC_INTSR0 + offset[2];
		irq_data->sfr_tx.msr = EXYNOS_MCU_IPC_INTMSR0 + offset[2];
		irq_data->sfr_tx.shift = offset[3];
		irq_data->sfr_tx.mask = 0xFFFF << offset[3];

		/* Request IRQ */
		irq = platform_get_irq(pdev, irq_data->idx);
		err = devm_request_irq(&pdev->dev, irq, cp_mbox_irq_handler,
					IRQF_ONESHOT, irq_data->name, irq_data);
		if (err) {
			mif_err("devm_request_irq() error:%d\n", err);
			goto fail;
		}
		err = enable_irq_wake(irq);
		if (err) {
			mif_err("enable_irq_wake() error:%d\n", err);
			goto fail;
		}
		irq_data->irq = irq;

		/* IRQ affinity */
		mif_dt_read_u32(irq_child_np, "cp_irq,affinity", irq_data->affinity);
		err = cp_mbox_set_affinity(irq_data->idx, irq_data->affinity);
		if (err)
			mif_err("cp_mbox_set_affinity() error:%d\n", err);

		/* Init CP2AP interrupt */
		mcu_ipc_write(irq_data->sfr_rx.mask, irq_data->sfr_rx.mr);
		mcu_ipc_write(irq_data->sfr_rx.mask, irq_data->sfr_rx.cr);

		mif_info("count:%d idx:%d name:%s rx.gr:0x%02x rx.shift:%d tx.gr:0x%02x tx.shift:%d affinity:%d mr:0x%08x\n",
			count, irq_data->idx, irq_data->name,
			irq_data->sfr_rx.gr, irq_data->sfr_rx.shift,
			irq_data->sfr_tx.gr, irq_data->sfr_tx.shift,
			irq_data->affinity, mcu_ipc_read(irq_data->sfr_rx.mr));

		count++;
	}

	dev_set_drvdata(dev, &mbox_data);

	mif_info("---\n");

	return 0;

fail:
	panic("CP mbox probe failed\n");
	return err;
}

static int cp_mbox_remove(struct platform_device *pdev)
{
	return 0;
}

static int cp_mbox_suspend(struct device *dev)
{
	return 0;
}

static int cp_mbox_resume(struct device *dev)
{
	struct cp_mbox_drv_data *data = dev->driver_data;
	int i;

	if (!data) {
		mif_err_limited("data is null\n");
		return -EINVAL;
	}

	for (i = 0; i < MAX_CP_MBOX_IRQ_IDX; i++) {
		struct cp_mbox_irq_data *irq_data = NULL;

		irq_data = &data->irq_data[i];
		if (!irq_data) {
			mif_err_limited("irq_data %d is null\n", i);
			return -EINVAL;
		}

		if (!irq_data->enable)
			continue;

		cp_mbox_set_affinity(irq_data->idx, irq_data->affinity);
	}

	return 0;
}

static const struct dev_pm_ops cp_mbox_pm_ops = {
	.suspend = cp_mbox_suspend,
	.resume = cp_mbox_resume,
};

static const struct of_device_id cp_mbox_dt_match[] = {
		{ .compatible = "samsung,exynos-cp-mailbox", },
		{},
};
MODULE_DEVICE_TABLE(of, cp_mbox_dt_match);

static struct platform_driver cp_mbox_driver = {
	.probe = cp_mbox_probe,
	.remove = cp_mbox_remove,
	.driver = {
		.name = "cp_mailbox",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cp_mbox_dt_match),
		.pm = &cp_mbox_pm_ops,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(cp_mbox_driver);

MODULE_DESCRIPTION("Exynos CP mailbox driver");
MODULE_LICENSE("GPL");
