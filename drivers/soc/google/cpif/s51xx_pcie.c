// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe modem control driver for S51xx series
 *
 * Copyright (C) 2019 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/if_arp.h>
#include <linux/version.h>

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
//#include <sound/samsung/abox.h>

#include <linux/exynos-pci-ctrl.h>
#include <linux/exynos-pci-noti.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "s51xx_pcie.h"

static int s51xx_pcie_read_procmem(struct seq_file *m, void *v)
{
	pr_err("Procmem READ!\n");

	return 0;
}

static int s51xx_pcie_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, s51xx_pcie_read_procmem, NULL);
}

static const struct proc_ops s51xx_pcie_proc_fops = {
	.proc_open = s51xx_pcie_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

void s51xx_pcie_chk_ep_conf(struct pci_dev *pdev)
{
	int i;
	u32 val1, val2, val3, val4;

	/* EP config. full dump: */
	for (i = 0x0; i < 0x50; i += 0x10) {
		pci_read_config_dword(pdev, i, &val1);
		pci_read_config_dword(pdev, i + 0x4, &val2);
		pci_read_config_dword(pdev, i + 0x8, &val3);
		pci_read_config_dword(pdev, i + 0xC, &val4);
		dev_dbg(&pdev->dev, "0x%02x:  %08x  %08x  %08x  %08x\n",
				i, val1, val2, val3, val4);
	}
}

inline int s51xx_pcie_send_doorbell_int(struct pci_dev *pdev, int int_num)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);
	u32 reg, count = 0;
	int cnt = 0;
	u16 cmd;

	if (s51xx_pcie->link_status == 0) {
		mif_err_limited("Can't send Interrupt(not enabled)!!!\n");
		return -EAGAIN;
	}

	if (s51xx_check_pcie_link_status(s51xx_pcie->pcie_channel_num) == 0) {
		mif_err_limited("Can't send Interrupt(not linked)!!!\n");
		return -EAGAIN;
	}

	pci_read_config_word(pdev, PCI_COMMAND, &cmd);
	if ((((cmd & PCI_COMMAND_MEMORY) == 0) ||
			(cmd & PCI_COMMAND_MASTER) == 0) || (cmd == 0xffff)) {
		mif_err_limited("Can't send Interrupt(not setted bme_en, 0x%04x)!!!\n", cmd);

		do {
			cnt++;

			/* set bme bit */
			pci_set_master(pdev);

			pci_read_config_word(pdev, PCI_COMMAND, &cmd);
			mif_info("cmd reg = 0x%04x\n", cmd);

			/* set mse bit */
			cmd |= PCI_COMMAND_MEMORY;
			pci_write_config_word(pdev, PCI_COMMAND, cmd);

			pci_read_config_word(pdev, PCI_COMMAND, &cmd);
			mif_info("cmd reg = 0x%04x\n", cmd);

			if ((cmd & PCI_COMMAND_MEMORY) &&
					(cmd & PCI_COMMAND_MASTER) && (cmd != 0xffff))
				break;
		} while (cnt < 10);

		if (cnt >= 10) {
			mif_err_limited("BME is not set(cnt=%d)\n", cnt);
			exynos_pcie_rc_register_dump(
					s51xx_pcie->pcie_channel_num);
			return -EAGAIN;
		}
	}

send_doorbell_again:
	iowrite32(int_num, s51xx_pcie->doorbell_addr);

	reg = ioread32(s51xx_pcie->doorbell_addr);

	/* debugging:
	 * mif_info("s51xx_pcie.doorbell_addr = 0x%p -
	 * written(int_num=0x%x) read(reg=0x%x)\n", \
	 *	s51xx_pcie->doorbell_addr, int_num, reg);
	 */

	if (reg == 0xffffffff) {
		count++;
		if (count < 100) {
			if (!in_interrupt())
				udelay(1000); /* 1ms */
			else {
				mif_err_limited("Can't send doorbell in interrupt mode (0x%08X)\n"
						, reg);
				return 0;
			}

			goto send_doorbell_again;
		}
		mif_err("[Need to CHECK] Can't send doorbell int (0x%x)\n", reg);
		exynos_pcie_rc_register_dump(s51xx_pcie->pcie_channel_num);

		return -EAGAIN;
	}

	return 0;
}

void first_save_s51xx_status(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);

	if (s51xx_check_pcie_link_status(s51xx_pcie->pcie_channel_num) == 0) {
		mif_err("It's not Linked - Ignore saving the s5100\n");
		return;
	}

	pci_save_state(pdev);
	s51xx_pcie->pci_saved_configs = pci_store_saved_state(pdev);
	if (s51xx_pcie->pci_saved_configs == NULL)
		mif_err("MSI-DBG: s51xx pcie.pci_saved_configs is NULL(s51xx config NOT saved)\n");
	else
		mif_err("first s51xx config status save: done\n");
}

void s51xx_pcie_save_state(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "[%s]\n", __func__);

	if (s51xx_check_pcie_link_status(s51xx_pcie->pcie_channel_num) == 0) {
		mif_err("It's not Linked - Ignore restore state!!!\n");
		return;
	}

	/* pci_pme_active(s51xx_pcie.s51xx_pdev, 0); */

	/* Disable L1.2 before PCIe power off */
	s51xx_pcie_l1ss_ctrl(0, s51xx_pcie->pcie_channel_num);

	pci_clear_master(pdev);

	if (s51xx_pcie->pci_saved_configs)
		kfree(s51xx_pcie->pci_saved_configs);

	pci_save_state(pdev);

	s51xx_pcie->pci_saved_configs = pci_store_saved_state(pdev);

	s51xx_pcie_chk_ep_conf(pdev);

	disable_msi_int(pdev);

	/* pci_enable_wake(s51xx_pcie.s51xx_pdev, PCI_D0, 0); */

	pci_disable_device(pdev);

	pci_wake_from_d3(pdev, false);
	if (pci_set_power_state(pdev, PCI_D3hot))
		mif_err("Can't set D3 state!!!!\n");
}

void s51xx_pcie_restore_state(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);
	int ret;
	u32 val;

	dev_dbg(&pdev->dev, "[%s]\n", __func__);

	if (s51xx_check_pcie_link_status(s51xx_pcie->pcie_channel_num) == 0) {
		mif_err("It's not Linked - Ignore restore state!!!\n");
		return;
	}

	if (pci_set_power_state(pdev, PCI_D0))
		mif_err("Can't set D0 state!!!!\n");

	if (!s51xx_pcie->pci_saved_configs)
		dev_err(&pdev->dev, "[%s] s51xx pcie saved configs is NULL\n", __func__);

	pci_load_saved_state(pdev, s51xx_pcie->pci_saved_configs);
	pci_restore_state(pdev);

	/* move chk_ep_conf function after setting BME(Bus Master Enable)
	 * s51xx_pcie_chk_ep_conf(pdev);
	 */

	pci_enable_wake(pdev, PCI_D0, 0);
	/* pci_enable_wake(s51xx_pcie.s51xx_pdev, PCI_D3hot, 0); */

	ret = pci_enable_device(pdev);

	if (ret)
		pr_err("Can't enable PCIe Device after linkup!\n");
	pci_set_master(pdev);

	/* DBG: print out EP config values after restore_state */
	s51xx_pcie_chk_ep_conf(pdev);

	/* BAR0 value correction  */
	pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &val);
	dev_dbg(&pdev->dev, "[%s]restored:PCI_BASE_ADDRESS_0 = 0x%x\n", __func__, val);
	if ((val & 0xfffffff0) != 0x40000000) {
		pci_write_config_dword(pdev, PCI_BASE_ADDRESS_0, 0x40000000);
		pci_write_config_dword(pdev, PCI_BASE_ADDRESS_1, 0x0);
		mif_info("write BAR0 value: 0x40000000\n");
		dev_info(&pdev->dev, "[%s]:VALUE CHECK\n", __func__);
		s51xx_pcie_chk_ep_conf(pdev);
	}

	/* Enable L1.2 after PCIe power on */
	s51xx_pcie_l1ss_ctrl(1, s51xx_pcie->pcie_channel_num);

	s51xx_pcie->link_status = 1;
	/* pci_pme_active(s51xx_pcie.s51xx_pdev, 1); */
}

int s51xx_check_pcie_link_status(int ch_num)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	return exynos_pcie_rc_chk_link_status(ch_num);
#else
	return exynos_check_pcie_link_status(ch_num);
#endif
}

void s51xx_pcie_l1ss_ctrl(int enable, int ch_num)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	exynos_pcie_rc_l1ss_ctrl(enable, PCIE_L1SS_CTRL_MODEM_IF, ch_num);
#else
	exynos_pcie_host_v1_l1ss_ctrl(enable, PCIE_L1SS_CTRL_MODEM_IF);
#endif
}

void disable_msi_int(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "[%s]\n", __func__);

	s51xx_pcie->link_status = 0;
	/* It's not needed now...
	 * pci_disable_msi(s51xx_pcie.s51xx_pdev);
	 * pci_config_pm_runtime_put(&s51xx_pcie.s51xx_pdev->dev);
	 */
}

int s51xx_pcie_request_msi_int(struct pci_dev *pdev, int int_num)
{
	int err = -EFAULT;

	if (int_num > MAX_MSI_NUM) {
		pr_err("Too many MSI interrupts are requested(<=16)!!!\n");
		return -EFAULT;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
	/* old kernel(KC+S359):
	 * err = __pci_enable_msi_range(s51xx_pcie.s51xx_pdev, int_num, int_num);
	 */
	err = pci_alloc_irq_vectors_affinity(pdev, int_num, int_num, PCI_IRQ_MSI, NULL);
#else
	err = pci_enable_msi_block(pdev, int_num);
#endif

	if (err <= 0) {
		pr_err("Can't get msi IRQ!!!!!\n");
		return -EFAULT;
	}

	return pdev->irq;
}

static void s51xx_pcie_linkdown_cb(struct exynos_pcie_notify *noti)
{
	struct pci_dev *pdev = (struct pci_dev *)noti->user;
	struct pci_driver *driver = pdev->driver;
	struct modem_ctl *mc = container_of(driver, struct modem_ctl, pci_driver);

	pr_err("s51xx Link-Down notification callback function!!!\n");

	if (mc->pcie_powered_on == false) {
		pr_info("%s: skip cp crash during dislink sequence\n", __func__);
		exynos_pcie_set_perst_gpio(mc->pcie_ch_num, 0);
	} else {
		s5100_force_crash_exit_ext();
	}
}

static int s51xx_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int ret;
	int __maybe_unused i;
	struct s51xx_pcie *s51xx_pcie;
	struct device *dev = &pdev->dev;
	struct pci_driver *driver = pdev->driver;
	struct modem_ctl *mc = container_of(driver, struct modem_ctl, pci_driver);
	struct device *mc_dev = mc->dev;
	struct pci_bus *bus = pdev->bus;
	struct pci_dev *bus_self = bus->self;
	struct resource *tmp_rsc;
	int resno = PCI_BRIDGE_MEM_WINDOW;
	u32 val, db_addr;

	dev_info(dev, "%s EP driver Probe(%s), chNum: %d\n",
			driver->name, __func__, mc->pcie_ch_num);

	s51xx_pcie = devm_kzalloc(dev, sizeof(*s51xx_pcie), GFP_KERNEL);
	s51xx_pcie->s51xx_pdev = pdev;
	s51xx_pcie->irq_num_base = pdev->irq;
	s51xx_pcie->link_status = 1;
	s51xx_pcie->pcie_channel_num = mc->pcie_ch_num;

	mc->s51xx_pdev = pdev;

	if (of_property_read_u32(mc_dev->of_node, "pci_db_addr", &db_addr)) {
		dev_err(dev, "Failed to parse the EP DB base address\n");
		return -EINVAL;
	}

	pci_write_config_dword(pdev, PCI_BASE_ADDRESS_0, db_addr);
	pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &val);
	val &= PCI_BASE_ADDRESS_MEM_MASK;
	s51xx_pcie->dbaddr_offset = db_addr - val;
	dev_info(dev, "db_addr : 0x%x , val : 0x%x, offset : 0x%x\n",
			db_addr, val, (unsigned int)s51xx_pcie->dbaddr_offset);

	pr_err("Disable BAR resources.\n");
	for (i = 0; i < 6; i++) {
		pdev->resource[i].start = 0x0;
		pdev->resource[i].end = 0x0;
		pdev->resource[i].flags = 0x82000000;
		pci_assign_resource(pdev, i);
	}

	/* EP BAR setup: BAR0 (4kB) */
	pdev->resource[0].start = val;
	pdev->resource[0].end = val + SZ_4K;
	pdev->resource[0].flags = 0x82000000;
	pci_assign_resource(pdev, 0);

	/* get Doorbell base address from root bus range */
	tmp_rsc = bus_self->resource + resno;
	dev_info(&bus_self->dev, "[%s] BAR %d: tmp rsc : %pR\n", __func__, resno, tmp_rsc);
	s51xx_pcie->dbaddr_base = tmp_rsc->start;

	pr_err("Set Doorbell register address.\n");
	s51xx_pcie->doorbell_addr = devm_ioremap(&pdev->dev,
			s51xx_pcie->dbaddr_base + s51xx_pcie->dbaddr_offset, SZ_4);

	/*
	 * ret = abox_pci_doorbell_paddr_set(s51xx_pcie->dbaddr_base +
	 * s51xx_pcie->dbaddr_offset);
	 * if (!ret)
	 * dev_err(dev, "PCIe doorbell setting for ABOX is failed\n");
	 */

	pr_info("s51xx_pcie.doorbell_addr = %p  (start 0x%lx offset : %lx)\n",
		s51xx_pcie->doorbell_addr, (unsigned long)s51xx_pcie->dbaddr_base,
					(unsigned long)s51xx_pcie->dbaddr_offset);

	if (s51xx_pcie->doorbell_addr == NULL)
		pr_err("Can't ioremap doorbell address!!!\n");

	pr_info("Register PCIE notification LINKDOWN event...\n");
	s51xx_pcie->pcie_event.events = EXYNOS_PCIE_EVENT_LINKDOWN;
	s51xx_pcie->pcie_event.user = pdev;
	s51xx_pcie->pcie_event.mode = EXYNOS_PCIE_TRIGGER_CALLBACK;
	s51xx_pcie->pcie_event.callback = s51xx_pcie_linkdown_cb;
	exynos_pcie_register_event(&s51xx_pcie->pcie_event);

	pr_err("Enable PCI device...\n");
	ret = pci_enable_device(pdev);

	pci_set_master(pdev);

	pci_set_drvdata(pdev, s51xx_pcie);

	return 0;
}

void print_msi_register(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);
	u32 msi_val;

	pci_read_config_dword(pdev, 0x50, &msi_val);
	mif_debug("MSI Control Reg(0x50) : 0x%x\n", msi_val);
	pci_read_config_dword(pdev, 0x54, &msi_val);
	mif_debug("MSI Message Reg(0x54) : 0x%x\n", msi_val);
	pci_read_config_dword(pdev, 0x58, &msi_val);
	mif_debug("MSI MsgData Reg(0x58) : 0x%x\n", msi_val);

	if (msi_val == 0x0) {
		mif_debug("MSI Message Reg == 0x0 - set MSI again!!!\n");

		if (s51xx_pcie->pci_saved_configs != NULL) {
			mif_debug("msi restore\n");
			pci_restore_msi_state(pdev);
		} else {
			mif_debug("[skip] msi restore: saved configs is NULL\n");
		}

		mif_debug("exynos_pcie_msi_init_ext is not implemented\n");
		/* exynos_pcie_msi_init_ext(s51xx_pcie.pcie_channel_num); */

		pci_read_config_dword(pdev, 0x50, &msi_val);
		mif_debug("Recheck - MSI Control Reg : 0x%x (0x50)\n", msi_val);
		pci_read_config_dword(pdev, 0x54, &msi_val);
		mif_debug("Recheck - MSI Message Reg : 0x%x (0x54)\n", msi_val);
		pci_read_config_dword(pdev, 0x58, &msi_val);
		mif_debug("Recheck - MSI MsgData Reg : 0x%x (0x58)\n", msi_val);
	}
}

static void s51xx_pcie_remove(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);

	pr_err("s51xx PCIe Remove!!!\n");

	if (s51xx_pcie->pci_saved_configs)
		kfree(s51xx_pcie->pci_saved_configs);

	pci_release_regions(pdev);
}

/* For Test */
static struct pci_device_id s51xx_pci_id_tbl[] = {
	{ PCI_VENDOR_ID_SAMSUNG, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, },   // SC Basic
	{ }
};

MODULE_DEVICE_TABLE(pci, s51xx_pci_id_tbl);

static struct pci_driver s51xx_driver = {
	.name = "s51xx",
	.id_table = s51xx_pci_id_tbl,
	.probe = s51xx_pcie_probe,
	.remove = s51xx_pcie_remove,
};

/*
 * Initialize PCIe s51xx EP driver.
 */
int s51xx_pcie_init(struct modem_ctl *mc)
{
	int ch_num = mc->pcie_ch_num;
	int ret;

	pr_err("Register PCIE drvier for s51xx.(chNum: %d, mc: 0x%p)\n", ch_num, mc);

	mc->pci_driver = s51xx_driver;

	ret = pci_register_driver(&mc->pci_driver);

	/* Create PROC fs */
	proc_create("driver/s51xx_pcie_proc", 0, NULL, &s51xx_pcie_proc_fops);

	return 0;
}
