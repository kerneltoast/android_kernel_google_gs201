// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe host controller driver for gs101 SoC
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/gpio.h>

#include "pcie-designware.h"
#include "pcie-exynos-common.h"
#include "pcie-exynos-dbg.h"

static int chk_pcie_dislink(struct exynos_pcie *exynos_pcie)
{
	int test_result = 0;
	struct exynos_pcie_ops *pcie_ops = &exynos_pcie->exynos_pcie_ops;
	u32 linkup_offset = exynos_pcie->linkup_offset;
	u32 val;

	if (!pcie_ops->poweroff) {
		pr_err("Can't find PCIe poweroff function\n");

		return -1;
	}

	pcie_ops->poweroff(exynos_pcie->ch_num);

	if (exynos_pcie->use_phy_isol_con &&
	    exynos_pcie->phy_control == PCIE_PHY_ISOLATION)
		return test_result;

	val = exynos_elbi_read(exynos_pcie, linkup_offset) & 0x1f;
	if (val == 0x15) {
		pr_info("PCIe link Down test Success.\n");
	} else {
		pr_info("PCIe Link Down test Fail...\n");
		test_result = -1;
	}

	return test_result;
}

static int chk_link_recovery(struct exynos_pcie *exynos_pcie)
{
	int test_result = 0;
	struct exynos_pcie_ops *pcie_ops = &exynos_pcie->exynos_pcie_ops;
	u32 linkup_offset = exynos_pcie->linkup_offset;
	u32 val;

	if (!pcie_ops->poweroff) {
		pr_err("Can't find PCIe poweroff function\n");

		return -1;
	}

	if (!pcie_ops->poweron) {
		pr_err("Can't find PCIe poweron function\n");

		return -1;
	}

	exynos_elbi_write(exynos_pcie, 0x1, exynos_pcie->app_req_exit_l1);
	val = exynos_elbi_read(exynos_pcie, exynos_pcie->app_req_exit_l1_mode);
	val &= ~APP_REQ_EXIT_L1_MODE;
	exynos_elbi_write(exynos_pcie, val, exynos_pcie->app_req_exit_l1_mode);
	pr_info("%s: Before set perst, gpio val = %d\n",
		__func__, gpio_get_value(exynos_pcie->perst_gpio));
	gpio_set_value(exynos_pcie->perst_gpio, 0);
	pr_info("%s: After set perst, gpio val = %d\n",
		__func__, gpio_get_value(exynos_pcie->perst_gpio));
	val = exynos_elbi_read(exynos_pcie, exynos_pcie->app_req_exit_l1_mode);
	val |= APP_REQ_EXIT_L1_MODE;
	exynos_elbi_write(exynos_pcie, val, exynos_pcie->app_req_exit_l1_mode);
	exynos_elbi_write(exynos_pcie, 0x0, exynos_pcie->app_req_exit_l1);
	msleep(5000);

	val = exynos_elbi_read(exynos_pcie, linkup_offset) & 0x1f;
	if (val >= 0x0d && val <= 0x14) {
		pr_info("PCIe link Recovery test Success.\n");
	} else {
		/*
		 * If recovery callback is defined, pcie poweron
		 * function will not be called.
		 */
		pcie_ops->poweroff(exynos_pcie->ch_num);
		pcie_ops->poweron(exynos_pcie->ch_num);
		val = exynos_elbi_read(exynos_pcie, linkup_offset) & 0x1f;
		if (val >= 0x0d && val <= 0x14) {
			pr_info("PCIe link Recovery test Success.\n");
		} else {
			pr_info("PCIe Link Recovery test Fail...\n");
			test_result = -1;
		}
	}

	return test_result;
}

static int chk_epmem_access(struct exynos_pcie *exynos_pcie)
{
	u32 val;
	int test_result = 0;
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct exynos_pcie_ops *pcie_ops = &exynos_pcie->exynos_pcie_ops;

	struct pci_bus *ep_pci_bus;
	void __iomem *reg_addr;
	struct resource_entry *entry =
		resource_list_first_type(&pp->bridge->windows, IORESOURCE_MEM);

	if (!pcie_ops->rd_other_conf) {
		pr_err("Can't find PCIe read other configuration function\n");
		return -1;
	}

	if (!pcie_ops->wr_other_conf) {
		pr_err("Can't find PCIe write other configuration function\n");
		return -1;
	}

	ep_pci_bus = pci_find_bus(exynos_pcie->pci_dev->bus->domain_nr, 1);
	pcie_ops->wr_other_conf(pp, ep_pci_bus, 0, PCI_BASE_ADDRESS_0,
				4, lower_32_bits(entry->res->start));
	pcie_ops->rd_other_conf(pp, ep_pci_bus, 0, PCI_BASE_ADDRESS_0,
				4, &val);
	pr_info("Set BAR0 : 0x%x\n", val);

	reg_addr = ioremap(entry->res->start, SZ_4K);

	val = readl(reg_addr);
	iounmap(reg_addr);
	if (val != 0xffffffff) {
		pr_info("PCIe EP Outbound mem access Success.\n");
	} else {
		pr_info("PCIe EP Outbound mem access Fail...\n");
		test_result = -1;
	}

	return test_result;
}

static int chk_epconf_access(struct exynos_pcie *exynos_pcie)
{
	u32 val;
	int test_result = 0;
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct pci_bus *ep_pci_bus;
	struct exynos_pcie_ops *pcie_ops = &exynos_pcie->exynos_pcie_ops;

	if (!pcie_ops->rd_other_conf) {
		pr_err("Can't find PCIe read other configuration function\n");

		return -1;
	}

	if (!pcie_ops->wr_other_conf) {
		pr_err("Can't find PCIe write other configuration function\n");

		return -1;
	}

	ep_pci_bus = pci_find_bus(exynos_pcie->pci_dev->bus->domain_nr, 1);

	pcie_ops->rd_other_conf(pp, ep_pci_bus, 0, 0x0, 4, &val);
	pr_info("PCIe EP Vendor ID/Device ID = 0x%x\n", val);

	pcie_ops->wr_other_conf(pp, ep_pci_bus,
					0, PCI_COMMAND, 4, 0x146);
	pcie_ops->rd_other_conf(pp, ep_pci_bus,
					0, PCI_COMMAND, 4, &val);
	if ((val & 0xfff) == 0x146) {
		pr_info("PCIe EP conf access Success.\n");
	} else {
		pr_info("PCIe EP conf access Fail...\n");
		test_result = -1;
	}

	return test_result;
}

static int chk_dbi_access(struct exynos_pcie *exynos_pcie)
{
	u32 val;
	int test_result = 0;
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct exynos_pcie_ops *pcie_ops = &exynos_pcie->exynos_pcie_ops;

	if (!pcie_ops->rd_own_conf) {
		pr_err("Can't find PCIe read own configuration function\n");

		return -1;
	}

	if (!pcie_ops->wr_own_conf) {
		pr_err("Can't find PCIe write own configuration function\n");

		return -1;
	}

	pcie_ops->wr_own_conf(pp, PCI_COMMAND, 4, 0x140);
	pcie_ops->rd_own_conf(pp, PCI_COMMAND, 4, &val);
	if ((val & 0xfff) == 0x140) {
		pr_info("PCIe DBI access Success.\n");
	} else {
		pr_info("PCIe DBI access Fail...\n");
		test_result = -1;
	}

	return test_result;
}

static int chk_pcie_link(struct exynos_pcie *exynos_pcie)
{
	int test_result = 0;
	struct exynos_pcie_ops *pcie_ops = &exynos_pcie->exynos_pcie_ops;
	u32 linkup_offset = exynos_pcie->linkup_offset;
	u32 val;

	if (!pcie_ops->poweron) {
		pr_err("Can't find PCIe poweron function\n");

		return -1;
	}

	pcie_ops->poweron(exynos_pcie->ch_num);
	val = exynos_elbi_read(exynos_pcie, linkup_offset) & 0x1f;
	if (val >= 0x0d && val <= 0x14) {
		pr_info("PCIe link test Success.\n");
	} else {
		pr_info("PCIe Link test Fail...\n");
		test_result = -1;
	}

	return test_result;
}

int exynos_pcie_dbg_unit_test(struct device *dev, struct exynos_pcie *exynos_pcie)
{
	int ret = 0;

	if (exynos_pcie->ssd_gpio < 0) {
		dev_warn(dev, "can't find ssd pin info. Need to check EP device power pin\n");
	} else {
		gpio_set_value(exynos_pcie->ssd_gpio, 1);
		mdelay(100);
	}

	if (exynos_pcie->wlan_gpio < 0) {
		dev_warn(dev, "can't find wlan pin info. Need to check EP device power pin\n");
	} else {
		gpio_direction_output(exynos_pcie->wlan_gpio, 0);
		gpio_set_value(exynos_pcie->wlan_gpio, 1);
		mdelay(100);
	}

	dev_info(dev, "1. Test PCIe LINK...\n");
	/* Test PCIe Link */
	if (chk_pcie_link(exynos_pcie)) {
		dev_info(dev, "PCIe UNIT test FAIL[1/6]!!!\n");
		ret = -1;

		goto done;
	}

	dev_info(dev, "2. Test DBI access...\n");
	/* Test PCIe DBI access */
	if (chk_dbi_access(exynos_pcie)) {
		dev_info(dev, "PCIe UNIT test FAIL[2/6]!!!\n");
		ret = -2;

		goto done;
	}

	dev_info(dev, "3. Test EP configuration access...\n");
	/* Test EP configuration access */
	if (chk_epconf_access(exynos_pcie)) {
		dev_info(dev, "PCIe UNIT test FAIL[3/6]!!!\n");
		ret = -3;

		goto done;
	}

	dev_info(dev, "4. Test EP Outbound memory region...\n");
	/* Test EP Outbound memory region */
	if (chk_epmem_access(exynos_pcie)) {
		dev_info(dev, "PCIe UNIT test FAIL[4/6]!!!\n");
		ret = -4;

		goto done;
	}

	dev_info(dev, "5. Test PCIe Link recovery...\n");
	/* PCIe Link recovery test */
	if (chk_link_recovery(exynos_pcie)) {
		dev_info(dev, "PCIe UNIT test FAIL[5/6]!!!\n");
		ret = -5;

		goto done;
	}

	dev_info(dev, "6. Test PCIe Dislink...\n");
	/* PCIe DisLink Test */
	if (chk_pcie_dislink(exynos_pcie)) {
		dev_info(dev, "PCIe UNIT test FAIL[6/6]!!!\n");
		ret = -6;

		goto done;
	}

done:
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pcie_dbg_unit_test);

int exynos_pcie_dbg_link_test(struct device *dev, struct exynos_pcie *exynos_pcie, int enable)
{
	struct exynos_pcie_ops *pcie_ops = &exynos_pcie->exynos_pcie_ops;
	int ret;

	dev_info(dev, "TEST PCIe %sLink Test\n", enable ? "" : "Dis");

	if (enable) {
		if (!pcie_ops->poweron) {
			pr_err("Can't find PCIe poweron function\n");

			return -1;
		}

		if (exynos_pcie->ssd_gpio < 0) {
			dev_err(dev, "can't find ssd pin info. Need to check EP device pwr pin\n");
		} else {
			gpio_set_value(exynos_pcie->ssd_gpio, 1);
			mdelay(100);
		}

		if (exynos_pcie->wlan_gpio < 0) {
			dev_err(dev, "can't find wlan pin info. Need to check EP device pwr pin\n");
		} else {
			dev_err(dev, "## make gpio direction to output\n");
			gpio_direction_output(exynos_pcie->wlan_gpio, 0);

			dev_err(dev, "## make gpio set high\n");
			gpio_set_value(exynos_pcie->wlan_gpio, 1);
			mdelay(100);
		}

		mdelay(100);
		ret = pcie_ops->poweron(exynos_pcie->ch_num);
	} else {
		if (!pcie_ops->poweroff) {
			pr_err("Can't find PCIe poweroff function\n");

			return -1;
		}
		pcie_ops->poweroff(exynos_pcie->ch_num);

		if (exynos_pcie->ssd_gpio < 0)
			dev_err(dev, "can't find ssd pin info. Need to check EP device pwr pin\n");
		else
			gpio_set_value(exynos_pcie->ssd_gpio, 0);

		if (exynos_pcie->wlan_gpio < 0)
			dev_err(dev, "can't find wlan pin info. Need to check EP device pwr pin\n");
		else
			gpio_set_value(exynos_pcie->wlan_gpio, 0);

		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pcie_dbg_link_test);
