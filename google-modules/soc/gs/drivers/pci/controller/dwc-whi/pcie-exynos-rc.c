// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe RC(RootComplex) controller driver for gs101 SoC
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Hongseock Kim <hongpooh.kim@samsung.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/panic_notifier.h>
#include <linux/pci.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/pm_qos.h>
#include <dt-bindings/pci/pci.h>
#include <linux/exynos-pci-noti.h>
#include <linux/exynos-pci-ctrl.h>
#include <linux/soc/samsung/exynos-smc.h>
#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
#include <soc/google/exynos-itmon.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/pm_runtime.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/irqdomain.h>

#if IS_ENABLED(CONFIG_CPU_IDLE)
#include <soc/google/exynos-powermode.h>
#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-cpupm.h>
#endif
#if IS_ENABLED(CONFIG_PM_DEVFREQ)
#include <soc/google/exynos_pm_qos.h>
#endif

#include <linux/shm_ipc.h>     /* to get Exynos Modem - MSI target addr. */
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
#define MODIFY_MSI_ADDR
#endif	/* CONFIG_LINK_DEVICE_PCIE */

#include <misc/logbuffer.h>
#include "pcie-designware.h"
#include "pcie-exynos-common.h"
#include "pcie-exynos-rc.h"
#include "pcie-exynos-dbg.h"

#include <linux/dma-map-ops.h>
#include <soc/google/s2mpu.h>
#include "../../../iommu/exynos-pcie-iommu-exp.h"

struct exynos_pcie g_pcie_rc[MAX_RC_NUM];
int pcie_is_linkup;	/* checkpatch: do not initialise globals to 0 */
static struct separated_msi_vector sep_msi_vec[MAX_RC_NUM][PCIE_MAX_SEPA_IRQ_NUM];
/* currnet_cnt & current_cnt2 for EOM test */
static int current_cnt;
static int current_cnt2;

static struct pci_dev *exynos_pcie_get_pci_dev(struct dw_pcie_rp *pp);

#if IS_ENABLED(CONFIG_PM_DEVFREQ)
static struct exynos_pm_qos_request exynos_pcie_int_qos[MAX_RC_NUM];
#endif

#define PMU_CHECK_MASK (BIT(2) | BIT(3) | BIT(10))
/*
 * PCIe channel0 is in the HSI1 block.
 * PCIe channel1 is in the HSI2 block.
 */
#define pcie_ch_to_hsi(ch_num)	((ch_num) + 1)
#if IS_ENABLED(CONFIG_GS_S2MPU) || IS_ENABLED(CONFIG_EXYNOS_PCIE_IOMMU)
static const struct dma_map_ops pcie_dma_ops;
static struct device fake_dma_dev;
#define to_pci_dev_from_dev(dev) container_of((dev), struct pci_dev, dev)

#define MODEM_CH_NUM    0
#define WIFI_CH_NUM     1
#define NUM_PMA_REGS	2048

#if IS_ENABLED(CONFIG_GS_S2MPU)
struct phys_mem {
	struct list_head list;
	phys_addr_t start;
	size_t size;
	unsigned char *refcnt_array;
};

#define ALIGN_SIZE	0x1000UL
#define REF_COUNT_UNDERFLOW 255


unsigned char *s2mpu_get_refcnt_ptr(struct exynos_pcie *exynos_pcie,
				    phys_addr_t addr)
{
	struct phys_mem *pm;

	/* Find the memory region the address falls into, then determine the
	 * offset into the corresponding refcnt_array.
	 */
	list_for_each_entry(pm, &exynos_pcie->phys_mem_list, list) {
		if (addr >= pm->start && addr < (pm->start + pm->size))
			return pm->refcnt_array
			       + (addr - pm->start) / ALIGN_SIZE;
	}
	return NULL;
}

void s2mpu_get_alignment(dma_addr_t addr, size_t size,
			 phys_addr_t *align_addr, size_t *align_size)
{
	*align_addr = ALIGN_DOWN(addr, ALIGN_SIZE);
	*align_size = ALIGN(addr - *align_addr + size, ALIGN_SIZE);
}

unsigned char s2mpu_get_and_modify(struct exynos_pcie *exynos_pcie,
				   unsigned char *refcnt_ptr,
				   bool inc)
{
	unsigned char val;

	val = (*refcnt_ptr);
	if (inc) {
		val++;
		*refcnt_ptr = val;
	} else {
		// Check for underflow. Should never happen.
		if (val == 0) {
			val = REF_COUNT_UNDERFLOW;
		} else {
			val--;
			*refcnt_ptr = val;
		}
	}

	return val;
}

void s2mpu_update_refcnt(struct device *dev,
			 dma_addr_t dma_addr, size_t size, bool incr,
			 enum dma_data_direction dir)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[WIFI_CH_NUM];
	phys_addr_t align_addr;
	size_t align_size;
	unsigned char *refcnt_ptr;
	int ret;
	unsigned char refcnt;
	unsigned long flags;

	/* Align to 4K as required by S2MPU */
	s2mpu_get_alignment(dma_addr, size, &align_addr, &align_size);

	/* Get the pointer into the ref count array used to keep track of
	 * the number of map and unmap calls for 4K blocks
	 * needed by the S2MPU.
	 * This is needed because there may be a few skbs within one 4K
	 * aligned block and we need to ensure that we don't prematurely
	 * disable access to this skb by calling s2mpu_close.
	 */
	refcnt_ptr = s2mpu_get_refcnt_ptr(exynos_pcie, align_addr);
	if (!refcnt_ptr) {
		dev_err(dev,
			"s2mpu refcnt_ptr failed addr=%pad, size=%zx\n",
			&dma_addr, size);
		return;
	}

	/* Put lock on while-loop to protect race condition on
	 * s2mpu_open/s2mpu_close and the case of align_size over 4K */
	spin_lock_irqsave(&exynos_pcie->s2mpu_refcnt_lock, flags);
	while (align_size != 0) {
		if (incr) {
			refcnt = s2mpu_get_and_modify(exynos_pcie, refcnt_ptr,
						      true);
			/* Note that this will open the memory with read/write
			 * permissions based on the first invocation. Subsequent
			 * read/write permissions will be ignored.
			 */
			if (refcnt == 1) {
				ret = s2mpu_open(exynos_pcie->s2mpu,
						 align_addr, ALIGN_SIZE, dir);
				if (ret) {
					dev_err(dev,
						"s2mpu_open failed addr=%pad, size=%zx\n",
						&dma_addr, size);
				}
			}
		} else {
			refcnt = s2mpu_get_and_modify(exynos_pcie, refcnt_ptr,
						      false);
			if (refcnt == REF_COUNT_UNDERFLOW) {
				dev_err(dev, "s2mpu error underflow in refcount\n");
				spin_unlock_irqrestore(&exynos_pcie->s2mpu_refcnt_lock, flags);
				return;
			}
			if (refcnt == 0) {
				ret = s2mpu_close(exynos_pcie->s2mpu,
						  align_addr, ALIGN_SIZE, dir);
				if (ret) {
					dev_err(dev,
						"s2mpu_close failed addr=%pad, size=%zx\n",
						&dma_addr, size);
				}
			}
		}
		align_addr += ALIGN_SIZE;
		align_size -= ALIGN_SIZE;
		refcnt_ptr++;
	}
	spin_unlock_irqrestore(&exynos_pcie->s2mpu_refcnt_lock, flags);
}
#endif

static int get_ch_num(struct pci_dev *epdev)
{
	int ch_num = WIFI_CH_NUM;
	if (epdev->vendor == PCI_VENDOR_ID_SAMSUNG)
		ch_num = MODEM_CH_NUM;

	return ch_num;
}

static void *pcie_dma_alloc_attrs(struct device *dev, size_t size,
				  dma_addr_t *dma_handle, gfp_t flag,
				  unsigned long attrs)
{
	void *cpu_addr;
	struct pci_dev *epdev = to_pci_dev_from_dev(dev);
	int ch_num = get_ch_num(epdev);
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	int ret;

	cpu_addr = dma_alloc_attrs(&fake_dma_dev, size,
				   dma_handle, flag, attrs);
	if (exynos_pcie->s2mpu) {
		s2mpu_update_refcnt(dev, *dma_handle, size, true, DMA_BIDIRECTIONAL);
	} else if (exynos_pcie->use_sysmmu) {
		ret = pcie_iommu_map(*dma_handle, *dma_handle, size,
				     DMA_BIDIRECTIONAL, pcie_ch_to_hsi(ch_num));
		if (ret != 0) {
			pr_err("Can't map PCIe SysMMU table!\n");
			dma_free_attrs(&fake_dma_dev, size,
				       cpu_addr, *dma_handle, attrs);
			return NULL;
		}
	}

	return cpu_addr;
}

static void pcie_dma_free_attrs(struct device *dev, size_t size,
				void *cpu_addr, dma_addr_t dma_addr,
				unsigned long attrs)
{
	struct pci_dev *epdev = to_pci_dev_from_dev(dev);
	int ch_num = get_ch_num(epdev);
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];

	dma_free_attrs(&fake_dma_dev, size, cpu_addr, dma_addr, attrs);
	if (exynos_pcie->s2mpu)
		s2mpu_update_refcnt(dev, dma_addr, size, false, DMA_BIDIRECTIONAL);
	else if (exynos_pcie->use_sysmmu)
		pcie_iommu_unmap(dma_addr, size, pcie_ch_to_hsi(ch_num));
}

static dma_addr_t pcie_dma_map_page(struct device *dev, struct page *page,
				    size_t offset, size_t size,
				    enum dma_data_direction dir,
				    unsigned long attrs)
{
	struct pci_dev *epdev = to_pci_dev_from_dev(dev);
	int ch_num = get_ch_num(epdev);
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	dma_addr_t dma_addr;
	int ret;

	dma_addr = dma_map_page_attrs(&fake_dma_dev, page, offset,
				      size, dir, attrs);
	if (exynos_pcie->s2mpu) {
		s2mpu_update_refcnt(dev, dma_addr, size, true, dir);
	} else if (exynos_pcie->use_sysmmu) {
		ret = pcie_iommu_map(dma_addr, dma_addr, size,
				     dir, pcie_ch_to_hsi(ch_num));
		if (ret != 0) {
			pr_err("DMA map - Can't map PCIe SysMMU table!!!\n");
			return 0;
		}
	}
	return dma_addr;
}

static void pcie_dma_unmap_page(struct device *dev, dma_addr_t dma_addr,
				size_t size, enum dma_data_direction dir,
				unsigned long attrs)
{
	struct pci_dev *epdev = to_pci_dev_from_dev(dev);
	int ch_num = get_ch_num(epdev);
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];

	dma_unmap_page_attrs(&fake_dma_dev, dma_addr, size, dir, attrs);

	if (exynos_pcie->s2mpu)
		s2mpu_update_refcnt(dev, dma_addr, size, false, dir);
	else if (exynos_pcie->use_sysmmu)
		pcie_iommu_unmap(dma_addr, size, pcie_ch_to_hsi(ch_num));
}

static const struct dma_map_ops pcie_dma_ops = {
	.alloc = pcie_dma_alloc_attrs,
	.free = pcie_dma_free_attrs,
	.mmap = NULL,
	.get_sgtable = NULL,
	.map_page = pcie_dma_map_page,
	.unmap_page = pcie_dma_unmap_page,
	.map_sg = NULL,
	.unmap_sg = NULL,
	.map_resource = NULL,
	.unmap_resource = NULL,
	.sync_single_for_cpu = NULL,
	.sync_single_for_device = NULL,
	.sync_sg_for_cpu = NULL,
	.sync_sg_for_device = NULL,
	.cache_sync = NULL,
	.dma_supported = NULL,
	.get_required_mask = NULL,
	.max_mapping_size = NULL,
	.get_merge_boundary = NULL,
};
#endif

static void save_pma_regs(struct exynos_pcie *exynos_pcie)
{
	struct device *dev = exynos_pcie->pci->dev;
	u32 *save;
	int i;

	if (!exynos_pcie)
		return;

	if (exynos_pcie->pma_regs_valid)
		return;

	save = exynos_pcie->pma_regs;
	for (i = 0; i < NUM_PMA_REGS; i++) {
		*save = exynos_phy_read(exynos_pcie, i * sizeof(u32));
		dev_dbg(dev, "i=%d val=0x%08x\n", i, *save);
		save++;
	}
	exynos_pcie->pma_regs_valid = true;
}

static void restore_pma_regs(struct exynos_pcie *exynos_pcie)
{
	struct device *dev = exynos_pcie->pci->dev;
	u32 *save;
	u32 val;
	int i;

	if (!exynos_pcie)
		return;

	if (!exynos_pcie->pma_regs_valid) {
		dev_err(dev, "pma regs are not valid, restore skipped\n");
		return;
	}

	save = exynos_pcie->pma_regs;
	for (i = 0; i < NUM_PMA_REGS; i++) {
		val = exynos_phy_read(exynos_pcie, i * sizeof(u32));
		if (val != *save)
			dev_err(dev, "diff i=%d curr_val=0x%08x save_val=0x%08x\n",
				i, val, *save);
		exynos_phy_write(exynos_pcie, *save, i * sizeof(u32));
		save++;
	}
}

static void exynos_pcie_phy_isolation(struct exynos_pcie *exynos_pcie, int val)
{
	struct device *dev = exynos_pcie->pci->dev;
	int ret;

	dev_dbg(dev, "PCIe PHY ISOLATION = %d\n", val);
	logbuffer_log(exynos_pcie->log, "Phy isolation val=%d", val);
	exynos_pcie->phy_control = val;
	ret = rmw_priv_reg(exynos_pcie->pmu_alive_pa +
			   exynos_pcie->pmu_offset, PCIE_PHY_CONTROL_MASK, val);
	if (ret)
		regmap_update_bits(exynos_pcie->pmureg,
				   exynos_pcie->pmu_offset,
				   PCIE_PHY_CONTROL_MASK, val);
}

void exynos_pcie_set_perst_gpio(int ch_num, bool on)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];

	if (exynos_pcie->ep_device_type == EP_SAMSUNG_MODEM) {
		pr_debug("%s: force setting for abnormal state\n", __func__);
		if (on) {
			gpio_set_value(exynos_pcie->perst_gpio, 1);
			pr_debug("%s: Set PERST to HIGH, gpio val = %d\n",
				 __func__, gpio_get_value(exynos_pcie->perst_gpio));
		} else {
			gpio_set_value(exynos_pcie->perst_gpio, 0);
			pr_debug("%s: Set PERST to LOW, gpio val = %d\n",
				 __func__, gpio_get_value(exynos_pcie->perst_gpio));
		}
	}
}
EXPORT_SYMBOL_GPL(exynos_pcie_set_perst_gpio);

void exynos_pcie_set_ready_cto_recovery(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;

	pr_info("[%s] ch_num:%d\n", __func__, ch_num);

	disable_irq(pp->irq);

	exynos_pcie_set_perst_gpio(ch_num, 0);

	/* LTSSM disable */
	exynos_elbi_write(exynos_pcie, PCIE_ELBI_LTSSM_DISABLE,
			PCIE_APP_LTSSM_ENABLE);
}
EXPORT_SYMBOL(exynos_pcie_set_ready_cto_recovery);

static ssize_t exynos_pcie_rc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, ">>>> PCIe Test <<<<\n");
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "0 : PCIe Unit Test\n");
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "1 : Link Test\n");
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "2 : DisLink Test\n");
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "14 : PCIe Hot Reset\n");

	return ret;
}

static ssize_t exynos_pcie_rc_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int op_num;
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);
	int ret = 0;

	if (sscanf(buf, "%10d", &op_num) == 0)
		return -EINVAL;

	if (exynos_pcie->use_phy_isol_con)
		exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_BYPASS);

	switch (op_num) {
	case 0:
		dev_info(dev, "## PCIe UNIT test START ##\n");
		ret = exynos_pcie_dbg_unit_test(dev, exynos_pcie);
		if (ret) {
			dev_err(dev, "PCIe UNIT test failed (%d)\n", ret);
			break;
		}
		dev_err(dev, "## PCIe UNIT test SUCCESS!!##\n");
		break;
	case 1:
		dev_info(dev, "## PCIe establish link test ##\n");
		ret = exynos_pcie_dbg_link_test(dev, exynos_pcie, 1);
		if (ret) {
			dev_err(dev, "PCIe establish link test failed (%d)\n", ret);
			break;
		}
		dev_err(dev, "PCIe establish link test success\n");
		break;
	case 2:
		dev_info(dev, "## PCIe dis-link test ##\n");
		ret = exynos_pcie_dbg_link_test(dev, exynos_pcie, 0);
		if (ret) {
			dev_err(dev, "PCIe dis-link test failed (%d)\n", ret);
			break;
		}
		dev_err(dev, "PCIe dis-link test success\n");
		break;
	case 3:
		dev_info(dev, "## LTSSM ##\n");
		ret = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP)
		      & PCIE_ELBI_LTSSM_STATE_MASK;
		dev_info(dev, "LTSSM :0x%x\n", ret);
		break;

	case 13:
		dev_info(dev, "%s: force perst setting\n", __func__);
		exynos_pcie_set_perst_gpio(1, 0);

		break;

	case 14:
		dev_info(dev, "%s:[hot reset] by pulsing app_init_rst(ch %d)\n",
				__func__, exynos_pcie->ch_num);
		dev_info(dev, "PM_POWER_STATE  = 0x%08x\n",
			 exynos_phy_pcs_read(exynos_pcie, 0x188));
		exynos_elbi_write(exynos_pcie, 0x1, APP_INIT_RST);
		return count;

	case 16:
		exynos_pcie_rc_set_outbound_atu(1, 0x47200000, 0x0, SZ_1M);
		break;

	case 17:
		dev_info(dev, "## Dump RC Registers ##\n");
		exynos_pcie_rc_register_dump(exynos_pcie->ch_num);
		break;

	default:
		dev_err(dev, "Unsupported Test Number(%d)...\n", op_num);
	}
	if (exynos_pcie->use_phy_isol_con)
		exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_ISOLATION);

	return count;
}

static DEVICE_ATTR(pcie_rc_test, S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
		   exynos_pcie_rc_show, exynos_pcie_rc_store);

static ssize_t exynos_pcie_eom1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);
	struct pcie_eom_result **eom_result = exynos_pcie->eom_result;
	struct device_node *np = dev->of_node;
	int len = 0;
	u32 test_cnt = 0;
	unsigned int lane_width = 1;
	int i = 0, ret;

	if (!eom_result) {
		len += snprintf(buf + len, PAGE_SIZE, "eom_result structure is NULL !!!\n");

		goto exit;
	}

	ret = of_property_read_u32(np, "num-lanes", &lane_width);
	if (ret)
		lane_width = 0;

	while (current_cnt != EOM_PH_SEL_MAX * EOM_DEF_VREF_MAX) {
		len += snprintf(buf + len, PAGE_SIZE,
				"%u %u %lu\n",
				eom_result[i][current_cnt].phase,
				eom_result[i][current_cnt].vref,
				eom_result[i][current_cnt].err_cnt);
		current_cnt++;
		test_cnt++;
		if (test_cnt == 100)
			break;
	}

	if (current_cnt == EOM_PH_SEL_MAX * EOM_DEF_VREF_MAX)
		current_cnt = 0;

exit:
	return len;
}

static ssize_t exynos_pcie_eom1_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int op_num;
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	if (sscanf(buf, "%10d", &op_num) == 0)
		return -EINVAL;
	switch (op_num) {
	case 0:
		if (exynos_pcie->phy_ops.phy_eom)
			exynos_pcie->phy_ops.phy_eom(dev, exynos_pcie->phy_base);

		/* reset the counter before start eom_show() func. */
		current_cnt = 0;
		current_cnt2 = 0;

		break;
	}

	return count;
}

static DEVICE_ATTR(eom1, S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
		   exynos_pcie_eom1_show, exynos_pcie_eom1_store);

static ssize_t exynos_pcie_eom2_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t count)
{
	/* prevent to print kerenl warning message
	 * eom1_store function do all operation to get eom data
	 */

	return count;
}

static ssize_t exynos_pcie_eom2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);
	struct pcie_eom_result **eom_result = exynos_pcie->eom_result;
	struct device_node *np = dev->of_node;
	int len = 0;
	u32 test_cnt = 0;
	unsigned int lane_width = 1;
	int i = 1, ret;

	if (!eom_result) {
		len += snprintf(buf + len, PAGE_SIZE, "eom_result structure is NULL!!\n");

		goto exit;
	}

	ret = of_property_read_u32(np, "num-lanes", &lane_width);
	if (ret) {
		lane_width = 0;
		len += snprintf(buf + len, PAGE_SIZE,
				"can't get num of lanes!!\n");
		goto exit;
	}

	if (lane_width == 1) {
		len += snprintf(buf + len, PAGE_SIZE,
				"EOM2NULL\n");
		goto exit;
	}

	while (current_cnt2 != EOM_PH_SEL_MAX * EOM_DEF_VREF_MAX) {
		len += snprintf(buf + len, PAGE_SIZE,
				"%u %u %lu\n",
				eom_result[i][current_cnt2].phase,
				eom_result[i][current_cnt2].vref,
				eom_result[i][current_cnt2].err_cnt);
		current_cnt2++;
		test_cnt++;
		if (test_cnt == 100)
			break;
	}

	if (current_cnt2 == EOM_PH_SEL_MAX * EOM_DEF_VREF_MAX)
		current_cnt2 = 0;

exit:
	return len;
}

static DEVICE_ATTR(eom2, S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
		   exynos_pcie_eom2_show, exynos_pcie_eom2_store);

static ssize_t l12_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int  ret;
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	ret = scnprintf(buf, PAGE_SIZE, "l1ss_ctrl_id_state = 0x%08x\n",
			exynos_pcie->l1ss_ctrl_id_state);
	ret += scnprintf(buf, PAGE_SIZE, "LTSSM: 0x%08x, PM_STATE = 0x%08x\n",
			 exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP),
			 exynos_phy_pcs_read(exynos_pcie, PM_POWER_STATE));
	return ret;
}

static ssize_t l12_state_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);
	int ch_num = exynos_pcie->ch_num;
	int enable;

	if (sscanf(buf, "%10d", &enable) == 0)
		return -EINVAL;

	if (enable == 1) {
		dev_info(dev, "L1.2 Enable....on PCIe_ch%d\n", ch_num);
		exynos_pcie_rc_l1ss_ctrl(1, PCIE_L1SS_CTRL_TEST, ch_num);
	} else if (enable == 0) {
		dev_info(dev, "L1.2 Disable....on PCIe_ch%d\n", ch_num);
		exynos_pcie_rc_l1ss_ctrl(0, PCIE_L1SS_CTRL_TEST, ch_num);
	} else {
		dev_err(dev, "Value needs to be 0 or 1\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t link_speed_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "GEN%d\n", exynos_pcie->max_link_speed);
}

static ssize_t link_speed_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int link_speed;
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	if (sscanf(buf, "%10d", &link_speed) == 0)
		return -EINVAL;

	if (link_speed < 1 || link_speed > 3) {
		dev_info(dev, "Value needs to be between 1-3\n");
		return -EINVAL;
	}

	dev_info(dev, "Change Link Speed: Target Link Speed = GEN%d\n", link_speed);
	exynos_pcie->max_link_speed = link_speed;

	return count;
}

static ssize_t link_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u32 val;
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	if (exynos_pcie->phy_control == PCIE_PHY_ISOLATION ||
	    exynos_pcie->state != STATE_LINK_UP) {
		val = L2;   /* Not linked is equivalent to L2 */
		goto exit;
	}

	val = exynos_phy_pcs_read(exynos_pcie, PM_POWER_STATE);
	val &= PM_STATE_MASK;

	switch (val) {
	case 0:
		val = L0;
		break;
	case 1:
		val = L0S;
		break;
	case 2:
		val = L1;
		break;
	case 3:
		val = L2V;
		break;
	case 4:
		val = L2;
		break;
	case 5:
		val = L11;
		break;
	case 6:
		val = L12;
		break;
	default:
		val = UNKNOWN;
	}

exit:
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d\n", val);

	return ret;
}

static u64 power_stats_get_ts(void)
{
	return ktime_to_ms(ktime_get_boottime());
}

static void power_stats_init(struct exynos_pcie *pcie)
{
	pcie->link_up.count = 0;
	pcie->link_up.duration = 0;
	pcie->link_up.last_entry_ts = 0;
	pcie->link_down.count = 1;  // since system starts with link_down
	pcie->link_down.duration = 0;
	pcie->link_down.last_entry_ts = 0;
}

static void power_stats_update_up(struct exynos_pcie *pcie)
{
	u64 current_ts;
	unsigned long flags;

	spin_lock_irqsave(&pcie->power_stats_lock, flags);

	current_ts = power_stats_get_ts();

	pcie->link_up.count++;
	pcie->link_up.last_entry_ts = current_ts;

	pcie->link_down.duration += current_ts -
				    pcie->link_down.last_entry_ts;

	spin_unlock_irqrestore(&pcie->power_stats_lock, flags);
}

static void power_stats_update_down(struct exynos_pcie *pcie)
{
	u64 current_ts;
	unsigned long flags;

	spin_lock_irqsave(&pcie->power_stats_lock, flags);
	current_ts = power_stats_get_ts();

	pcie->link_down.count++;
	pcie->link_down.last_entry_ts = current_ts;

	pcie->link_up.duration += current_ts -
				  pcie->link_up.last_entry_ts;
	spin_unlock_irqrestore(&pcie->power_stats_lock, flags);
}

static ssize_t power_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct exynos_pcie *pcie = dev_get_drvdata(dev);
	u64 current_ts;
	u64 link_up_delta = 0;
	u64 link_down_delta = 0;
	struct power_stats link_up_copy;
	struct power_stats link_down_copy;
	enum exynos_pcie_state state_copy;
	unsigned long flags;

	spin_lock_irqsave(&pcie->power_stats_lock, flags);
	memcpy(&link_up_copy, &pcie->link_up, sizeof(struct power_stats));
	memcpy(&link_down_copy, &pcie->link_down, sizeof(struct power_stats));
	state_copy = pcie->state;
	current_ts = power_stats_get_ts();
	spin_unlock_irqrestore(&pcie->power_stats_lock, flags);

	if (state_copy == STATE_LINK_UP || state_copy == STATE_LINK_DOWN_TRY)
		link_up_delta = current_ts - link_up_copy.last_entry_ts;

	if (state_copy == STATE_LINK_DOWN || state_copy == STATE_LINK_UP_TRY)
		link_down_delta = current_ts - link_down_copy.last_entry_ts;

	ret = scnprintf(buf + ret, PAGE_SIZE - ret, "Version: 1\n");

	ret += PRINT_STATUS(buf, ret, "up", link_up_copy.count,
			    link_up_copy.duration + link_up_delta,
			    link_up_copy.last_entry_ts);

	ret += PRINT_STATUS(buf, ret, "down", link_down_copy.count,
			    link_down_copy.duration + link_down_delta,
			    link_down_copy.last_entry_ts);

	return ret;
}

static DEVICE_ATTR_RW(l12_state);
static DEVICE_ATTR_RW(link_speed);
static DEVICE_ATTR_RO(link_state);
static DEVICE_ATTR_RO(power_stats);

/* percentage (0-100) to weight new datapoints in moving average */
#define NEW_DATA_AVERAGE_WEIGHT  10
static void link_stats_init(struct exynos_pcie *pcie)
{
	pcie->link_stats.link_down_irq_count = 0;
	pcie->link_stats.link_down_irq_count_reported = 0;
	pcie->link_stats.cmpl_timeout_irq_count = 0;
	pcie->link_stats.cmpl_timeout_irq_count_reported = 0;
	pcie->link_stats.link_up_failure_count = 0;
	pcie->link_stats.link_up_failure_count_reported = 0;
	pcie->link_stats.link_recovery_failure_count = 0;
	pcie->link_stats.link_recovery_failure_count_reported = 0;
	pcie->link_stats.pll_lock_time_avg = 0;
	pcie->link_stats.link_up_time_avg = 0;
}

static void link_stats_log_pll_lock(struct exynos_pcie *pcie, u32 pll_lock_time)
{
	pcie->link_stats.pll_lock_time_avg =
	    DIV_ROUND_CLOSEST(pcie->link_stats.pll_lock_time_avg *
			      (100 - NEW_DATA_AVERAGE_WEIGHT) +
			      pll_lock_time * NEW_DATA_AVERAGE_WEIGHT, 100);
}

static void link_stats_log_link_up(struct exynos_pcie *pcie, u32 link_up_time)
{
	pcie->link_stats.link_up_time_avg =
	    DIV_ROUND_CLOSEST(pcie->link_stats.link_up_time_avg *
			      (100 - NEW_DATA_AVERAGE_WEIGHT) +
			      link_up_time * NEW_DATA_AVERAGE_WEIGHT, 100);
}

static int check_exynos_pcie_reg_status(struct exynos_pcie *exynos_pcie,
					void __iomem *base, int offset,
					u32 mask, u32 mask_value, int *cnt)
{
	int i;
	bool status = false;
	u32 value;
	ktime_t timestamp = ktime_get();

	for (i = 0; i < 10000; i++) {
		udelay(1);
		value = readl(base + offset);

		status = (value & mask) == mask_value;
		if (status)
			break;

	}
	*cnt = i;

	timestamp = ktime_sub(ktime_get(), timestamp);
	dev_dbg(exynos_pcie->pci->dev, "elapsed time: %lld uS\n", ktime_to_us(timestamp));
	logbuffer_log(exynos_pcie->log, "elapsed time: %lld uS", ktime_to_us(timestamp));

	if (status) {
		dev_dbg(exynos_pcie->pci->dev, "status %#x passed cnt=%d\n", offset, i);
		logbuffer_log(exynos_pcie->log, "status %#x passed cnt=%d", offset, i);
	} else {
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
			       "status %#x=%#x failed", offset, value);
	}

	return status;
}

static ssize_t link_down_irqs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct exynos_pcie *pcie = dev_get_drvdata(dev);
	u32 value = pcie->link_stats.link_down_irq_count;

	/* report delta since last write */
	value -= pcie->link_stats.link_down_irq_count_reported;

	return sysfs_emit(buf, "%d\n", value);
}

static ssize_t link_down_irqs_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;
	u32 value, delta;
	struct exynos_pcie *pcie = dev_get_drvdata(dev);

	/* Writing a value marks those as reported */
	ret = kstrtouint(buf, 10, &value);
	if (ret < 0)
		return ret;

	delta = pcie->link_stats.link_down_irq_count -
		pcie->link_stats.link_down_irq_count_reported;

	if (value > delta) {
		dev_info(dev, "Value needs to be <= %d\n", delta);
		return -EINVAL;
	}

	pcie->link_stats.link_down_irq_count_reported += value;

	return count;
}

static ssize_t complete_timeout_irqs_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct exynos_pcie *pcie = dev_get_drvdata(dev);
	u32 value = pcie->link_stats.cmpl_timeout_irq_count;

	/* report delta since last sysfs write */
	value -= pcie->link_stats.cmpl_timeout_irq_count_reported;

	return sysfs_emit(buf, "%d\n", value);
}

static ssize_t complete_timeout_irqs_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int ret;
	u32 value, delta;
	struct exynos_pcie *pcie = dev_get_drvdata(dev);

	/* Writing a value marks those as reported */
	ret = kstrtouint(buf, 10, &value);
	if (ret < 0)
		return ret;

	delta = pcie->link_stats.cmpl_timeout_irq_count -
		pcie->link_stats.cmpl_timeout_irq_count_reported;

	if (value > delta) {
		dev_info(dev, "Value needs to be <= %d\n", delta);
		return -EINVAL;
	}

	pcie->link_stats.cmpl_timeout_irq_count_reported += value;

	return count;
}


static ssize_t link_up_failures_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct exynos_pcie *pcie = dev_get_drvdata(dev);
	u32 value = pcie->link_stats.link_up_failure_count;

	/* report delta since last write */
	value -= pcie->link_stats.link_up_failure_count_reported;

	return sysfs_emit(buf, "%d\n", value);
}

static ssize_t link_up_failures_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	u32 value, delta;
	struct exynos_pcie *pcie = dev_get_drvdata(dev);

	/* Writing a value marks those as reported */
	ret = kstrtouint(buf, 10, &value);
	if (ret < 0)
		return ret;

	delta = pcie->link_stats.link_up_failure_count -
		pcie->link_stats.link_up_failure_count_reported;

	if (value > delta) {
		dev_info(dev, "Value needs to be <= %d\n", delta);
		return -EINVAL;
	}

	pcie->link_stats.link_up_failure_count_reported += value;

	return count;
}

static ssize_t link_recovery_failures_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct exynos_pcie *pcie = dev_get_drvdata(dev);
	u32 value = pcie->link_stats.link_recovery_failure_count;

	/* report delta since last write */
	value -= pcie->link_stats.link_recovery_failure_count_reported;

	return sysfs_emit(buf, "%d\n", value);
}

static ssize_t link_recovery_failures_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;
	u32 value, delta;
	struct exynos_pcie *pcie = dev_get_drvdata(dev);

	/* Writing a value marks those as reported */
	ret = kstrtouint(buf, 10, &value);
	if (ret < 0)
		return ret;

	delta = pcie->link_stats.link_recovery_failure_count -
		pcie->link_stats.link_recovery_failure_count_reported;

	if (value > delta) {
		dev_info(dev, "Value needs to be <= %d\n", delta);
		return -EINVAL;
	}

	pcie->link_stats.link_recovery_failure_count_reported += value;

	return count;
}

static ssize_t pll_lock_average_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct exynos_pcie *pcie = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", pcie->link_stats.pll_lock_time_avg);
}

static ssize_t link_up_average_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct exynos_pcie *pcie = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", pcie->link_stats.link_up_time_avg);
}

static DEVICE_ATTR_RW(link_down_irqs);
static DEVICE_ATTR_RW(complete_timeout_irqs);
static DEVICE_ATTR_RW(link_up_failures);
static DEVICE_ATTR_RW(link_recovery_failures);
static DEVICE_ATTR_RO(pll_lock_average);
static DEVICE_ATTR_RO(link_up_average);

static struct attribute *link_stats_attrs[] = {
	&dev_attr_link_down_irqs.attr,
	&dev_attr_complete_timeout_irqs.attr,
	&dev_attr_link_up_failures.attr,
	&dev_attr_link_recovery_failures.attr,
	&dev_attr_pll_lock_average.attr,
	&dev_attr_link_up_average.attr,
	NULL,
};

static const struct attribute_group link_stats_group = {
	.attrs = link_stats_attrs,
	.name = "link_stats",
};

static inline int create_pcie_sys_file(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct pci_dev *pdev = to_pci_dev_from_dev(dev);
	int ret;
	int num_lane;

	ret = device_create_file(dev, &dev_attr_pcie_rc_test);
	if (ret) {
		dev_err(dev, "couldn't create device file for test(%d)\n", ret);

		return ret;
	}

	ret = of_property_read_u32(np, "num-lanes", &num_lane);
	if (ret)
		num_lane = 0;

	ret = device_create_file(dev, &dev_attr_eom1);
	if (ret) {
		dev_err(dev, "couldn't create device file for eom(%d)\n", ret);
		return ret;
	}

	if (num_lane > 0) {
		ret = device_create_file(dev, &dev_attr_eom2);
		if (ret) {
			dev_err(dev, "couldn't create device file for eom(%d)\n", ret);
			return ret;
		}
	}

	ret = device_create_file(dev, &dev_attr_l12_state);
	if (ret) {
		dev_err(dev, "couldn't create device file for l12_state(%d)\n", ret);
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_link_speed);
	if (ret) {
		dev_err(dev, "couldn't create device file for link_speed(%d)\n", ret);
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_link_state);
	if (ret) {
		dev_err(dev, "couldn't create device file for linkst(%d)\n", ret);
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_power_stats);
	if (ret) {
		dev_err(dev, "couldn't create device file for power_stats(%d)\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &link_stats_group);
	if (ret) {
		dev_err(dev, "couldn't create sysfs group for link_stats(%d)\n", ret);
		return ret;
	}

	return 0;
}

static inline void remove_pcie_sys_file(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev_from_dev(dev);

	device_remove_file(dev, &dev_attr_pcie_rc_test);
	device_remove_file(dev, &dev_attr_l12_state);
	device_remove_file(dev, &dev_attr_link_speed);
	device_remove_file(dev, &dev_attr_link_state);
	device_remove_file(dev, &dev_attr_power_stats);
	sysfs_remove_group(&pdev->dev.kobj, &link_stats_group);
}

static int exynos_pcie_rc_clock_enable(struct dw_pcie_rp *pp, int enable)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct exynos_pcie_clks	*clks = &exynos_pcie->clks;
	int i;
	int ret = 0;

	if (enable) {
		for (i = 0; i < exynos_pcie->pcie_clk_num; i++)
			ret = clk_prepare_enable(clks->pcie_clks[i]);
	} else {
		for (i = 0; i < exynos_pcie->pcie_clk_num; i++)
			clk_disable_unprepare(clks->pcie_clks[i]);
	}

	return ret;
}

static int exynos_pcie_rc_phy_clock_enable(struct dw_pcie_rp *pp, int enable)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct exynos_pcie_clks	*clks = &exynos_pcie->clks;
	int i;
	int ret = 0;

	if (enable) {
		for (i = 0; i < exynos_pcie->phy_clk_num; i++)
			ret = clk_prepare_enable(clks->phy_clks[i]);
	} else {
		for (i = 0; i < exynos_pcie->phy_clk_num; i++)
			clk_disable_unprepare(clks->phy_clks[i]);
	}

	return ret;
}

void exynos_pcie_rc_print_link_history(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	u32 history;
	int i;

	for (i = 31; i >= 0; i--) {
		history = exynos_elbi_read(exynos_pcie,
					   PCIE_HISTORY_REG(i));

		dev_info(dev, "LTSSM: 0x%02x, L1sub: 0x%x, D state: 0x%x\n",
				LTSSM_STATE(history),
				L1SUB_STATE(history),
				PM_DSTATE(history));
	}
}

static int exynos_pcie_rc_rd_own_conf(struct dw_pcie_rp *pp, int where, int size, u32 *val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	int is_linked = 0;
	int ret = 0;
	u32 __maybe_unused reg_val;
	unsigned long flags;

	if (exynos_pcie->phy_control == PCIE_PHY_ISOLATION) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	spin_lock_irqsave(&exynos_pcie->reg_lock, flags);

	if (exynos_pcie->state == STATE_LINK_UP)
		is_linked = 1;

	if (is_linked == 0) {
		exynos_pcie_rc_clock_enable(pp, PCIE_ENABLE_CLOCK);
		exynos_pcie_rc_phy_clock_enable(pp, PCIE_ENABLE_CLOCK);

		if (exynos_pcie->phy_ops.phy_check_rx_elecidle)
			exynos_pcie->phy_ops.phy_check_rx_elecidle(exynos_pcie->phy_pcs_base,
								   IGNORE_ELECIDLE,
								   exynos_pcie->ch_num);
	}

	ret = dw_pcie_read(exynos_pcie->rc_dbi_base + (where), size, val);

	if (is_linked == 0) {
		if (exynos_pcie->phy_ops.phy_check_rx_elecidle)
			exynos_pcie->phy_ops.phy_check_rx_elecidle(exynos_pcie->phy_pcs_base,
								   ENABLE_ELECIDLE,
								   exynos_pcie->ch_num);

		exynos_pcie_rc_phy_clock_enable(pp, PCIE_DISABLE_CLOCK);
		exynos_pcie_rc_clock_enable(pp, PCIE_DISABLE_CLOCK);
	}
	spin_unlock_irqrestore(&exynos_pcie->reg_lock, flags);

	return ret;
}

static int exynos_pcie_rc_wr_own_conf(struct dw_pcie_rp *pp, int where, int size, u32 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	int is_linked = 0;
	int ret = 0;
	u32 __maybe_unused reg_val;
	unsigned long flags;

	if (exynos_pcie->phy_control == PCIE_PHY_ISOLATION)
		return PCIBIOS_DEVICE_NOT_FOUND;

	spin_lock_irqsave(&exynos_pcie->reg_lock, flags);

	if (exynos_pcie->state == STATE_LINK_UP)
		is_linked = 1;

	if (is_linked == 0) {
		exynos_pcie_rc_clock_enable(pp, PCIE_ENABLE_CLOCK);
		exynos_pcie_rc_phy_clock_enable(pp, PCIE_ENABLE_CLOCK);

		if (exynos_pcie->phy_ops.phy_check_rx_elecidle)
			exynos_pcie->phy_ops.phy_check_rx_elecidle(exynos_pcie->phy_pcs_base,
								   IGNORE_ELECIDLE,
								   exynos_pcie->ch_num);
	}

	/* If secure ATU then make SMC call, since only TFA has write access */
	if (exynos_pcie->use_secure_atu && where == SECURE_ATU_ENABLE)
		ret = exynos_smc(SMC_SECURE_ATU_SETUP, 0, 0, 0);
	else
		ret = dw_pcie_write(exynos_pcie->rc_dbi_base + (where), size, val);

	if (is_linked == 0) {
		if (exynos_pcie->phy_ops.phy_check_rx_elecidle)
			exynos_pcie->phy_ops.phy_check_rx_elecidle(exynos_pcie->phy_pcs_base,
								   ENABLE_ELECIDLE,
								   exynos_pcie->ch_num);

		exynos_pcie_rc_phy_clock_enable(pp, PCIE_DISABLE_CLOCK);
		exynos_pcie_rc_clock_enable(pp, PCIE_DISABLE_CLOCK);
	}

	spin_unlock_irqrestore(&exynos_pcie->reg_lock, flags);
	return ret;
}

static void exynos_pcie_rc_prog_viewport_cfg0(struct dw_pcie_rp *pp, u32 busdev)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	/* ATU needs to be written just once during establish link */

	if (exynos_pcie->atu_ok)
		return;

	exynos_pcie->atu_ok = 1;

	if (exynos_pcie->use_secure_atu) {
		exynos_pcie_rc_wr_own_conf(pp, SECURE_ATU_ENABLE, 0, 0);
		return;
	}

	/* Program viewport 0 : OUTBOUND : CFG0 */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LOWER_BASE_OUTBOUND0, 4, pp->cfg0_base);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_UPPER_BASE_OUTBOUND0, 4, (pp->cfg0_base >> 32));
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LIMIT_OUTBOUND0, 4,
				   pp->cfg0_base + pp->cfg0_size - 1);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LOWER_TARGET_OUTBOUND0, 4, busdev);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_UPPER_TARGET_OUTBOUND0, 4, 0);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_CR1_OUTBOUND0, 4, PCIE_ATU_TYPE_CFG0);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_CR2_OUTBOUND0, 4, EXYNOS_PCIE_ATU_ENABLE);
}

static void exynos_pcie_rc_prog_viewport_mem_outbound(struct dw_pcie_rp *pp)
{
	struct resource_entry *entry =
		resource_list_first_type(&pp->bridge->windows, IORESOURCE_MEM);
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	/* Secure ATU is already setup by prog_viewport_cfg0, so skip it here */
	if (exynos_pcie->use_secure_atu)
		return;

	/* Program viewport 0 : OUTBOUND : MEM */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_CR1_OUTBOUND1, 4, EXYNOS_PCIE_ATU_TYPE_MEM);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LOWER_BASE_OUTBOUND1, 4, entry->res->start);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_UPPER_BASE_OUTBOUND1, 4, (entry->res->start >> 32));
	/* exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LIMIT_OUTBOUND1, 4,
	 *			      entry->res->start +
	 *			      resource_size(entry->res) - 1);
	 */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LIMIT_OUTBOUND1, 4, entry->res->start + SZ_2M - 1);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LOWER_TARGET_OUTBOUND1, 4,
				   entry->res->start - entry->offset);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_UPPER_TARGET_OUTBOUND1, 4,
				   upper_32_bits(entry->res->start -
						 entry->offset));
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_CR2_OUTBOUND1, 4, EXYNOS_PCIE_ATU_ENABLE);
}

int exynos_pcie_rc_set_bar(int ch_num, u32 bar_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct pci_dev *ep_pci_dev;
	u32 val;

	pr_info("%s: +++\n", __func__);

	if (exynos_pcie->state == STATE_LINK_UP) {
		ep_pci_dev = exynos_pcie_get_pci_dev(pp);
	} else {
		pr_info("%s: PCIe link is not up\n", __func__);

		return -EPIPE;
	}

	/* EP BAR setup */
	ep_pci_dev->resource[bar_num].start =
		exynos_pcie->btl_target_addr + exynos_pcie->btl_offset;
	ep_pci_dev->resource[bar_num].end =
		exynos_pcie->btl_target_addr + exynos_pcie->btl_offset + exynos_pcie->btl_size;
	ep_pci_dev->resource[bar_num].flags = 0x82000000;
	if (pci_assign_resource(ep_pci_dev, bar_num) != 0)
		pr_warn("%s: failed to assign PCI resource for %i\n", __func__, bar_num);

	pci_read_config_dword(ep_pci_dev, PCI_BASE_ADDRESS_0 + (bar_num * 0x4), &val);
	pr_info("%s: Check EP BAR[%d] = 0x%x\n", __func__, bar_num, val);

	pr_info("%s: ---\n", __func__);
	return 0;
}

int exynos_pcie_rc_set_outbound_atu(int ch_num, u32 target_addr, u32 offset, u32 size)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct resource_entry *entry =
		resource_list_first_type(&pp->bridge->windows, IORESOURCE_MEM);
	u32 val;
	int ret;

	pr_info("%s: +++\n", __func__);

	exynos_pcie->btl_target_addr = target_addr;
	exynos_pcie->btl_offset = offset;
	exynos_pcie->btl_size = size;

	pr_info("%s: target_addr = 0x%x, offset = 0x%x, size = 0x%x\n", __func__,
		exynos_pcie->btl_target_addr,
		exynos_pcie->btl_offset,
		exynos_pcie->btl_size);

	/* Only for BTL */
	/* 0x1420_0000 ~ (size -1) */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_CR1_OUTBOUND2, 4, EXYNOS_PCIE_ATU_TYPE_MEM);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LOWER_BASE_OUTBOUND2, 4, entry->res->start + SZ_2M);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_UPPER_BASE_OUTBOUND2, 4,
				   ((entry->res->start + SZ_2M) >> 32));
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LIMIT_OUTBOUND2, 4,
				   entry->res->start + SZ_2M + exynos_pcie->btl_size - 1);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_LOWER_TARGET_OUTBOUND2, 4,
				   exynos_pcie->btl_target_addr + exynos_pcie->btl_offset);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_UPPER_TARGET_OUTBOUND2, 4, 0);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_ATU_CR2_OUTBOUND2, 4, EXYNOS_PCIE_ATU_ENABLE);

	exynos_pcie_rc_rd_own_conf(pp, PCIE_ATU_CR1_OUTBOUND2, 4, &val);
	pr_info("%s:  PCIE_ATU_CR1_OUTBOUND2(0x400) = 0x%x\n", __func__, val);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_ATU_LOWER_BASE_OUTBOUND2, 4, &val);
	pr_info("%s:  PCIE_ATU_LOWER_BASE_OUTBOUND2(0x408) = 0x%x\n", __func__, val);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_ATU_UPPER_BASE_OUTBOUND2, 4, &val);
	pr_info("%s:  PCIE_ATU_UPPER_BASE_OUTBOUND2(0x40C) = 0x%x\n", __func__, val);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_ATU_LIMIT_OUTBOUND2, 4, &val);
	pr_info("%s:  PCIE_ATU_LIMIT_OUTBOUND2(0x410) = 0x%x\n", __func__, val);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_ATU_LOWER_TARGET_OUTBOUND2, 4, &val);
	pr_info("%s:  PCIE_ATU_LOWER_TARGET_OUTBOUND2(0x414) = 0x%x\n", __func__, val);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_ATU_UPPER_TARGET_OUTBOUND2, 4, &val);
	pr_info("%s:  PCIE_ATU_UPPER_TARGET_OUTBOUND2(0x418) = 0x%x\n", __func__, val);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_ATU_CR2_OUTBOUND2, 4, &val);
	pr_info("%s:  PCIE_ATU_CR2_OUTBOUND2(0x404) = 0x%x\n", __func__, val);

	ret = exynos_pcie_rc_set_bar(ch_num, 2);

	pr_info("%s: ---\n", __func__);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_set_outbound_atu);

static int exynos_pcie_rc_rd_other_conf(struct dw_pcie_rp *pp, struct pci_bus *bus, u32 devfn,
					int where, int size, u32 *val)
{
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;

	busdev = EXYNOS_PCIE_ATU_BUS(bus->number) | EXYNOS_PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 EXYNOS_PCIE_ATU_FUNC(PCI_FUNC(devfn));

	cpu_addr = pp->cfg0_base;
	cfg_size = pp->cfg0_size;
	va_cfg_base = pp->va_cfg0_base;
	/* setup ATU for cfg/mem outbound */
	exynos_pcie_rc_prog_viewport_cfg0(pp, busdev);

	return dw_pcie_read(va_cfg_base + where, size, val);
}

static int exynos_pcie_rc_wr_other_conf(struct dw_pcie_rp *pp, struct pci_bus *bus, u32 devfn,
					int where, int size, u32 val)
{
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;

	busdev = EXYNOS_PCIE_ATU_BUS(bus->number) | EXYNOS_PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 EXYNOS_PCIE_ATU_FUNC(PCI_FUNC(devfn));

	cpu_addr = pp->cfg0_base;
	cfg_size = pp->cfg0_size;
	va_cfg_base = pp->va_cfg0_base;
	/* setup ATU for cfg/mem outbound */
	exynos_pcie_rc_prog_viewport_cfg0(pp, busdev);

	return dw_pcie_write(va_cfg_base + where, size, val);
}

static int exynos_pcie_rc_rd_other_conf_new(struct pci_bus *bus,
					    unsigned int devfn,
					    int where, int size, u32 *val)
{
	struct dw_pcie_rp *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	int ret = 0;

	if (exynos_pcie->state == STATE_LINK_UP)
		ret = exynos_pcie_rc_rd_other_conf(pp, bus, devfn, where, size, val);
	return ret;
}

static int exynos_pcie_rc_wr_other_conf_new(struct pci_bus *bus,
					    unsigned int devfn,
					    int where, int size, u32 val)
{
	struct dw_pcie_rp *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	int ret = 0;

	if (exynos_pcie->state == STATE_LINK_UP)
		ret = exynos_pcie_rc_wr_other_conf(pp, bus, devfn, where, size, val);
	return ret;
}

static int exynos_pcie_rc_rd_own_conf_new(struct pci_bus *bus,
					  unsigned int devfn,
					  int where, int size, u32 *val)
{
	struct dw_pcie_rp *pp = bus->sysdata;

	if (PCI_SLOT(devfn) > 0) {
		*val = ~0;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	return exynos_pcie_rc_rd_own_conf(pp, where, size, val);
}

static int exynos_pcie_rc_wr_own_conf_new(struct pci_bus *bus,
					  unsigned int devfn,
					  int where, int size, u32 val)
{
	struct dw_pcie_rp *pp = bus->sysdata;

	if (PCI_SLOT(devfn) > 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	return exynos_pcie_rc_wr_own_conf(pp, where, size, val);
}

static struct pci_ops exynos_pcie_rc_child_ops = {
	.read = exynos_pcie_rc_rd_other_conf_new,
	.write = exynos_pcie_rc_wr_other_conf_new
};

static struct pci_ops exynos_pcie_rc_root_ops = {
	.read = exynos_pcie_rc_rd_own_conf_new,
	.write = exynos_pcie_rc_wr_own_conf_new
};

u32 exynos_pcie_rc_read_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size)
{
	struct dw_pcie_rp *pp = &pci->pp;
	u32 val;

	exynos_pcie_rc_rd_own_conf(pp, reg, size, &val);

	return val;
}

void exynos_pcie_rc_write_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size,
			      u32 val)
{
	struct dw_pcie_rp *pp = &pci->pp;

	exynos_pcie_rc_wr_own_conf(pp, reg, size, val);
}

static int exynos_pcie_rc_link_up(struct dw_pcie *pci)
{
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	u32 val;

	if (exynos_pcie->state != STATE_LINK_UP)
		return 0;

	val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP) & PCIE_ELBI_LTSSM_STATE_MASK;
	if (val >= S_RCVRY_LOCK && val <= S_L1_IDLE)
		return 1;

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.read_dbi = exynos_pcie_rc_read_dbi,
	.write_dbi = exynos_pcie_rc_write_dbi,
	.link_up = exynos_pcie_rc_link_up,
};

static void exynos_pcie_rc_set_iocc(struct dw_pcie_rp *pp, int enable)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	int val;
	u32 sysreg_sharability;

	if (!exynos_pcie->use_cache_coherency) {
		dev_err(pci->dev, "IOCC: use_cache_coherency = false\n");

		return;
	}

	if (exynos_pcie->ip_ver != EXYNOS_IP_VER_OF_WHI) {
		dev_info(pci->dev, "IOCC: not supported SoC(ip_ver=0x%x)\n", exynos_pcie->ip_ver);

		return;
	}

	/* set SYSREG SHAABILITY sfr offset of HSI1(GEN4A_0) or HSI2(GEN4A_1) */
	if (exynos_pcie->ch_num == 0)		/* HSI1 */
		sysreg_sharability = PCIE_SYSREG_HSI1_SHARABILITY_CTRL;
	else if (exynos_pcie->ch_num == 1) {	/* HSI2 */
		sysreg_sharability = PCIE_SYSREG_HSI2_SHARABILITY_CTRL;
	} else {
		dev_err(pci->dev, "IOCC: not supported: wrong ch_num\n");

		return;
	}

	if (enable) {
		dev_dbg(pci->dev, "enable cache coherency.\n");

		/* set PCIe Axcache[1] = 1 */
		exynos_pcie_rc_wr_own_conf(pp, PCIE_COHERENCY_CONTROL_3_OFF, 4, 0x10101010);

		/* set PCIe Shareability */
		val = exynos_sysreg_read(exynos_pcie, sysreg_sharability);
		val &= ~(PCIE_SYSREG_HSIX_SHARABLE_MASK);
		val |= PCIE_SYSREG_HSIX_SHARABLE_ENABLE;
		exynos_sysreg_write(exynos_pcie, val, sysreg_sharability);
	} else {
		dev_dbg(pci->dev, "disable cache coherency.\n");

		/* clear PCIe Axcache[1] = 1 */
		exynos_pcie_rc_wr_own_conf(pp, PCIE_COHERENCY_CONTROL_3_OFF, 4, 0x0);

		/* clear PCIe Shareability */
		val = exynos_sysreg_read(exynos_pcie, sysreg_sharability);
		val &= ~(PCIE_SYSREG_HSIX_SHARABLE_MASK);
		exynos_sysreg_write(exynos_pcie, val, sysreg_sharability);
	}

#if IS_ENABLED(CONFIG_EXYNOS_PCIE_IOMMU)
	pcie_sysmmu_set_use_iocc(pcie_ch_to_hsi(exynos_pcie->ch_num));
#endif

	exynos_pcie_rc_rd_own_conf(pp, PCIE_COHERENCY_CONTROL_3_OFF, 4, &val);
	dev_dbg(pci->dev, "PCIe Axcache[1] = 0x%x\n", val);

	dev_dbg(pci->dev, "PCIe Shareability(offset: 0x%x) = 0x%x\n", sysreg_sharability,
		exynos_sysreg_read(exynos_pcie, sysreg_sharability));
}

static int exynos_pcie_rc_parse_dt(struct device *dev, struct exynos_pcie *exynos_pcie)
{
	struct device_node *np = dev->of_node;
	struct device_node *syscon_np;
	struct resource res;
	const char *use_cache_coherency;
	const char *use_msi;
	const char *use_sicd;
	const char *use_sysmmu;
	const char *use_ia;
	const char *use_l1ss;
	const char *use_secure_atu;
	const char *use_nclkoff_en;
	const char *use_pcieon_sleep;
	const char *use_phy_isol_con;
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;

	if (of_property_read_u32(np, "ip-ver", &exynos_pcie->ip_ver)) {
		dev_err(dev, "Failed to parse the number of ip-ver\n");

		return -EINVAL;
	}

	if (of_property_read_u32(np, "pcie-clk-num", &exynos_pcie->pcie_clk_num)) {
		dev_err(dev, "Failed to parse the number of pcie clock\n");

		return -EINVAL;
	}

	if (of_property_read_u32(np, "phy-clk-num", &exynos_pcie->phy_clk_num)) {
		dev_err(dev, "Failed to parse the number of phy clock\n");

		return -EINVAL;
	}

	if (of_property_read_u32(np, "pmu-offset", &exynos_pcie->pmu_offset)) {
		dev_err(dev, "Failed to parse the number of pmu-offset\n");

		return -EINVAL;
	}

	if (of_property_read_u32(np, "ep-device-type", &exynos_pcie->ep_device_type)) {
		dev_err(dev, "EP device type is NOT defined, device type is 'EP_NO_DEVICE(0)'\n");
		exynos_pcie->ep_device_type = EP_NO_DEVICE;
	}

	if (of_property_read_u32(np, "max-link-speed", &exynos_pcie->max_link_speed)) {
		dev_err(dev, "MAX Link Speed is NOT defined...(GEN1)\n");
		/* Default Link Speet is GEN1 */
		exynos_pcie->max_link_speed = LINK_SPEED_GEN1;
	}

	if (of_property_read_u32(np, "chip-ver", &exynos_pcie->chip_ver)) {
		dev_err(dev, "Failed to parse the number of chip-ver, default '0'\n");
		exynos_pcie->chip_ver = 0;
	}

	if (of_property_read_u32(np, "num-lanes", &exynos_pcie->num_lanes)) {
		dev_err(dev, "Failed to parse the # of lanes, default '1'\n");
		exynos_pcie->num_lanes = 1;
	} else {
		dev_info(dev, "parse the number of lanes: %d\n", exynos_pcie->num_lanes);
	}

	if (of_property_read_u32(np, "separated-msi", &exynos_pcie->separated_msi)) {
		dev_info(dev, "Unset separated-msi value, default '0'\n");
		exynos_pcie->separated_msi = 0;
	} else {
		dev_info(dev, "parse the separated msi: %d\n", exynos_pcie->separated_msi);
	}

	if (!of_property_read_string(np, "use-cache-coherency", &use_cache_coherency)) {
		if (!strcmp(use_cache_coherency, "true")) {
			dev_info(dev, "Cache Coherency unit is ENABLED.\n");
			exynos_pcie->use_cache_coherency = true;
		} else if (!strcmp(use_cache_coherency, "false")) {
			exynos_pcie->use_cache_coherency = false;
		} else {
			dev_err(dev, "Invalid use-cache-coherency value(Set to default->false)\n");
			exynos_pcie->use_cache_coherency = false;
		}
	} else {
		exynos_pcie->use_cache_coherency = false;
	}

	if (!of_property_read_string(np, "use-msi", &use_msi)) {
		if (!strcmp(use_msi, "true")) {
			exynos_pcie->use_msi = true;
			dev_info(dev, "MSI is ENABLED.\n");
		} else if (!strcmp(use_msi, "false")) {
			dev_info(dev, "## PCIe don't use MSI\n");
			exynos_pcie->use_msi = false;
		} else {
			dev_err(dev, "Invalid use-msi value(Set to default->true)\n");
			exynos_pcie->use_msi = true;
		}
	} else {
		exynos_pcie->use_msi = false;
	}

	/* Even if MSI is enabled, we need to set msi_irq to -ENODEV.
	 * This ensures that the core pci driver doesn't register the msi_irq in
	 * dw_pcie_host_init().
	 * The PCIe interrupt is registered during the probe of this driver.
	 * The EP drivers register MSI interrupt handlers with the core driver's
	 * MSI framework.
	 * When the EP has an MSI to send, it will write to the MSI address
	 * specified in the PCIe config header which is populated with an entry
	 * from the end point device tree.
	 * The PCIe RC on detecting this write will generate an interrupt that is
	 * routed to exynos_pcie_rc_irq_handler().
	 * If it's an MSI interrupt, it will be sent to the core pcie driver
	 * dw_handle_msi_irq().
	 * This in turn will route it to the appropriate handler registered by the
	 * EP driver.
	 */

	pp->msi_irq[0] = -ENODEV;

	if (!of_property_read_string(np, "use-sicd", &use_sicd)) {
		if (!strcmp(use_sicd, "true")) {
			dev_info(dev, "## PCIe use SICD\n");
			exynos_pcie->use_sicd = true;
		} else if (!strcmp(use_sicd, "false")) {
			dev_info(dev, "## PCIe don't use SICD\n");
			exynos_pcie->use_sicd = false;
		} else {
			dev_err(dev, "Invalid use-sicd value(set to default->false)\n");
			exynos_pcie->use_sicd = false;
		}
	} else {
		exynos_pcie->use_sicd = false;
	}

	if (!of_property_read_string(np, "use-pcieon-sleep", &use_pcieon_sleep)) {
		if (!strcmp(use_pcieon_sleep, "true")) {
			dev_info(dev, "## PCIe use PCIE ON Sleep\n");
			exynos_pcie->use_pcieon_sleep = true;
		} else if (!strcmp(use_pcieon_sleep, "false")) {
			dev_info(dev, "## PCIe don't use PCIE ON Sleep\n");
			exynos_pcie->use_pcieon_sleep = false;
		} else {
			dev_err(dev, "Invalid use-pcieon-sleep value(set to default->false)\n");
			exynos_pcie->use_pcieon_sleep = false;
		}
	} else {
		exynos_pcie->use_pcieon_sleep = false;
	}

	if (!of_property_read_string(np, "use-sysmmu", &use_sysmmu)) {
		if (!strcmp(use_sysmmu, "true")) {
			dev_info(dev, "PCIe SysMMU is ENABLED.\n");
			exynos_pcie->use_sysmmu = true;
		} else if (!strcmp(use_sysmmu, "false")) {
			dev_info(dev, "PCIe SysMMU is DISABLED.\n");
			exynos_pcie->use_sysmmu = false;
		} else {
			dev_err(dev, "Invalid use-sysmmu value(set to default->false)\n");
			exynos_pcie->use_sysmmu = false;
		}
	} else {
		exynos_pcie->use_sysmmu = false;
	}

	if (!of_property_read_string(np, "use-ia", &use_ia)) {
		if (!strcmp(use_ia, "true")) {
			dev_info(dev, "PCIe I/A is ENABLED.\n");
			exynos_pcie->use_ia = true;
		} else if (!strcmp(use_ia, "false")) {
			dev_info(dev, "PCIe I/A is DISABLED.\n");
			exynos_pcie->use_ia = false;
		} else {
			dev_err(dev, "Invalid use-ia value(set to default->false)\n");
			exynos_pcie->use_ia = false;
		}
	} else {
		exynos_pcie->use_ia = false;
	}

	if (!of_property_read_string(np, "use-l1ss", &use_l1ss)) {
		if (!strcmp(use_l1ss, "true")) {
			dev_info(dev, "PCIe L1SS(L1.2) is ENABLED.\n");
			exynos_pcie->use_l1ss = true;
		} else if (!strcmp(use_l1ss, "false")) {
			dev_info(dev, "PCIe L1SS(L1.2) is DISABLED.\n");
			exynos_pcie->use_l1ss = false;
		} else {
			dev_err(dev, "Invalid use-l1ss value(default=false)\n");
			exynos_pcie->use_l1ss = false;
		}
	} else {
		exynos_pcie->use_l1ss = false;
	}

	exynos_pcie->use_secure_atu = false;
	if (!of_property_read_string(np, "use-secure-atu", &use_secure_atu)) {
		if (!strcmp(use_secure_atu, "true")) {
			dev_info(dev, "PCIe Secure ATU is ENABLED.\n");
			exynos_pcie->use_secure_atu = true;
		} else if (!strcmp(use_secure_atu, "false"))
			dev_info(dev, "PCIe Secure ATU is DISABLED.\n");
		else
			dev_err(dev, "Invalid use-secure-atu value(default=false)\n");
	}

	if (!of_property_read_string(np, "use-nclkoff-en", &use_nclkoff_en)) {
		if (!strcmp(use_nclkoff_en, "true")) {
			dev_info(dev, "PCIe NCLKOFF is ENABLED.\n");
			exynos_pcie->use_nclkoff_en = true;
		} else if (!strcmp(use_nclkoff_en, "false")) {
			dev_info(dev, "PCIe NCLKOFF is DISABLED.\n");
			exynos_pcie->use_nclkoff_en = false;
		} else {
			dev_err(dev, "Invalid use-nclkoff_en value(set to default -> false)\n");
			exynos_pcie->use_nclkoff_en = false;
		}
	} else {
		exynos_pcie->use_nclkoff_en = false;
	}

	if (!of_property_read_string(np, "use-phy-isol-en", &use_phy_isol_con)) {
		if (!strcmp(use_phy_isol_con, "true")) {
			dev_info(dev, "PCIe DYNAMIC PHY ISOLATION is Enabled.\n");
			exynos_pcie->use_phy_isol_con = true;
		} else if (!strcmp(use_phy_isol_con, "false")) {
			dev_info(dev, "PCIe DYNAMIC PHY ISOLATION is Disabled.\n");
			exynos_pcie->use_phy_isol_con = false;
		} else {
			dev_err(dev, "Invalid use-phy-isol-en value(set to default -> false)\n");
			exynos_pcie->use_phy_isol_con = false;
		}
	} else {
		exynos_pcie->use_phy_isol_con = false;
	}

	if (!exynos_pcie->use_phy_isol_con)
		exynos_pcie->phy_control = PCIE_PHY_BYPASS;

#if IS_ENABLED(CONFIG_PM_DEVFREQ)
	if (of_property_read_u32(np, "pcie-pm-qos-int", &exynos_pcie->int_min_lock))
		exynos_pcie->int_min_lock = 0;

	if (exynos_pcie->int_min_lock)
		exynos_pm_qos_add_request(&exynos_pcie_int_qos[exynos_pcie->ch_num],
					  PM_QOS_DEVICE_THROUGHPUT, 0);

	dev_info(dev, "%s: pcie int_min_lock = %d\n", __func__, exynos_pcie->int_min_lock);
#endif
	exynos_pcie->pmureg = syscon_regmap_lookup_by_phandle(np, "samsung,syscon-phandle");
	if (IS_ERR(exynos_pcie->pmureg)) {
		dev_err(dev, "syscon regmap lookup failed.\n");
		return PTR_ERR(exynos_pcie->pmureg);
	}

	syscon_np = of_parse_phandle(np, "samsung,syscon-phandle", 0);
	if (!syscon_np) {
		dev_err(dev, "syscon device node not found\n");
		return -EINVAL;
	}

	if (of_address_to_resource(syscon_np, 0, &res)) {
		dev_err(dev, "failed to get syscon base address\n");
		return -ENOMEM;
	}

	exynos_pcie->pmu_alive_pa = res.start;

	exynos_pcie->sysreg = syscon_regmap_lookup_by_phandle(np, "samsung,sysreg-phandle");
	/* Check definitions to access SYSREG in DT*/
	if (IS_ERR(exynos_pcie->sysreg) && IS_ERR(exynos_pcie->sysreg_base)) {
		dev_err(dev, "SYSREG is not defined.\n");
		return PTR_ERR(exynos_pcie->sysreg);
	}

	/* SSD & WIFI power control */
	exynos_pcie->wlan_gpio = of_get_named_gpio(np, "pcie,wlan-gpio", 0);
	if (exynos_pcie->wlan_gpio < 0) {
		dev_err(dev, "wlan gpio is not defined -> don't use wifi through pcie#%d\n",
			exynos_pcie->ch_num);
	} else {
		gpio_direction_output(exynos_pcie->wlan_gpio, 0);
	}

	exynos_pcie->ssd_gpio = of_get_named_gpio(np, "pcie,ssd-gpio", 0);
	if (exynos_pcie->ssd_gpio < 0) {
		dev_err(dev, "ssd gpio is not defined -> don't use ssd through pcie#%d\n",
			exynos_pcie->ch_num);
	} else {
		gpio_direction_output(exynos_pcie->ssd_gpio, 0);
	}

	return 0;
}

static int exynos_pcie_rc_get_pin_state(struct platform_device *pdev,
					struct exynos_pcie *exynos_pcie)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	exynos_pcie->perst_gpio = of_get_gpio(np, 0);
	if (exynos_pcie->perst_gpio < 0) {
		dev_err(&pdev->dev, "cannot get perst_gpio\n");
	} else {
		ret = devm_gpio_request_one(dev, exynos_pcie->perst_gpio,
					    GPIOF_OUT_INIT_LOW, dev_name(dev));
		if (ret)
			return -EINVAL;
	}
	/* Get pin state */
	exynos_pcie->pcie_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(exynos_pcie->pcie_pinctrl)) {
		dev_err(&pdev->dev, "Can't get pcie pinctrl!!!\n");

		return -EINVAL;
	}
	exynos_pcie->pin_state[PCIE_PIN_ACTIVE] =
		pinctrl_lookup_state(exynos_pcie->pcie_pinctrl, "active");
	if (IS_ERR(exynos_pcie->pin_state[PCIE_PIN_ACTIVE])) {
		dev_err(&pdev->dev, "Can't set pcie clkerq to output high!\n");

		return -EINVAL;
	}
	exynos_pcie->pin_state[PCIE_PIN_IDLE] =
		pinctrl_lookup_state(exynos_pcie->pcie_pinctrl, "idle");
	if (IS_ERR(exynos_pcie->pin_state[PCIE_PIN_IDLE]))
		dev_err(&pdev->dev, "No idle pin state(but it's OK)!!\n");
	else
		pinctrl_select_state(exynos_pcie->pcie_pinctrl,
				     exynos_pcie->pin_state[PCIE_PIN_IDLE]);

	return 0;
}

static int exynos_pcie_rc_clock_get(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct exynos_pcie_clks	*clks = &exynos_pcie->clks;
	int i, total_clk_num, phy_count;

	/*
	 * CAUTION - PCIe and phy clock have to define in order.
	 * You must define related PCIe clock first in DT.
	 */
	total_clk_num = exynos_pcie->pcie_clk_num + exynos_pcie->phy_clk_num;

	for (i = 0; i < total_clk_num; i++) {
		if (i < exynos_pcie->pcie_clk_num) {
			clks->pcie_clks[i] = of_clk_get(dev->of_node, i);
			if (IS_ERR(clks->pcie_clks[i])) {
				dev_err(dev, "Failed to get pcie clock\n");

				return -ENODEV;
			}
		} else {
			phy_count = i - exynos_pcie->pcie_clk_num;
			clks->phy_clks[phy_count] = of_clk_get(dev->of_node, i);
			if (IS_ERR(clks->phy_clks[i])) {
				dev_err(dev, "Failed to get pcie clock\n");

				return -ENODEV;
			}
		}
	}

	return 0;
}

static int exynos_pcie_rc_nclkoff_ctrl(struct platform_device *pdev,
				       struct exynos_pcie *exynos_pcie)
{
#ifdef NEED_NCLKOFF	/* NEED_NCLKOFF is always 0(not defined) in gs101 */
	struct device *dev = &pdev->dev;
	u32 val;

	dev_info(dev, "control NCLK OFF to prevent DBI asseccing when PCIE off\n");

	/* need to check base address & offset of each channel's sysreg */
	val = readl(exynos_pcie->sysreg_base + 0x4);
	dev_info(dev, "orig HSI1_PCIE_GEN4_0_BUS_CTRL: 0x%x\n", val);
	val &= ~PCIE_SUB_CTRL_SLV_EN;
	val &= ~PCIE_SLV_BUS_NCLK_OFF;
	val &= ~PCIE_DBI_BUS_NCLK_OFF;
	writel(val, exynos_pcie->sysreg_base);
	dev_info(dev, "aft HSI1_PCIE_GEN4_0_BUS_CTRL: 0x%x\n", val);
#endif
	return 0;
}

static int exynos_pcie_rc_get_resource(struct platform_device *pdev,
				       struct exynos_pcie *exynos_pcie)
{
	struct resource *temp_rsc;
	int ret;

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "elbi");
	exynos_pcie->elbi_base_physical_addr = temp_rsc->start;
	exynos_pcie->elbi_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(exynos_pcie->elbi_base)) {
		ret = PTR_ERR(exynos_pcie->elbi_base);

		return ret;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "udbg");
	exynos_pcie->udbg_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (temp_rsc && IS_ERR(exynos_pcie->udbg_base)) {
		ret = PTR_ERR(exynos_pcie->udbg_base);
		return ret;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	exynos_pcie->phy_base_physical_addr = temp_rsc->start;
	exynos_pcie->phy_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(exynos_pcie->phy_base)) {
		ret = PTR_ERR(exynos_pcie->phy_base);

		return ret;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sysreg");
	exynos_pcie->sysreg_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(exynos_pcie->sysreg_base)) {
		ret = PTR_ERR(exynos_pcie->sysreg_base);

		return ret;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	exynos_pcie->rc_dbi_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(exynos_pcie->rc_dbi_base)) {
		ret = PTR_ERR(exynos_pcie->rc_dbi_base);

		return ret;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pcs");
	exynos_pcie->phy_pcs_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(exynos_pcie->phy_pcs_base)) {
		ret = PTR_ERR(exynos_pcie->phy_pcs_base);

		return ret;
	}

	if (exynos_pcie->use_ia) {
		temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ia");
		exynos_pcie->ia_base_physical_addr = temp_rsc->start;
		exynos_pcie->ia_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
		if (IS_ERR(exynos_pcie->ia_base)) {
			ret = PTR_ERR(exynos_pcie->ia_base);

			return ret;
		}
	}

	return 0;
}

static void exynos_pcie_rc_enable_interrupts(struct dw_pcie_rp *pp, int enable)
{
	u32 val_irq0, val_irq1, val_irq2;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	dev_dbg(pci->dev, "## %s PCIe INTERRUPT ##\n", enable ? "ENABLE" : "DISABLE");

	if (enable) {
		if (!exynos_pcie->use_msi) {
			/* IRQ0: to enable INTX interrupt */
			val_irq0 = exynos_elbi_read(exynos_pcie, PCIE_IRQ0_EN);
			val_irq0 |= (IRQ_INTA_ENABLE | IRQ_INTB_ENABLE |
				     IRQ_INTC_ENABLE | IRQ_INTD_ENABLE);
			exynos_elbi_write(exynos_pcie, val_irq0, PCIE_IRQ0_EN);
		}

		/* IRQ1: to enable LINKDOWN interrupt */
		val_irq1 = exynos_elbi_read(exynos_pcie, PCIE_IRQ1_EN);
		val_irq1 |= IRQ_LINK_DOWN_ENABLE;
		exynos_elbi_write(exynos_pcie, val_irq1, PCIE_IRQ1_EN);

		/* IRQ2: to enable Completion_Timeout interrupt */
		val_irq2 = exynos_elbi_read(exynos_pcie, PCIE_IRQ2_EN);
		val_irq2 |= IRQ_RADM_CPL_TIMEOUT_ENABLE;
		exynos_elbi_write(exynos_pcie, val_irq2, PCIE_IRQ2_EN);

		dev_dbg(pci->dev, "enabled irqs:IRQ0_EN = 0x%x / IRQ1_EN = 0x%x / IRQ2_EN = 0x%x\n",
			val_irq0, val_irq1, val_irq2);
	} else {
		exynos_elbi_write(exynos_pcie, 0, PCIE_IRQ0_EN);
		exynos_elbi_write(exynos_pcie, 0, PCIE_IRQ1_EN);
		exynos_elbi_write(exynos_pcie, 0, PCIE_IRQ2_EN);
	}
}

static void __maybe_unused exynos_pcie_notify_callback(struct dw_pcie_rp *pp, int event)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	if (exynos_pcie->event_reg && exynos_pcie->event_reg->callback &&
	    (exynos_pcie->event_reg->events & event)) {
		struct exynos_pcie_notify *notify = &exynos_pcie->event_reg->notify;

		notify->event = event;
		notify->user = exynos_pcie->event_reg->user;
		dev_info(pci->dev, "Callback for the event : %d\n", event);
		exynos_pcie->event_reg->callback(notify);
	} else {
		dev_info(pci->dev, "Client driver does not have registration of the event : %d\n",
			 event);
		dev_info(pci->dev, "Force PCIe poweroff --> poweron\n");
		exynos_pcie_rc_poweroff(exynos_pcie->ch_num);
		exynos_pcie_rc_poweron(exynos_pcie->ch_num);
	}
}

void exynos_pcie_rc_print_msi_register(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	u32 val;
	int ctrl;

	val = exynos_elbi_read(exynos_pcie, PCIE_IRQ2_EN);
	dev_info(pci->dev, "PCIE_IRQ2_EN: 0x%x\n", val);

	exynos_pcie_rc_rd_own_conf(pp, PCIE_MSI_ADDR_LO, 4, &val);
	dev_info(pci->dev, "PCIE_MSI_ADDR_LO: 0x%x\n", val);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_MSI_ADDR_HI, 4, &val);
	dev_info(pci->dev, "PCIE_MSI_ADDR_HI: 0x%x\n", val);

	for (ctrl = 0; ctrl < PCIE_MAX_MSI_NUM; ctrl++) {
		exynos_pcie_rc_rd_own_conf(pp, PCIE_MSI_INTR0_MASK +
				(ctrl * MSI_REG_CTRL_BLOCK_SIZE), 4, &val);
		dev_info(pci->dev, "PCIE_MSI_INTR0_MASK(0x%x):0x%x\n", PCIE_MSI_INTR0_MASK +
				(ctrl * MSI_REG_CTRL_BLOCK_SIZE), val);

		exynos_pcie_rc_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE +
				(ctrl * MSI_REG_CTRL_BLOCK_SIZE), 4, &val);
		dev_info(pci->dev, "PCIE_MSI_INTR0_ENABLE(0x%x):0x%x\n", PCIE_MSI_INTR0_ENABLE +
				(ctrl * MSI_REG_CTRL_BLOCK_SIZE), val);

		exynos_pcie_rc_rd_own_conf(pp, PCIE_MSI_INTR0_STATUS +
				(ctrl * MSI_REG_CTRL_BLOCK_SIZE), 4, &val);
		dev_info(pci->dev, "PCIE_MSI_INTR0_STATUS: 0x%x\n", val);
	}
}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_print_msi_register);

void exynos_pcie_rc_register_dump(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	u32 i, val_0, val_4, val_8, val_c;

	pr_err("%s: +++\n", __func__);
	/* ---------------------- */
	/* Link Reg : 0x0 ~ 0x47C */
	/* ---------------------- */
	pr_err("[Print SUB_CTRL region]\n");
	pr_err("offset:	0x0	0x4	0x8	0xC\n");
	for (i = 0; i < 0x480; i += 0x10) {
		pr_err("ELBI 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_elbi_read(exynos_pcie, i + 0x0),
				exynos_elbi_read(exynos_pcie, i + 0x4),
				exynos_elbi_read(exynos_pcie, i + 0x8),
				exynos_elbi_read(exynos_pcie, i + 0xC));
	}
	pr_err("\n");

	/* ---------------------- */
	/* PHY Reg : 0x0 ~ 0x19C */
	/* ---------------------- */
	pr_err("[Print PHY region]\n");
	pr_err("offset:	0x0	0x4	0x8	0xC\n");
	for (i = 0; i < 0x200; i += 0x10) {
		pr_err("PHY 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_phy_read(exynos_pcie, i + 0x0),
				exynos_phy_read(exynos_pcie, i + 0x4),
				exynos_phy_read(exynos_pcie, i + 0x8),
				exynos_phy_read(exynos_pcie, i + 0xC));
	}
	/* common */
	pr_err("PHY 0x03F0:    0x%08x\n", exynos_phy_read(exynos_pcie, 0x3F0));

	/* lane0 */
	for (i = 0xE00; i < 0xED0; i += 0x10) {
		pr_err("PHY 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_phy_read(exynos_pcie, i + 0x0),
				exynos_phy_read(exynos_pcie, i + 0x4),
				exynos_phy_read(exynos_pcie, i + 0x8),
				exynos_phy_read(exynos_pcie, i + 0xC));
	}
	pr_err("PHY 0x0FC0:    0x%08x\n", exynos_phy_read(exynos_pcie, 0xFC0));

	/* lane1 */
	for (i = (0xE00 + 0x800); i < ( 0xED0 + 0x800); i += 0x10) {
		pr_err("PHY 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_phy_read(exynos_pcie, i + 0x0),
				exynos_phy_read(exynos_pcie, i + 0x4),
				exynos_phy_read(exynos_pcie, i + 0x8),
				exynos_phy_read(exynos_pcie, i + 0xC));
	}
	pr_err("PHY 0x17C0 : 0x%08x\n",
		exynos_phy_read(exynos_pcie, 0xFC0 + 0x800));

	pr_err("PHY 0x760 : %#08x, 0x764 : %#08x\n",
				exynos_phy_read(exynos_pcie, 0x760),
				exynos_phy_read(exynos_pcie, 0x764));

	/* pll control signal/state monitor */
	for (i=0x760; i<0x780; i += 0x10) {
		pr_err("PHY 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_phy_read(exynos_pcie, i + 0x0),
				exynos_phy_read(exynos_pcie, i + 0x4),
				exynos_phy_read(exynos_pcie, i + 0x8),
				exynos_phy_read(exynos_pcie, i + 0xC));
	}

	/* pll control/retry setting check */
	for (i=0x7E0; i<0x800; i += 0x10) {
		pr_err("PHY 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_phy_read(exynos_pcie, i + 0x0),
				exynos_phy_read(exynos_pcie, i + 0x4),
				exynos_phy_read(exynos_pcie, i + 0x8),
				exynos_phy_read(exynos_pcie, i + 0xC));
	}

	/* OC configuration settings (override, monitor, code, control) */
	for (i=0xA70; i<0xBD0; i += 0x10) {
		pr_err("PHY 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_phy_read(exynos_pcie, i + 0x0),
				exynos_phy_read(exynos_pcie, i + 0x4),
				exynos_phy_read(exynos_pcie, i + 0x8),
				exynos_phy_read(exynos_pcie, i + 0xC));
	}

	/* OC and rate change state, override, monitor, control */
	for (i=0xD90; i<0xDF0; i += 0x10) {
		pr_err("PHY 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_phy_read(exynos_pcie, i + 0x0),
				exynos_phy_read(exynos_pcie, i + 0x4),
				exynos_phy_read(exynos_pcie, i + 0x8),
				exynos_phy_read(exynos_pcie, i + 0xC));
	}

	/* lane OC configuration check */
	for (i=0xFB0; i<0xFC0; i += 0x10) {
		pr_err("PHY 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_phy_read(exynos_pcie, i + 0x0),
				exynos_phy_read(exynos_pcie, i + 0x4),
				exynos_phy_read(exynos_pcie, i + 0x8),
				exynos_phy_read(exynos_pcie, i + 0xC));
	}
	pr_err("\n");

	/* ---------------------- */
	/* PHY PCS : 0x0 ~ 0x19C */
	/* ---------------------- */
	pr_err("[Print PHY_PCS region]\n");
	pr_err("offset:	0x0 	0x4	0x8	0xC\n");
	for (i = 0; i < 0x200; i += 0x10) {
		pr_err("PCS 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i,
				exynos_phy_pcs_read(exynos_pcie, i + 0x0),
				exynos_phy_pcs_read(exynos_pcie, i + 0x4),
				exynos_phy_pcs_read(exynos_pcie, i + 0x8),
				exynos_phy_pcs_read(exynos_pcie, i + 0xC));
	}
	pr_err("\n");

	/* ---------------------- */
	/* DBI : 0x0 ~ 0x8FC */
	/* ---------------------- */
	pr_err("[Print DBI region]\n");
	pr_err("offset:	0x0	0x4	0x8	0xC\n");
	for (i = 0; i < 0x900; i += 0x10) {
		exynos_pcie_rc_rd_own_conf(pp, i + 0x0, 4, &val_0);
		exynos_pcie_rc_rd_own_conf(pp, i + 0x4, 4, &val_4);
		exynos_pcie_rc_rd_own_conf(pp, i + 0x8, 4, &val_8);
		exynos_pcie_rc_rd_own_conf(pp, i + 0xC, 4, &val_c);
		pr_err("DBI 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
				i, val_0, val_4, val_8, val_c);
	}
	pr_err("\n");

	/* ---------------------- */
	/* UDBG: 0xA000 ~ 0xAFFF */
	/* ---------------------- */
	if (!IS_ERR(exynos_pcie->udbg_base)) {
		exynos_udbg_write(exynos_pcie, 0x13, 0xC604); /* change udbg read mode */
		pr_err("[Print UDBG region]\n");
		pr_err("offset: 0x0     0x4     0x8     0xC\n");
		for (i = 0xA000; i < 0xB000; i += 0x10) {
			pr_err("UDBG 0x%04x:    0x%08x    0x%08x    0x%08x    0x%08x\n",
					i,
					exynos_udbg_read(exynos_pcie, i + 0x0),
					exynos_udbg_read(exynos_pcie, i + 0x4),
					exynos_udbg_read(exynos_pcie, i + 0x8),
					exynos_udbg_read(exynos_pcie, i + 0xC));
		}
		pr_err("\n");
	}
	pr_err("%s: ---\n", __func__);

}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_register_dump);

void exynos_pcie_rc_dump_link_down_status(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;

	/* if (exynos_pcie->state == STATE_LINK_UP) { */
		dev_info(pci->dev, "LTSSM: 0x%08x\n",
			 exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP));
		dev_info(pci->dev, "LTSSM_H: 0x%08x\n",
			 exynos_elbi_read(exynos_pcie, PCIE_CXPL_DEBUG_INFO_H));
		dev_info(pci->dev, "DMA_MONITOR1: 0x%08x\n",
			 exynos_elbi_read(exynos_pcie, PCIE_DMA_MONITOR1));
		dev_info(pci->dev, "DMA_MONITOR2: 0x%08x\n",
			 exynos_elbi_read(exynos_pcie, PCIE_DMA_MONITOR2));
		dev_info(pci->dev, "DMA_MONITOR3: 0x%08x\n",
			 exynos_elbi_read(exynos_pcie, PCIE_DMA_MONITOR3));
	/* } else { */
		dev_info(pci->dev, "PCIE link state is %d\n", exynos_pcie->state);
	/* } */
}

void exynos_pcie_rc_dump_all_status(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	unsigned long flags;

	spin_lock_irqsave(&exynos_pcie->conf_lock, flags);
	exynos_pcie_rc_print_link_history(pp);
	exynos_pcie_rc_dump_link_down_status(exynos_pcie->ch_num);
	exynos_pcie_rc_register_dump(exynos_pcie->ch_num);
	spin_unlock_irqrestore(&exynos_pcie->conf_lock, flags);
}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_dump_all_status);

void exynos_pcie_rc_dislink_work(struct work_struct *work)
{
	struct exynos_pcie *exynos_pcie = container_of(work, struct exynos_pcie, dislink_work.work);
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct device *dev = pci->dev;
	unsigned long flags;

	if (exynos_pcie->state == STATE_LINK_DOWN)
		return;

	if (exynos_pcie->ep_device_type != EP_SAMSUNG_MODEM) {
		spin_lock_irqsave(&exynos_pcie->conf_lock, flags);
		exynos_pcie_rc_print_link_history(pp);
		exynos_pcie_rc_dump_link_down_status(exynos_pcie->ch_num);
		exynos_pcie_rc_register_dump(exynos_pcie->ch_num);
		spin_unlock_irqrestore(&exynos_pcie->conf_lock, flags);
	}

	exynos_pcie->linkdown_cnt++;
	dev_info(dev, "link down and recovery cnt: %d\n", exynos_pcie->linkdown_cnt);
	if (exynos_pcie->use_pcieon_sleep) {
		dev_info(dev, "%s, pcie_is_linkup 0\n", __func__);
		pcie_is_linkup = 0;
	}

	exynos_pcie_notify_callback(pp, EXYNOS_PCIE_EVENT_LINKDOWN);
}

void exynos_pcie_rc_cpl_timeout_work(struct work_struct *work)
{
	struct exynos_pcie *exynos_pcie =
		container_of(work, struct exynos_pcie, cpl_timeout_work.work);
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct device *dev = pci->dev;
	unsigned long flags;

	if (exynos_pcie->state == STATE_LINK_DOWN)
		return;

	if (exynos_pcie->ep_device_type != EP_SAMSUNG_MODEM) {
		spin_lock_irqsave(&exynos_pcie->conf_lock, flags);
		exynos_pcie_rc_print_link_history(pp);
		exynos_pcie_rc_dump_link_down_status(exynos_pcie->ch_num);
		exynos_pcie_rc_register_dump(exynos_pcie->ch_num);
		spin_unlock_irqrestore(&exynos_pcie->conf_lock, flags);
	}

	if (exynos_pcie->use_pcieon_sleep) {
		dev_info(dev, "[%s] pcie_is_linkup = 0\n", __func__);
		pcie_is_linkup = 0;
	}

	dev_info(dev, "call PCIE_CPL_TIMEOUT callback\n");
	exynos_pcie_notify_callback(pp, EXYNOS_PCIE_EVENT_CPL_TIMEOUT);
}

static void exynos_pcie_rc_use_ia(struct exynos_pcie *exynos_pcie)
{
	if (!exynos_pcie->use_ia) {
		pr_info("[%s] Not support I/A(use_ia = false)\n", __func__);

		return;
	}

	/* PCIE_IA_IRQ Selection */
	/* 1. Enable Link up Interrupt for PCIE_IA */
	exynos_elbi_write(exynos_pcie, 0x400, 0x388);

	/* BASE_ADDR */
	/* Base_addr of WHI
	 * 1) [elbi_base]
	 *	ch0> 0x1192_0000, ch1> 0x1452_0000
	 * 2) [phy_base]
	 *	ch0> 0x1195_0000, ch1> 0x1455_0000
	 * 3) [ia_base]
	 *	ch0> 0x1190_0000, ch1> 0x1450_0000
	 */
	/* 1. GEN4_HSIx sub_con */
	exynos_ia_write(exynos_pcie, exynos_pcie->elbi_base_physical_addr, 0x30);
	/* 2. GEN4_HSIx pma */
	exynos_ia_write(exynos_pcie, exynos_pcie->phy_base_physical_addr, 0x34);
	/* 3. PCIE_IA_GEN4A&B base */
	exynos_ia_write(exynos_pcie, exynos_pcie->ia_base_physical_addr, 0x38);

	/* Loop CTRL */
	/* 1. LOOP Interval, 0x3000(12288) x bus_clk_period */
	exynos_ia_write(exynos_pcie, 0x00023000, 0x40);

	/* EVENT: L1.2 EXIT Interrupt happens */
	/* SQ0) UIR_1 : DATA MASK */
	/* 1. UIR_1: DATA MASK_REG */
	exynos_ia_write(exynos_pcie, 0x50000004, 0x100);
	/* 2. ENABLE bit[15:0] */
	exynos_ia_write(exynos_pcie, 0x00000100, 0x104);

	/* SQ1) BRT */
	/* 1. READ CDR_DONE REG */
	exynos_ia_write(exynos_pcie, 0x20420008, 0x108);
	/* 2. CHECK CDR_DONE */
	exynos_ia_write(exynos_pcie, 0x00000100, 0x10C);

	/* SQ2) UIR_1 : DATA MASK */
	/* 1. UIR_1: DATA MASK_REG */
	exynos_ia_write(exynos_pcie, 0x50000004, 0x110);
	/* 2. ENABLE bit[15:0] */
	exynos_ia_write(exynos_pcie, 0x00000400, 0x114);

	/* SQ3) LOOP */
	/* 1. CDR_EN 0x18 */
	exynos_ia_write(exynos_pcie, 0x10000008, 0x118);
	/* 2. LOOP until L1.2 exit IRQ clear,
	 * but loop timeout because of pending still
	 */
	exynos_ia_write(exynos_pcie, 0x00000000, 0x11C);

	/* SQ4) WRITE */
	/* 1. CDR_EN 0x00 */
	exynos_ia_write(exynos_pcie, 0x40020008, 0x120);
	/* 2. Write0-Clear LOOP_CNT_OVER_INTR_STATUS */
	exynos_ia_write(exynos_pcie, 0x00000100, 0x124);

	/* SQ5) UIR_1 : DATA MASK */
	/* 1. AFC toggle */
	exynos_ia_write(exynos_pcie, 0x50000004, 0x128);
	/* 2. ENABLE bit[15:0] */
	exynos_ia_write(exynos_pcie, 0x000000F0, 0x12C);

	/* SQ6) BRT */
	/* 1. READ CDR_DONE REG */
	exynos_ia_write(exynos_pcie, 0x20D10FC0, 0x130);
	/* 2. CHECK CDR_DONE */
	exynos_ia_write(exynos_pcie, 0x000000F0, 0x134);

	/* SQ7) WRITE */
	/* 1. CDR_EN 0x20 */
	exynos_ia_write(exynos_pcie, 0x40010A48, 0x138);
	exynos_ia_write(exynos_pcie, 0x00000020, 0x13C);

	/* SQ8) WRITE */
	/* 1. CDR_EN 0x30 */
	exynos_ia_write(exynos_pcie, 0x40010A48, 0x140);
	exynos_ia_write(exynos_pcie, 0x00000030, 0x144);

	/* SQ9) WRITE */
	/* 1. CDR_EN 0x00 */
	exynos_ia_write(exynos_pcie, 0x40010A48, 0x148);
	exynos_ia_write(exynos_pcie, 0x00000000, 0x14C);

	/* SQ10) WRITE */
	/* 1. AFC toggle */
	exynos_ia_write(exynos_pcie, 0x40010BF4, 0x150);
	exynos_ia_write(exynos_pcie, 0x00000005, 0x154);

	/* SQ11) WRITE */
	/* 1. AFC toggle */
	exynos_ia_write(exynos_pcie, 0x40010BF4, 0x158);
	exynos_ia_write(exynos_pcie, 0x00000004, 0x15C);

	/* SQ12) BRF */
	/* 1. READ CDR_DONE REG */
	exynos_ia_write(exynos_pcie, 0x30010FC0, 0x160);
	/* 2. CHECK CDR_DONE  */
	exynos_ia_write(exynos_pcie, 0x000000F0, 0x164);

	/* SQ13) WRITE */
	/* 1. AFC toggle */
	exynos_ia_write(exynos_pcie, 0x40010BF4, 0x168);
	exynos_ia_write(exynos_pcie, 0x00000005, 0x16C);

	/* SQ14) WRITE */
	exynos_ia_write(exynos_pcie, 0x40000008, 0x170);
	/* 1. Write1-Clear IRQ  */
	exynos_ia_write(exynos_pcie, 0x00000400, 0x174);

	/* SQ15) WRITE */
	/* 1. RETURN to IDLE */
	exynos_ia_write(exynos_pcie, 0x80000000, 0x178);
	exynos_ia_write(exynos_pcie, 0x00000000, 0x17C);

	/* PCIE_IA_EN */
	exynos_ia_write(exynos_pcie, 0x00000001, 0x000);
}

void exynos_pcie_rc_assert_phy_reset(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct device *dev = pci->dev;
	int ret;

	ret = exynos_pcie_rc_phy_clock_enable(pp, PCIE_ENABLE_CLOCK);
	dev_dbg(dev, "phy clk enable, ret value = %d\n", ret);
	if (exynos_pcie->phy_ops.phy_config)
		exynos_pcie->phy_ops.phy_config(exynos_pcie, exynos_pcie->ch_num);

	/* Added for CDR Lock */
	exynos_pcie_rc_use_ia(exynos_pcie);
}

void exynos_pcie_rc_resumed_phydown(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	int ret;

	logbuffer_log(exynos_pcie->log, "resumed_phydown");

	ret = exynos_pcie_rc_clock_enable(pp, PCIE_ENABLE_CLOCK);
	dev_dbg(dev, "pcie clk enable, ret value = %d\n", ret);

	exynos_pcie_rc_enable_interrupts(pp, 0);
	exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_BYPASS);

	if (exynos_pcie->ch_num == 1) {
		void __iomem *pmu_addr, *cmu_addr;
		u32 pmu_val, cmu_val;

		pmu_addr = ioremap(exynos_pcie->pmu_alive_pa + 0x3C10, 0x4);
		cmu_addr = ioremap(0x14400800, 0x4);
		pmu_val = readl(pmu_addr);
		cmu_val = readl(cmu_addr);

		/* Make sure TXCO always on was enabled */
		if ((pmu_val & PMU_CHECK_MASK) != PMU_CHECK_MASK) {
			dev_err(dev, "ERR pmu+0x3C10=%#x\n", pmu_val);
			pmu_val |= 0x40C;
			writel(pmu_val, pmu_addr);
		}

		/* Disable automatic clock gating on OSCCLK */
		cmu_val &= ~BIT(28);
		writel(cmu_val, cmu_addr);
		logbuffer_log(exynos_pcie->log, "Disable automatic clock gating on OSCCLK");

		iounmap(pmu_addr);
		iounmap(cmu_addr);
	}

	if (exynos_pcie->phy_ops.phy_all_pwrdn)
		exynos_pcie->phy_ops.phy_all_pwrdn(exynos_pcie, exynos_pcie->ch_num);

	exynos_pcie_rc_phy_clock_enable(pp, PCIE_DISABLE_CLOCK);
	exynos_pcie_rc_clock_enable(pp, PCIE_DISABLE_CLOCK);
}

static void exynos_pcie_setup_rc(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	u32 pcie_cap_off = PCIE_CAP_OFFSET;
	u32 pm_cap_off = PM_CAP_OFFSET;
	u32 val;

	/* enable writing to DBI read-only registers */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MISC_CONTROL, 4, DBI_RO_WR_EN);

	if (exynos_pcie->ep_device_type == EP_SAMSUNG_MODEM) {
		/* Disable BAR and Exapansion ROM BAR */
		exynos_pcie_rc_wr_own_conf(pp, 0x100010, 4, 0);
		exynos_pcie_rc_wr_own_conf(pp, 0x100030, 4, 0);
	}

	/* change vendor ID and device ID for PCIe */
	exynos_pcie_rc_wr_own_conf(pp, PCI_VENDOR_ID, 2, PCI_VENDOR_ID_SAMSUNG);
	exynos_pcie_rc_wr_own_conf(pp, PCI_DEVICE_ID, 2,
				   PCI_DEVICE_ID_EXYNOS + exynos_pcie->ch_num);

	/* set max link width & speed : Gen3, Lane1 */
	exynos_pcie_rc_rd_own_conf(pp, pcie_cap_off + PCI_EXP_LNKCAP, 4, &val);
	val &= ~(PCI_EXP_LNKCAP_L1EL | PCI_EXP_LNKCAP_SLS);
	val |= PCI_EXP_LNKCAP_L1EL_64USEC;
	val |= PCI_EXP_LNKCTL2_TLS_8_0GB;

	exynos_pcie_rc_wr_own_conf(pp, pcie_cap_off + PCI_EXP_LNKCAP, 4, val);

	/* set auxiliary clock frequency: 26MHz */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_AUX_CLK_FREQ_OFF, 4, PCIE_AUX_CLK_FREQ_26MHZ);

	/* set duration of L1.2 & L1.2.Entry */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_L1_SUBSTATES_OFF, 4, PCIE_L1_SUB_VAL);

	/* clear power management control and status register */
	exynos_pcie_rc_wr_own_conf(pp, pm_cap_off + PCI_PM_CTRL, 4, 0x0);

	/* set target speed from DT */
	exynos_pcie_rc_rd_own_conf(pp, pcie_cap_off + PCI_EXP_LNKCTL2, 4, &val);
	val &= ~PCI_EXP_LNKCTL2_TLS;
	val |= exynos_pcie->max_link_speed;
	exynos_pcie_rc_wr_own_conf(pp, pcie_cap_off + PCI_EXP_LNKCTL2, 4, val);

	/* TBD : */
	/* initiate link retraining */
	/* exynos_pcie_rc_rd_own_conf(pp, pcie_cap_off + PCI_EXP_LNKCTL, 4, &val);
	 * val |= PCI_EXP_LNKCTL_RL;
	 * exynos_pcie_rc_wr_own_conf(pp, pcie_cap_off + PCI_EXP_LNKCTL, 4, val);
	 */

	/* completion timeout setting values*/
	/* Range/ Encoding/ Spec Minimum/ Spec Maximum/ PCIe controller Minimum/
	 *						 PCIe controller Maximum
	 *Default 0000b 50 us 50 ms 28 ms 44 ms (M-PCIe: 45)
	 *	A 0001b 50 us 100 us 65 us 99 us (M-PCIe: 100)
	 *	A 0010b 1 ms 10 ms 4.1 ms 6.2 ms (M-PCIe: 6.4)
	 *	B 0101b 16 ms 55 ms 28 ms 44 ms (M-PCIe: 45)
	 *	B 0110b 65 ms 210 ms 86 ms 131 ms (M-PCIe: 133)
	 *	C 1001b 260 ms 900 ms 260 ms 390 ms (M-PCIe: 398)
	 *	C 1010b 1s 3.5 s 1.8 s 2.8 s (M-PCIe: 2.8)
	 *	D 1101b 4s 13 s 5.4 s 8.2 s (M-PCIe: 8.4)
	 *	D 1110b 17s 64 s 38 s 58 s (M-PCIe: 59)
	 */
	exynos_pcie_rc_rd_own_conf(pp, pcie_cap_off + PCI_EXP_DEVCTL2, 4, &val);
	/* pr_info("%s:read:device_ctrl_status2(0x98)=0x%x\n", __func__, val); */
	val &= ~(PCIE_CAP_CPL_TIMEOUT_VAL_MASK);
	val |= PCIE_CAP_CPL_TIMEOUT_VAL_44MS_DEFALT;
	exynos_pcie_rc_wr_own_conf(pp, pcie_cap_off + PCI_EXP_DEVCTL2, 4, val);
}

static int exynos_pcie_rc_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	/* Setup RC to avoid initialization faile in PCIe stack */
	if (exynos_pcie->use_phy_isol_con)
		pp->bridge->ops = &exynos_pcie_rc_root_ops;
	pp->bridge->child_ops = &exynos_pcie_rc_child_ops;

	return 0;
}

static struct dw_pcie_host_ops exynos_pcie_rc_ops = {
	.host_init = exynos_pcie_rc_init,
};

void exynos_pcie_msi_post_process(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	int ctrl, num_ctrls;

	if (exynos_pcie->separated_msi)
		return;

	num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;
	for (ctrl = 0; ctrl < num_ctrls; ctrl++) {
		/* clear MSI register because of edge */
		exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_MASK +
				(ctrl * MSI_REG_CTRL_BLOCK_SIZE), 4, 0xffffffff);
		exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_MASK +
				(ctrl * MSI_REG_CTRL_BLOCK_SIZE), 4, 0x0);
	}
}

void exynos_pcie_rc_force_linkdown_work(int ch_num) {
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct device *dev = pci->dev;

	dev_info(dev, "start force Link down S/W recovery\n");
	if (exynos_pcie->cpl_timeout_recovery)
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
				"already in cpl recovery");
	else {
		if (exynos_pcie->sudden_linkdown)
			logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
					"already in linkdown recovery");
		else {
			logbuffer_log(exynos_pcie->log, "start linkdown recovery");
			exynos_pcie->sudden_linkdown = 1;
			exynos_pcie->state = STATE_LINK_DOWN_TRY;
			exynos_pcie_notify_callback(pp, EXYNOS_PCIE_EVENT_LINKDOWN);
		}
	}

}
EXPORT_SYMBOL(exynos_pcie_rc_force_linkdown_work);

static irqreturn_t exynos_pcie_rc_irq_handler(int irq, void *arg)
{
	struct dw_pcie_rp *pp = arg;
	u32 val_irq0, val_irq1, val_irq2;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct device *dev = pci->dev;

	/* handle IRQ0 interrupt */
	val_irq0 = exynos_elbi_read(exynos_pcie, PCIE_IRQ0);
	exynos_elbi_write(exynos_pcie, val_irq0, PCIE_IRQ0);

	/* handle IRQ1 interrupt */
	val_irq1 = exynos_elbi_read(exynos_pcie, PCIE_IRQ1);
	exynos_elbi_write(exynos_pcie, val_irq1, PCIE_IRQ1);

	/* handle IRQ2 interrupt */
	val_irq2 = exynos_elbi_read(exynos_pcie, PCIE_IRQ2);
	exynos_elbi_write(exynos_pcie, val_irq2, PCIE_IRQ2);

	/* only support after EXYNOS9820 EVT 1.1 */
	if (val_irq1 & IRQ_LINK_DOWN_ASSERT) {
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_INFO,
			       "! PCIE LINK DOWN-irq1_state: 0x%x !", val_irq1);
		dev_info(dev, "(irq0 = 0x%x, irq1 = 0x%x, irq2 = 0x%x)\n",
			 val_irq0, val_irq1, val_irq2);
		exynos_pcie->link_stats.link_down_irq_count++;

		if (exynos_pcie->cpl_timeout_recovery)
			logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR, "already in cpl recovery");
		else {
			if (exynos_pcie->sudden_linkdown)
				logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
						"already in linkdown recovery");
			else {
				logbuffer_log(exynos_pcie->log, "start linkdown recovery");
				exynos_pcie->sudden_linkdown = 1;
				exynos_pcie->state = STATE_LINK_DOWN_TRY;
				queue_work(exynos_pcie->pcie_wq, &exynos_pcie->dislink_work.work);
			}
		}
	}

	if (val_irq2 & IRQ_RADM_CPL_TIMEOUT_ASSERT) {
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_INFO,
			       "!! PCIE_CPL_TIMEOUT-irq2_state: 0x%x !!", val_irq2);
		dev_info(dev, "(irq0 = 0x%x, irq1 = 0x%x, irq2 = 0x%x)\n",
			 val_irq0, val_irq1, val_irq2);
		exynos_pcie->link_stats.cmpl_timeout_irq_count++;

		val_irq2 = exynos_elbi_read(exynos_pcie, PCIE_IRQ2);
		dev_info(dev, "check irq22 pending clear: irq2_state = 0x%x\n", val_irq2);

		if (exynos_pcie->sudden_linkdown)
			logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
				       "already in linkdown recovery");
		else {
			if (exynos_pcie->cpl_timeout_recovery)
				logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
					      "in cpl recovery");
			else {
				logbuffer_log(exynos_pcie->log, "start cpl recovery");
				exynos_pcie->cpl_timeout_recovery = 1;
				exynos_pcie->state = STATE_LINK_DOWN_TRY;
				queue_work(exynos_pcie->pcie_wq,
					   &exynos_pcie->cpl_timeout_work.work);
			}
		}
	}

#if IS_ENABLED(CONFIG_PCI_MSI)
	if (val_irq2 & IRQ_MSI_RISING_ASSERT && exynos_pcie->use_msi) {
		if (exynos_pcie->separated_msi && exynos_pcie->use_pcieon_sleep) {
			dev_info(dev, "MSI: separated msi & pcieonsleep\n");
			return IRQ_HANDLED;
		}

		dw_handle_msi_irq(pp);

		/* Mask & Clear MSI to pend MSI interrupt.
		 * After clearing IRQ_PULSE, MSI interrupt can be ignored if
		 * lower MSI status bit is set while processing upper bit.
		 * Through the Mask/Unmask, ignored interrupts will be pended.
		 */
		exynos_pcie_msi_post_process(pp);
	}
#endif

	return IRQ_HANDLED;
}

static int exynos_pcie_rc_msi_init(struct dw_pcie_rp *pp)
{
	u32 val, mask_val, i;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct device *dev = pci->dev;
	struct pci_bus *ep_pci_bus;
	/*
	 * The following code is added to avoid duplicated allocation.
	 */
	if (!exynos_pcie->probe_ok) {
		dev_dbg(dev, "%s: allocate MSI data\n", __func__);
		ep_pci_bus = pci_find_bus(exynos_pcie->pci_dev->bus->domain_nr, 1);
		exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0, MSI_CONTROL, 4, &val);
		dev_dbg(dev, "%s: EP support %d-bit MSI address (0x%x)\n",
			__func__, (val & MSI_64CAP_MASK) ? 64 : 32, val);

		if (exynos_pcie->ep_device_type == EP_SAMSUNG_MODEM) {
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
			/* Should have been populated by exynos_pcie_set_msi_ctrl_addr()*/
			if (!pp->msi_data) {
				dev_err(dev, "%s: msi address is null\n", __func__);
				return -EINVAL;
			}
			dev_dbg(dev, "msi_data : %pad\n", &pp->msi_data);
#else
			dev_dbg(dev, "EP is Modem but ModemIF is disabled\n");
#endif
		} else {
			if ((pp->msi_data >> 32) != 0)
				dev_info(dev, "MSI memory is allocated over 32bit boundary\n");
			dev_dbg(dev, "msi_data : %pad\n", &pp->msi_data);
		}
	}

	dev_dbg(dev, "%s: Program the MSI data: 0x%lx (probe ok:%d)\n",
		__func__, (unsigned long)pp->msi_data, exynos_pcie->probe_ok);
	/* Program the msi_data */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4, lower_32_bits(pp->msi_data));
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4, upper_32_bits(pp->msi_data));

	val = exynos_elbi_read(exynos_pcie, PCIE_IRQ2_EN);
	if (exynos_pcie->separated_msi)
		val &= ~IRQ_MSI_CTRL_EN_RISING_EDG;
	else
		val |= IRQ_MSI_CTRL_EN_RISING_EDG;
	exynos_elbi_write(exynos_pcie, val, PCIE_IRQ2_EN);

	/* Enable MSI interrupt after PCIe reset */
	val = (u32)(*pp->msi_irq_in_use);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE, 4, val);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE, 4, &val);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_MSI_INTR0_MASK, 4, &mask_val);

	if (exynos_pcie->separated_msi) {
		pr_debug("Enable Separated MSI IRQs.\n");
		for (i = PCIE_START_SEP_MSI_VEC; i < PCIE_MAX_SEPA_IRQ_NUM; i++) {
			if (sep_msi_vec[exynos_pcie->ch_num][i].is_used) {
				/* Enable MSI interrupt for separated MSI. */
				pr_debug("Separated MSI%d is Enabled.\n", i);
				exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE +
						(i * MSI_REG_CTRL_BLOCK_SIZE), 4, 0x1);
				exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_MASK +
						(i * MSI_REG_CTRL_BLOCK_SIZE), 4, ~(0x1));
			}
		}
	}
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	dev_dbg(dev, "MSI INIT: check MSI_INTR0_ENABLE(0x%x): 0x%x\n", PCIE_MSI_INTR0_ENABLE, val);
	if (exynos_pcie->ep_device_type != EP_QC_WIFI) {
		if (val != 0xf1) {
			exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE, 4, 0xf1);
			exynos_pcie_rc_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE, 4, &val);
		}
	}

	dev_dbg(dev, "MSI INIT: check MSI_INTR0_MASK(0x%x): 0x%x\n", PCIE_MSI_INTR0_MASK, mask_val);
	mask_val &= ~(val);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_MASK, 4, mask_val);
	udelay(1);
	exynos_pcie_rc_rd_own_conf(pp, PCIE_MSI_INTR0_MASK, 4, &mask_val);
#endif

	dev_dbg(dev, "%s: MSI INIT END (MSI_ENABLE(0x%x)=0x%x, MSI_MASK(0x%x)=0x%x)\n",
		__func__, PCIE_MSI_INTR0_ENABLE, val, PCIE_MSI_INTR0_MASK, mask_val);

	return 0;
}

static void exynos_pcie_rc_send_pme_turn_off(struct exynos_pcie *exynos_pcie)
{
	struct dw_pcie *pci = exynos_pcie->pci;
	struct device *dev = pci->dev;
	int count = 0;
	u32 val;

	/* L1.2 enable check */
	dev_dbg(dev, "Current PM state(PCS + 0x188) : 0x%x\n",
		readl(exynos_pcie->phy_pcs_base + 0x188));
	logbuffer_log(exynos_pcie->log, "Current PM state(PCS + 0x188) : 0x%x",
		      readl(exynos_pcie->phy_pcs_base + 0x188));
	dev_dbg(dev, "DBI Link Control Register: 0x%x\n", readl(exynos_pcie->rc_dbi_base + 0x80));
	logbuffer_log(exynos_pcie->log, "DBI Link Control Register: 0x%x",
		      readl(exynos_pcie->rc_dbi_base + 0x80));

	val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP) & PCIE_ELBI_LTSSM_STATE_MASK;
	dev_dbg(dev, "%s: link state:%x\n", __func__, val);
	logbuffer_log(exynos_pcie->log, "link state:%x", val);
	if (!(val >= S_RCVRY_LOCK && val <= S_L1_IDLE)) {
		dev_info(dev, "%s, pcie link is not up\n", __func__);

		return;
	}

	exynos_elbi_write(exynos_pcie, 0x1, PCIE_APP_REQ_EXIT_L1);
	val = exynos_elbi_read(exynos_pcie, PCIE_APP_REQ_EXIT_L1_MODE);
	val &= ~APP_REQ_EXIT_L1_MODE;
	val |= L1_REQ_NAK_CONTROL_MASTER;
	exynos_elbi_write(exynos_pcie, val, PCIE_APP_REQ_EXIT_L1_MODE);

	exynos_elbi_write(exynos_pcie, 0x1, XMIT_PME_TURNOFF);
	logbuffer_log(exynos_pcie->log, "Xmit OFF sent");

	while (count < MAX_L2_TIMEOUT) {
		if ((exynos_elbi_read(exynos_pcie, PCIE_IRQ0) & IRQ_RADM_PM_TO_ACK)) {
			dev_dbg(dev, "ack message is ok\n");
			logbuffer_log(exynos_pcie->log, "ack message is ok");
			udelay(10);

			break;
		}

		udelay(10);
		count++;
	}
	if (count >= MAX_L2_TIMEOUT)
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
			       "cannot receive ack message from EP");

	exynos_elbi_write(exynos_pcie, 0x0, XMIT_PME_TURNOFF);

	count = 0;
	do {
		val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP)
		      & PCIE_ELBI_LTSSM_STATE_MASK;
		if (val == S_L2_IDLE) {
			dev_dbg(dev, "received Enter_L23_READY DLLP packet\n");
			logbuffer_log(exynos_pcie->log, "received Enter_L23_READY DLLP packet");
			break;
		}
		udelay(10);
		count++;
	} while (count < MAX_L2_TIMEOUT);

	if (count >= MAX_L2_TIMEOUT)
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
			       "cannot receive L23_READY DLLP packet(0x%x)", val);
}

static int exynos_pcie_rc_establish_link(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	void __iomem *phy_base_regs = exynos_pcie->phy_base;
	struct device *dev = pci->dev;
	u32 val, busdev;
	int count = 0, try_cnt = 0;
	unsigned int save_before_state = 0xff;
	bool pll_lock, cdr_lock, oc_done;
	int lock_cnt;
retry:
	logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR, "OC Initial Status check: 0x5FC(0x%x)\n",
			exynos_phy_read(exynos_pcie, 0x5FC));

	/* to call eyxnos_pcie_rc_pcie_phy_config() in cal.c file */
	exynos_pcie_rc_assert_phy_reset(pp);

	/* check pll lock */
	pll_lock = check_exynos_pcie_reg_status(exynos_pcie, phy_base_regs,
						0x03F0, BIT(3), BIT(3), &lock_cnt);

	/* check cdr lock */
	cdr_lock = check_exynos_pcie_reg_status(exynos_pcie, phy_base_regs,
						0x0E0C, BIT(2), BIT(2), &lock_cnt);
	if (cdr_lock)
		link_stats_log_pll_lock(exynos_pcie, lock_cnt);

	/* check offset calibration */
	oc_done = check_exynos_pcie_reg_status(exynos_pcie, phy_base_regs,
					       0x0DE8, 0xf, 0xf, &lock_cnt);
	if (!oc_done)
		dev_err(dev, "OC Fail: PLL_LOCK:%d, CDR_LOCK:%d, OC:%d\n",
			pll_lock, cdr_lock, oc_done);

	logbuffer_log(exynos_pcie->log,
		      "PLL_LOCK:%d, CDR_LOCK:%d, OC:%d",
		      pll_lock, cdr_lock, oc_done);

	/* Soft Power RST */
	val = exynos_elbi_read(exynos_pcie, PCIE_SOFT_RESET);
	val &= ~SOFT_PWR_RESET;
	exynos_elbi_write(exynos_pcie, val, PCIE_SOFT_RESET);
	/* old: udelay(20); */
	mdelay(1);
	val |= SOFT_PWR_RESET;
	exynos_elbi_write(exynos_pcie, val, PCIE_SOFT_RESET);

	val = exynos_phy_read(exynos_pcie, 0x0E18) & (1 << 7);
	if (!val) {
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR, "OC Fail after soft power reset!");
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
			      "PMA Info : 0x760(0x%x), 0xE0C(0x%x), 0x3F0(0x%x), 0xFC0(0x%x)",
			      exynos_phy_read(exynos_pcie, 0x760),
			      exynos_phy_read(exynos_pcie, 0xE0C),
			      exynos_phy_read(exynos_pcie, 0x3F0),
			      exynos_phy_read(exynos_pcie, 0xFC0));
	}
	/* Device Type (Sub Controller: DEVICE_TYPE offset: 0x80  */
	exynos_elbi_write(exynos_pcie, 0x04, 0x80);

	/* NON-sticky RST */
	val = exynos_elbi_read(exynos_pcie, PCIE_SOFT_RESET);
	val |= SOFT_NON_STICKY_RESET;
	exynos_elbi_write(exynos_pcie, val, PCIE_SOFT_RESET);
	usleep_range(10, 12);
	val &= ~SOFT_NON_STICKY_RESET;
	exynos_elbi_write(exynos_pcie, val, PCIE_SOFT_RESET);
	mdelay(1);
	val |= SOFT_NON_STICKY_RESET;
	exynos_elbi_write(exynos_pcie, val, PCIE_SOFT_RESET);

	/* EQ Off */
	exynos_pcie_rc_wr_own_conf(pp, 0x890, 4, 0x12000);

	/* set #PERST high */
	gpio_set_value(exynos_pcie->perst_gpio, 1);

	dev_dbg(dev, "%s: Set PERST to HIGH, gpio val = %d\n",
		__func__, gpio_get_value(exynos_pcie->perst_gpio));
	if (exynos_pcie->ep_device_type == EP_BCM_WIFI) {
		usleep_range(20000, 22000);
	} else {
		usleep_range(18000, 20000);
	}

	val = exynos_elbi_read(exynos_pcie, PCIE_APP_REQ_EXIT_L1_MODE);
	val |= APP_REQ_EXIT_L1_MODE;
	val |= L1_REQ_NAK_CONTROL_MASTER;
	exynos_elbi_write(exynos_pcie, val, PCIE_APP_REQ_EXIT_L1_MODE);
	exynos_elbi_write(exynos_pcie, PCIE_LINKDOWN_RST_MANUAL, PCIE_LINKDOWN_RST_CTRL_SEL);

	/* Q-Channel support */
	val = exynos_elbi_read(exynos_pcie, PCIE_QCH_SEL);
	if (exynos_pcie->ip_ver >= 0x889500) {
		val &= ~(CLOCK_GATING_PMU_MASK | CLOCK_GATING_APB_MASK | CLOCK_GATING_AXI_MASK);
	} else {
		val &= ~CLOCK_GATING_MASK;
		val |= CLOCK_NOT_GATING;
	}
	exynos_elbi_write(exynos_pcie, val, PCIE_QCH_SEL);

	/* NAK enable when AXI pending */
	exynos_elbi_write(exynos_pcie, NACK_ENABLE, PCIE_MSTR_PEND_SEL_NAK);
	dev_dbg(dev, "%s: NACK option enable: 0x%x\n", __func__,
		exynos_elbi_read(exynos_pcie, PCIE_MSTR_PEND_SEL_NAK));

	/* DBI L1 exit disable(use aux_clk in L1.2) */
	exynos_elbi_write(exynos_pcie, DBI_L1_EXIT_DISABLE, PCIE_DBI_L1_EXIT_DISABLE);
	dev_dbg(dev, "%s: DBI L1 exit disable option enable: 0x%x\n", __func__,
		exynos_elbi_read(exynos_pcie, PCIE_DBI_L1_EXIT_DISABLE));

	/* setup root complex */
	dw_pcie_setup_rc(pp);
	exynos_pcie_setup_rc(pp);

	if (exynos_pcie->use_cache_coherency)
		exynos_pcie_rc_set_iocc(pp, 1);

	logbuffer_logk(exynos_pcie->log, LOGLEVEL_INFO, "D state: %x, LTSSM: %x",
		exynos_elbi_read(exynos_pcie, PCIE_PM_DSTATE) & PCIE_PM_DSTATE_MASK,
		exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP) & PCIE_ELBI_LTSSM_STATE_MASK);

	save_before_state = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP);
	/* DBG: sleep_range(48000, 50000); */

	/* assert LTSSM enable */
	exynos_elbi_write(exynos_pcie, PCIE_ELBI_LTSSM_ENABLE, PCIE_APP_LTSSM_ENABLE);
	count = 0;
	while (count < MAX_TIMEOUT) {
		val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP)
		      & PCIE_ELBI_LTSSM_STATE_MASK;
		if (val == S_L0)
			break;
		count++;
		usleep_range(10, 12);
	}

	if (count >= MAX_TIMEOUT) {
		try_cnt++;

		val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP)
		      & PCIE_ELBI_LTSSM_STATE_MASK;
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
			       "Link is not up, try count: %d, linksts: %s(0x%x)",
			       try_cnt, LINK_STATE_DISP(val), val);
		exynos_pcie->link_stats.link_up_failure_count++;

		restore_pma_regs(exynos_pcie);

		if (try_cnt < 10) {
			gpio_set_value(exynos_pcie->perst_gpio, 0);
			dev_dbg(dev, "%s: Set PERST to LOW, gpio val = %d\n", __func__,
				gpio_get_value(exynos_pcie->perst_gpio));
			logbuffer_log(exynos_pcie->log, "%s: Set PERST to LOW, gpio val = %d",
				      __func__, gpio_get_value(exynos_pcie->perst_gpio));
			/* LTSSM disable */
			exynos_elbi_write(exynos_pcie, PCIE_ELBI_LTSSM_DISABLE,
					  PCIE_APP_LTSSM_ENABLE);
			exynos_pcie_rc_phy_clock_enable(pp, PCIE_DISABLE_CLOCK);

			goto retry;
		} else {
			//exynos_pcie_host_v1_print_link_history(pp);
			exynos_pcie_rc_print_link_history(pp);
			exynos_pcie_rc_dump_link_down_status(exynos_pcie->ch_num);
			exynos_pcie_rc_register_dump(exynos_pcie->ch_num);

			exynos_pcie->link_stats.link_recovery_failure_count++;

			if (exynos_pcie->ip_ver >= 0x889000 &&
			    exynos_pcie->ep_device_type == EP_BCM_WIFI) {
				return -EPIPE;
			}

			return -EPIPE;
		}
	} else {
		val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP)
		      & PCIE_ELBI_LTSSM_STATE_MASK;
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_INFO, "%s(0x%x)", LINK_STATE_DISP(val),
			       val);
		link_stats_log_link_up(exynos_pcie, count);

		save_pma_regs(exynos_pcie);

		dev_dbg(dev, "(phy+0xC08=0x%x)(phy+0x1408=0x%x)(phy+0xC6C=0x%x)(phy+0x146C=0x%x)\n",
			exynos_phy_read(exynos_pcie, 0xC08),
			exynos_phy_read(exynos_pcie, 0x1408),
			exynos_phy_read(exynos_pcie, 0xC6C),
			exynos_phy_read(exynos_pcie, 0x146C));
		logbuffer_log(exynos_pcie->log,
			      "(phy+0xC08=0x%x)(phy+0x1408=0x%x)(phy+0xC6C=0x%x)(phy+0x146C=0x%x)",
			      exynos_phy_read(exynos_pcie, 0xC08),
			      exynos_phy_read(exynos_pcie, 0x1408),
			      exynos_phy_read(exynos_pcie, 0xC6C),
			      exynos_phy_read(exynos_pcie, 0x146C));

		/* need delay for link speed change from GEN1 to Max(ex GEN3) */
		usleep_range(2800, 3000); /* 3 ms - OK */

		exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_CTRL_STAT, 4, &val);
		val = (val >> 16) & 0xf;
		dev_dbg(dev, "Current Link Speed is GEN%d (MAX GEN%d)\n",
			val, exynos_pcie->max_link_speed);
		logbuffer_log(exynos_pcie->log, "Current Link Speed is GEN%d (MAX GEN%d)",
			      val, exynos_pcie->max_link_speed);

		/* check link training result(speed) */
		if (exynos_pcie->ip_ver >= 0x982000 && val < exynos_pcie->max_link_speed) {
			try_cnt++;
			dev_info(dev, "%s: Link is up. But not max speed, try count: %d\n",
				__func__, try_cnt);
			if (try_cnt < 10) {
				gpio_set_value(exynos_pcie->perst_gpio, 0);
				dev_dbg(dev, "Set PERST LOW, gpio val = %d\n",
					gpio_get_value(exynos_pcie->perst_gpio));
				/* LTSSM disable */
				exynos_elbi_write(exynos_pcie, PCIE_ELBI_LTSSM_DISABLE,
						  PCIE_APP_LTSSM_ENABLE);
				exynos_pcie_rc_phy_clock_enable(pp, PCIE_DISABLE_CLOCK);

				goto retry;
			} else {
				dev_info(dev, "Current Link Speed is GEN%d (MAX GEN%d)\n",
					val, exynos_pcie->max_link_speed);
			}
		}

		/* check L0 state one more time after link recovery */
		count = 0;
		dev_dbg(dev, "check L0 state after link recovery\n");
		while (count < MAX_TIMEOUT) {
			val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP)
			      & PCIE_ELBI_LTSSM_STATE_MASK;
			if (val >= S_L0 && val <= S_L1_IDLE)
				break;
			count++;
			usleep_range(10, 12);
		}

		val = exynos_elbi_read(exynos_pcie, PCIE_IRQ0);
		exynos_elbi_write(exynos_pcie, val, PCIE_IRQ0);
		val = exynos_elbi_read(exynos_pcie, PCIE_IRQ1);
		exynos_elbi_write(exynos_pcie, val, PCIE_IRQ1);
		val = exynos_elbi_read(exynos_pcie, PCIE_IRQ2);
		exynos_elbi_write(exynos_pcie, val, PCIE_IRQ2);

		/* enable IRQ */
		exynos_pcie_rc_enable_interrupts(pp, 1);

		/* setup ATU for cfg/mem outbound */
		busdev = EXYNOS_PCIE_ATU_BUS(1) | EXYNOS_PCIE_ATU_DEV(0) | EXYNOS_PCIE_ATU_FUNC(0);
		exynos_pcie_rc_prog_viewport_cfg0(pp, busdev);
		exynos_pcie_rc_prog_viewport_mem_outbound(pp);
	}

	return 0;
}

int exynos_pcie_rc_poweron(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci;
	struct dw_pcie_rp *pp;
	struct device *dev;
	u32 val, vendor_id, device_id;
	int ret;
	unsigned long flags;

	if (!exynos_pcie) {
		pr_err("%s: ch#%d PCIe device is not loaded\n", __func__, ch_num);

		return -ENODEV;
	}

	mutex_lock(&exynos_pcie->power_onoff_lock);

	pci = exynos_pcie->pci;
	pp = &pci->pp;
	dev = pci->dev;

	dev_dbg(dev, "start poweron, state: %d\n", exynos_pcie->state);
	logbuffer_log(exynos_pcie->log, "start poweron, state: %d", exynos_pcie->state);
	if (exynos_pcie->state == STATE_LINK_DOWN) {
		if (exynos_pcie->use_phy_isol_con)
			exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_BYPASS);

		if (exynos_pcie->use_pcieon_sleep) {
			dev_dbg(dev, "%s, pcie_is_linkup 1\n", __func__);
			pcie_is_linkup = 1;
		}
		ret = exynos_pcie_rc_clock_enable(pp, PCIE_ENABLE_CLOCK);
		dev_dbg(dev, "pcie clk enable, ret value = %d\n", ret);
		logbuffer_log(exynos_pcie->log, "pcie clk enable, ret value = %d", ret);

#if IS_ENABLED(CONFIG_CPU_IDLE)
		if (exynos_pcie->use_sicd) {
			dev_dbg(dev, "ip idle status: %d, index: %d\n",
				PCIE_IS_ACTIVE, exynos_pcie->idle_ip_index);
			exynos_update_ip_idle_status(exynos_pcie->idle_ip_index, PCIE_IS_ACTIVE);
		}
#endif
#if IS_ENABLED(CONFIG_PM_DEVFREQ)
		if (exynos_pcie->int_min_lock) {
			exynos_pm_qos_update_request(&exynos_pcie_int_qos[ch_num],
						     exynos_pcie->int_min_lock);
			dev_dbg(dev, "%s: pcie int_min_lock = %d\n",
				__func__, exynos_pcie->int_min_lock);
			logbuffer_log(exynos_pcie->log, "%s: pcie int_min_lock = %d", __func__,
				      exynos_pcie->int_min_lock);
		}
#endif
		/* Enable SysMMU */
		if (exynos_pcie->use_sysmmu)
			pcie_sysmmu_enable(pcie_ch_to_hsi(ch_num));

		pinctrl_select_state(exynos_pcie->pcie_pinctrl,
				     exynos_pcie->pin_state[PCIE_PIN_ACTIVE]);

		/* phy all power down clear */
		if (exynos_pcie->phy_ops.phy_all_pwrdn_clear)
			exynos_pcie->phy_ops.phy_all_pwrdn_clear(exynos_pcie, exynos_pcie->ch_num);

		/* make sure force_pclk_en disabled before link-up start */
		/* this setting will be done during phy_config
		 * writel(0x0C, exynos_pcie->phy_pcs_base + 0x0180);
		 */

		spin_lock_irqsave(&exynos_pcie->reg_lock, flags);
		exynos_pcie->state = STATE_LINK_UP_TRY;
		spin_unlock_irqrestore(&exynos_pcie->reg_lock, flags);

		enable_irq(pp->irq);

		if (exynos_pcie_rc_establish_link(pp)) {
			logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR, "pcie link up fail");
			goto poweron_fail;
		}

		exynos_pcie->sudden_linkdown = 0;
		exynos_pcie->cpl_timeout_recovery = 0;

		val = exynos_elbi_read(exynos_pcie, PCIE_STATE_HISTORY_CHECK);
		val &= ~(HISTORY_BUFFER_CONDITION_SEL);
		exynos_elbi_write(exynos_pcie, val, PCIE_STATE_HISTORY_CHECK);

		exynos_elbi_write(exynos_pcie, 0xffffffff, PCIE_STATE_POWER_S);
		exynos_elbi_write(exynos_pcie, 0xffffffff, PCIE_STATE_POWER_M);

		val = exynos_elbi_read(exynos_pcie, PCIE_STATE_HISTORY_CHECK);
		val |= HISTORY_BUFFER_ENABLE;
		exynos_elbi_write(exynos_pcie, val, PCIE_STATE_HISTORY_CHECK);

		spin_lock_irqsave(&exynos_pcie->reg_lock, flags);
		exynos_pcie->state = STATE_LINK_UP;
		spin_unlock_irqrestore(&exynos_pcie->reg_lock, flags);

		power_stats_update_up(exynos_pcie);

		dev_dbg(dev, "[%s] exynos_pcie->probe_ok : %d\n", __func__, exynos_pcie->probe_ok);
		if (!exynos_pcie->probe_ok) {
			exynos_pcie_rc_rd_own_conf(pp, PCI_VENDOR_ID, 4, &val);
			vendor_id = val & ID_MASK;
			device_id = (val >> 16) & ID_MASK;

			exynos_pcie->pci_dev = pci_get_device(vendor_id, device_id, NULL);
			if (!exynos_pcie->pci_dev) {
				logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
					       "Failed to get pci device");

				goto poweron_fail;
			}
			dev_dbg(dev, "(%s):ep_pci_device:vendor/device id = 0x%x\n", __func__, val);
			logbuffer_log(exynos_pcie->log, "ep_pci_device:vendor/device id = 0x%x",
				      val);

			pci_rescan_bus(exynos_pcie->pci_dev->bus);
			if (exynos_pcie->use_msi) {
				ret = exynos_pcie_rc_msi_init(pp);
				if (ret) {
					dev_err(dev, "%s: Failed MSI initialization(%d)\n",
						__func__, ret);
					mutex_unlock(&exynos_pcie->power_onoff_lock);
					return ret;
				}
			}

			if (pci_save_state(exynos_pcie->pci_dev)) {
				dev_err(dev, "Failed to save pcie state\n");

				goto poweron_fail;
			}
			exynos_pcie->pci_saved_configs =
				pci_store_saved_state(exynos_pcie->pci_dev);

			exynos_pcie->ep_pci_dev = exynos_pcie_get_pci_dev(&pci->pp);

#if IS_ENABLED(CONFIG_GS_S2MPU) || IS_ENABLED(CONFIG_EXYNOS_PCIE_IOMMU)
			if (exynos_pcie->s2mpu || exynos_pcie->use_sysmmu) {
				if (exynos_pcie->ep_device_type == EP_BCM_WIFI) {
					set_dma_ops(&exynos_pcie->ep_pci_dev->dev, &pcie_dma_ops);
					dev_info(dev, "Wifi DMA operations are changed\n");
					memcpy(&fake_dma_dev, exynos_pcie->pci->dev,
					       sizeof(fake_dma_dev));
				}
			}
#endif

			exynos_pcie->probe_ok = 1;
		} else if (exynos_pcie->probe_ok) {
			if (exynos_pcie->use_msi) {
				ret = exynos_pcie_rc_msi_init(pp);
				if (ret) {
					logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
						      "%s: Failed MSI initialization(%d)",
						      __func__, ret);
					mutex_unlock(&exynos_pcie->power_onoff_lock);
					return ret;
				}
			}

			if (pci_load_saved_state(exynos_pcie->pci_dev,
						 exynos_pcie->pci_saved_configs)) {
				logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
					       "Failed to load pcie state");

				goto poweron_fail;
			}
			pci_restore_state(exynos_pcie->pci_dev);
		}
	}

	dev_dbg(dev, "end poweron, state: %d\n", exynos_pcie->state);
	logbuffer_log(exynos_pcie->log, "end poweron, state: %d\n", exynos_pcie->state);
	mutex_unlock(&exynos_pcie->power_onoff_lock);

	if (exynos_pcie->ch_num == 1) {
		void __iomem *cmu_addr;
		u32 cmu_val;

		cmu_addr = ioremap(0x14400800, 0x4);
		cmu_val = readl(cmu_addr);

		/* Enable automatic clock gating on OSCCLK */
		cmu_val |= BIT(28);
		writel(cmu_val, cmu_addr);
		logbuffer_log(exynos_pcie->log, "Enable automatic clock gating on OSCCLK");

		iounmap(cmu_addr);
	}

	return 0;

poweron_fail:
	exynos_pcie->state = STATE_LINK_UP;
	mutex_unlock(&exynos_pcie->power_onoff_lock);
	exynos_pcie_rc_poweroff(exynos_pcie->ch_num);

	return -EPIPE;
}

void exynos_pcie_rc_poweroff(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci;
	struct dw_pcie_rp *pp;
	struct device *dev;
	unsigned long flags, flags1;
	u32 val;

	if (!exynos_pcie) {
		pr_err("%s: ch#%d PCIe device is not loaded\n", __func__, ch_num);
		return;
	}

	mutex_lock(&exynos_pcie->power_onoff_lock);

	pci = exynos_pcie->pci;
	pp = &pci->pp;
	dev = pci->dev;

	dev_dbg(dev, "start poweroff, state: %d\n", exynos_pcie->state);
	logbuffer_log(exynos_pcie->log, "start poweroff, state: %d", exynos_pcie->state);

	if (exynos_pcie->state == STATE_LINK_UP ||
	    exynos_pcie->state == STATE_LINK_DOWN_TRY) {
		spin_lock_irqsave(&exynos_pcie->reg_lock, flags1);
		exynos_pcie->state = STATE_LINK_DOWN_TRY;
		spin_unlock_irqrestore(&exynos_pcie->reg_lock, flags1);

		disable_irq(pp->irq);

		/* disable LINKDOWN irq */
		if (exynos_pcie->ip_ver == 0x982000) {
			/* only support EVT1.1 */
			val = exynos_elbi_read(exynos_pcie, PCIE_IRQ1_EN);
			val &= ~IRQ_LINKDOWN_ENABLE_EVT1_1;
			exynos_elbi_write(exynos_pcie, val, PCIE_IRQ1_EN);
		} else {
			val = exynos_elbi_read(exynos_pcie, PCIE_IRQ1_EN);
			val &= ~IRQ_LINK_DOWN_ENABLE;
			exynos_elbi_write(exynos_pcie, val, PCIE_IRQ1_EN);
		}

		spin_lock_irqsave(&exynos_pcie->conf_lock, flags);
		exynos_pcie->state = STATE_LINK_DOWN;
		exynos_pcie_rc_send_pme_turn_off(exynos_pcie);
		power_stats_update_down(exynos_pcie);

		/* Disable SysMMU */
		if (exynos_pcie->use_sysmmu)
			pcie_sysmmu_disable(pcie_ch_to_hsi(ch_num));

		/* Disable history buffer */
		val = exynos_elbi_read(exynos_pcie, PCIE_STATE_HISTORY_CHECK);
		val &= ~HISTORY_BUFFER_ENABLE;
		exynos_elbi_write(exynos_pcie, val, PCIE_STATE_HISTORY_CHECK);

		gpio_set_value(exynos_pcie->perst_gpio, 0);
		dev_dbg(dev, "%s: Set PERST to LOW, gpio val = %d\n",
			__func__, gpio_get_value(exynos_pcie->perst_gpio));
		logbuffer_log(exynos_pcie->log, "%s: Set PERST to LOW, gpio val = %d", __func__,
			      gpio_get_value(exynos_pcie->perst_gpio));

		/* LTSSM disable */
		exynos_elbi_write(exynos_pcie, PCIE_ELBI_LTSSM_DISABLE, PCIE_APP_LTSSM_ENABLE);

		/* force SOFT_PWR_RESET */
		val = exynos_elbi_read(exynos_pcie, PCIE_SOFT_RESET);
		val &= ~SOFT_PWR_RESET;
		exynos_elbi_write(exynos_pcie, val, PCIE_SOFT_RESET);
		udelay(20);
		val |= SOFT_PWR_RESET;
		exynos_elbi_write(exynos_pcie, val, PCIE_SOFT_RESET);

		/* phy all power down */
		if (exynos_pcie->phy_ops.phy_all_pwrdn)
			exynos_pcie->phy_ops.phy_all_pwrdn(exynos_pcie, exynos_pcie->ch_num);

		spin_unlock_irqrestore(&exynos_pcie->conf_lock, flags);

		exynos_pcie_rc_phy_clock_enable(pp, PCIE_DISABLE_CLOCK);
		exynos_pcie_rc_clock_enable(pp, PCIE_DISABLE_CLOCK);
		exynos_pcie->atu_ok = 0;

		if (!IS_ERR(exynos_pcie->pin_state[PCIE_PIN_IDLE]))
			pinctrl_select_state(exynos_pcie->pcie_pinctrl,
					     exynos_pcie->pin_state[PCIE_PIN_IDLE]);

#if IS_ENABLED(CONFIG_PM_DEVFREQ)
		if (exynos_pcie->int_min_lock) {
			exynos_pm_qos_update_request(&exynos_pcie_int_qos[ch_num], 0);
			dev_dbg(dev, "%s: pcie int_min_lock = %d\n",
				__func__, exynos_pcie->int_min_lock);
		}
#endif
#if IS_ENABLED(CONFIG_CPU_IDLE)
		if (exynos_pcie->use_sicd) {
			dev_dbg(dev, "%s, ip idle status: %d, idle_ip_index: %d\n",
				__func__, PCIE_IS_IDLE, exynos_pcie->idle_ip_index);
			exynos_update_ip_idle_status(exynos_pcie->idle_ip_index, PCIE_IS_IDLE);
		}
#endif
		if (exynos_pcie->use_phy_isol_con)
			exynos_pcie_phy_isolation(exynos_pcie,
						  PCIE_PHY_ISOLATION);
	}

	if (exynos_pcie->use_pcieon_sleep) {
		dev_dbg(dev, "%s, pcie_is_linkup 0\n", __func__);
		logbuffer_log(exynos_pcie->log, "%s, pcie_is_linkup 0", __func__);
		pcie_is_linkup = 0;
	}

	dev_dbg(dev, "end poweroff, state: %d\n", exynos_pcie->state);
	logbuffer_log(exynos_pcie->log, "end poweroff, state: %d\n", exynos_pcie->state);

	mutex_unlock(&exynos_pcie->power_onoff_lock);
}

void exynos_pcie_pm_suspend(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	unsigned long flags;

	logbuffer_log(exynos_pcie->log, "pm_suspend api called");
	if (exynos_pcie->state == STATE_LINK_DOWN) {
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_INFO, "RC%d already off",
			       exynos_pcie->ch_num);

		return;
	}

	spin_lock_irqsave(&exynos_pcie->conf_lock, flags);
	exynos_pcie->state = STATE_LINK_DOWN_TRY;
	spin_unlock_irqrestore(&exynos_pcie->conf_lock, flags);

	exynos_pcie_rc_poweroff(ch_num);
}
EXPORT_SYMBOL_GPL(exynos_pcie_pm_suspend);

int exynos_pcie_pm_resume(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];

	logbuffer_log(exynos_pcie->log, "pm_resume api called");
	return exynos_pcie_rc_poweron(ch_num);
}
EXPORT_SYMBOL_GPL(exynos_pcie_pm_resume);

bool exynos_pcie_rc_get_sudden_linkdown_state(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];

	logbuffer_log(exynos_pcie->log, "get_sudden_linkdown state=%d",
		      exynos_pcie->sudden_linkdown);
	return exynos_pcie->sudden_linkdown;
}
EXPORT_SYMBOL(exynos_pcie_rc_get_sudden_linkdown_state);

void exynos_pcie_rc_set_sudden_linkdown_state(int ch_num, bool recovery)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];

	pr_err("[%s] set sudden_linkdown_state to recovery_on\n", __func__);
	exynos_pcie->sudden_linkdown = recovery;
}
EXPORT_SYMBOL(exynos_pcie_rc_set_sudden_linkdown_state);

bool exynos_pcie_rc_get_cpl_timeout_state(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];

	return exynos_pcie->cpl_timeout_recovery;
}
EXPORT_SYMBOL(exynos_pcie_rc_get_cpl_timeout_state);

void exynos_pcie_rc_set_cpl_timeout_state(int ch_num, bool recovery)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];

	pr_err("set cpl_timeout_recovery to %d for ch_num:%d\n", recovery, ch_num);
	exynos_pcie->cpl_timeout_recovery = recovery;
}
EXPORT_SYMBOL(exynos_pcie_rc_set_cpl_timeout_state);

/* get EP pci_dev structure of BUS */
static struct pci_dev *exynos_pcie_get_pci_dev(struct dw_pcie_rp *pp)
{
	int domain_num;
	struct pci_bus *ep_pci_bus;
	struct pci_dev *ep_pci_dev;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	u32 val;

	if (exynos_pcie->ep_pci_dev)
		return exynos_pcie->ep_pci_dev;

	/* Get EP vendor/device ID to get pci_dev structure */
	domain_num = exynos_pcie->pci_dev->bus->domain_nr;
	ep_pci_bus = pci_find_bus(domain_num, 1);

	exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0, PCI_VENDOR_ID, 4, &val);

	ep_pci_dev = pci_get_device(val & ID_MASK, (val >> 16) & ID_MASK, NULL);

	return ep_pci_dev;
}

int exynos_pcie_l1_exit(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	u32 count = 0, ret = 0, val = 0;
	unsigned long flags;

	if (exynos_pcie->ep_device_type != EP_BCM_WIFI) {
		pr_err("%s: EP is not EP_BCM_WIFI (not support l1_exit)\n", __func__);

		return -EPIPE;
	}

	spin_lock_irqsave(&exynos_pcie->pcie_l1_exit_lock, flags);

	if (exynos_pcie->l1ss_ctrl_id_state == 0) {
		/* Set s/w L1 exit mode */
		/* 1. Set app_req_exit_l1 Signal */
		exynos_elbi_write(exynos_pcie, 0x1, PCIE_APP_REQ_EXIT_L1);
		/* 2. exit_l1_control_mode[0:0] - 0x0: SW mode (0x1: HW mode) */
		val = exynos_elbi_read(exynos_pcie, PCIE_APP_REQ_EXIT_L1_MODE);
		val &= ~APP_REQ_EXIT_L1_MODE;
		exynos_elbi_write(exynos_pcie, val, PCIE_APP_REQ_EXIT_L1_MODE);

		/* Max timeout = 3ms (300 * 10us) */
		while (count < MAX_L1_EXIT_TIMEOUT) {
			val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP)
			      & PCIE_ELBI_LTSSM_STATE_MASK;
			if (val == S_L0)
				break;
			count++;
			udelay(10);
		}

		if (count >= MAX_L1_EXIT_TIMEOUT) {
			pr_err("%s: cannot change to L0(LTSSM = 0x%x, cnt = %d)\n",
			       __func__, val, count);
			ret = -EPIPE;
		}

		/* Set h/w L1 exit mode */
		/* 1. exit_l1_control_mode[0:0] - 0x1: HW mode (0x0: SW mode) */
		val = exynos_elbi_read(exynos_pcie, PCIE_APP_REQ_EXIT_L1_MODE);
		val |= APP_REQ_EXIT_L1_MODE;
		exynos_elbi_write(exynos_pcie, val, PCIE_APP_REQ_EXIT_L1_MODE);
		/* 2. Reset app_req_exit_l1 Signal */
		exynos_elbi_write(exynos_pcie, 0x0, PCIE_APP_REQ_EXIT_L1);
	}

	spin_unlock_irqrestore(&exynos_pcie->pcie_l1_exit_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pcie_l1_exit);

static int exynos_pcie_rc_set_l1ss(int enable, struct dw_pcie_rp *pp, int id)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct device *dev = pci->dev;
	u32 val;
	unsigned long flags;
	int domain_num;
	struct pci_bus *ep_pci_bus;
	u32 exp_cap_off = PCIE_CAP_OFFSET;

	/* This function is only working with the devices which support L1SS */
	if (exynos_pcie->ep_device_type != EP_SAMSUNG_MODEM &&
	    exynos_pcie->ep_device_type != EP_BCM_WIFI &&
	    exynos_pcie->ep_device_type != EP_QC_WIFI) {
		dev_err(dev, "Can't set L1SS!!! (EP: L1SS not supported)\n");

		return -EINVAL;
	}

	dev_dbg(dev, "%s:L1SS_START: l1ss_ctrl_id_state = 0x%x\n",
		__func__, exynos_pcie->l1ss_ctrl_id_state);
	dev_dbg(dev, "%s:\tid = 0x%x, enable=%d, exynos_pcie=%pK\n",
		__func__, id, enable, exynos_pcie);

	if (exynos_pcie->state != STATE_LINK_UP || exynos_pcie->atu_ok == 0) {
		spin_lock_irqsave(&exynos_pcie->conf_lock, flags);

		if (enable)
			exynos_pcie->l1ss_ctrl_id_state &= ~(id);
		else
			exynos_pcie->l1ss_ctrl_id_state |= id;

		spin_unlock_irqrestore(&exynos_pcie->conf_lock, flags);

		dev_dbg(dev, "%s:state = 0x%x, id = 0x%x: not needed(This will be set later)\n",
			__func__, exynos_pcie->l1ss_ctrl_id_state, id);

		return -1;
	} else {
		exynos_pcie->ep_l1ss_cap_off =
			pci_find_ext_capability(exynos_pcie->ep_pci_dev, PCI_EXT_CAP_ID_L1SS);
		exynos_pcie->ep_link_ctrl_off = exynos_pcie->ep_pci_dev->pcie_cap + PCI_EXP_LNKCTL;
		exynos_pcie->ep_l1ss_ctrl1_off = exynos_pcie->ep_l1ss_cap_off + PCI_L1SS_CTL1;
		exynos_pcie->ep_l1ss_ctrl2_off = exynos_pcie->ep_l1ss_cap_off + PCI_L1SS_CTL2;
	}

	/* get the domain_num & ep_pci_bus of EP device */
	domain_num = exynos_pcie->pci_dev->bus->domain_nr;
	ep_pci_bus = pci_find_bus(domain_num, 1);
	dev_dbg(dev, "%s:[DBG] domain_num = %d, ep_pci_bus = %pK\n",
		__func__, domain_num, ep_pci_bus);

	spin_lock_irqsave(&exynos_pcie->conf_lock, flags);
	if (enable) {	/* enable == 1 */
		exynos_pcie->l1ss_ctrl_id_state &= ~(id);

		if (exynos_pcie->l1ss_ctrl_id_state == 0) {
			/* [RC & EP] enable L1SS & ASPM */
			if (exynos_pcie->ep_device_type == EP_SAMSUNG_MODEM) {
				dev_dbg(dev, "%s: #1 enalbe CP L1.2\n", __func__);

				/* 1) [RC] enable L1SS */
				exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, &val);
				/* Actual TCOMMON is 42usec (val = 0x2a << 8) */
				val |= PORT_LINK_TCOMMON_32US
					| PORT_LINK_L1SS_ENABLE;
				dev_dbg(dev, "CPen:1RC:L1SS_CTRL(0x19C) = 0x%x\n", val);
				exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, val);

				/* [RC] set TPOWERON */
				/* Set TPOWERON value for RC: 90->200 usec */
				exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL2, 4,
							   PORT_LINK_TPOWERON_200US);

				/* exynos_pcie_rc_wr_own_conf(pp,
				 *	PCIE_L1_SUBSTATES_OFF, 4,
				 *	PCIE_L1_SUB_VAL);
				 */

				/* [RC] set LTR_EN */
				exynos_pcie_rc_wr_own_conf(pp, exp_cap_off + PCI_EXP_DEVCTL2,
							   4, PCI_EXP_DEVCTL2_LTR_EN);

				/* [EP] set LTR_EN (reg_addr = 0x98) */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
							     exp_cap_off + PCI_EXP_DEVCTL2, 4,
							     &val);
				val |= PCI_EXP_DEVCTL2_LTR_EN;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     exp_cap_off + PCI_EXP_DEVCTL2, 4, val);

				/* [EP] set TPOWERON */
				/* Set TPOWERON value for EP: 90->200 usec */
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     exynos_pcie->ep_l1ss_ctrl2_off, 4,
							     PORT_LINK_TPOWERON_200US);

				/* [EP] set Entrance latency */
				/* Set L1.2 Enterance Latency for EP: 64 usec */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
							     PCIE_ACK_F_ASPM_CONTROL, 4, &val);
				val &= ~PCIE_L1_ENTERANCE_LATENCY;
				val |= PCIE_L1_ENTERANCE_LATENCY_64us;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     PCIE_ACK_F_ASPM_CONTROL, 4, val);

				/* 2) [EP] enable L1SS */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
							     exynos_pcie->ep_l1ss_ctrl1_off, 4, &val);
				val |= PORT_LINK_L1SS_ENABLE;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     exynos_pcie->ep_l1ss_ctrl1_off, 4, val);
				dev_dbg(dev, "CPen:2EP:L1SS_CTRL(0x19C)=0x%x\n", val);

				/* 3) [RC] enable ASPM */
				exynos_pcie_rc_rd_own_conf(pp, exp_cap_off +
							   PCI_EXP_LNKCTL, 4, &val);
				val &= ~PCI_EXP_LNKCTL_ASPMC;
				val |= PCI_EXP_LNKCTL_CCC | PCI_EXP_LNKCTL_ASPM_L1;
				exynos_pcie_rc_wr_own_conf(pp, exp_cap_off +
							   PCI_EXP_LNKCTL, 4, val);
				dev_dbg(dev, "CPen:3RC:ASPM(0x70+16)=0x%x\n", val);

				/* 4) [EP] enable ASPM */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
							     exynos_pcie->ep_link_ctrl_off, 4, &val);
				val |= PCI_EXP_LNKCTL_CCC | PCI_EXP_LNKCTL_CLKREQ_EN |
					PCI_EXP_LNKCTL_ASPM_L1;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     exynos_pcie->ep_link_ctrl_off, 4, val);
				dev_dbg(dev, "CPen:4EP:ASPM(0x80)=0x%x\n", val);
			} else if (exynos_pcie->ep_device_type == EP_BCM_WIFI) {
				dev_dbg(dev, "%s: #2 enable WIFI L1.2\n", __func__);

				/* enable sequence:
				 * 1. PCIPM RC
				 * 2. PCIPM EP
				 * 3. ASPM RC
				 * 4. ASPM EP
				 */

				/* 1. to enable PCIPM RC */
				/* [RC:set value] TPowerOn(130 usec) */
				exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL2, 4,
							   PORT_LINK_TPOWERON_130US);

				/* [RC:set value] TPowerOff(3 us), T_L1.2(10 us), T_PCLKACK(2 us) */
				exynos_pcie_rc_wr_own_conf(pp, PCIE_L1_SUBSTATES_OFF, 4,
							   PCIE_L1_SUB_VAL);

				/* [RC:set enable bit] LTR Mechanism Enable */
				exynos_pcie_rc_rd_own_conf(pp, exp_cap_off + PCI_EXP_DEVCTL2,
							   4, &val);
				val |= PCI_EXP_DEVCTL2_LTR_EN;
				exynos_pcie_rc_wr_own_conf(pp, exp_cap_off + PCI_EXP_DEVCTL2,
							   4, val);

				/* [RC:set value] LTR_L1.2_Threshold(160 us) and TCommon(42 us)
				 *	Actual TCommon is 42usec (val = (0xa | 0x20) << 8)
				 * [RC:enable] L1SS_ENABLE(0xf)
				 */
				exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, &val);
				val |= PORT_LINK_L12_LTR_THRESHOLD | PORT_LINK_TCOMMON_32US |
				       PORT_LINK_L1SS_ENABLE;
				exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, val);
				dev_dbg(dev, "WIFIen:1RC:L1SS_CTRL(0x19C)=0x%x\n", val);

				/* 2. to enable PCIPM EP */
				/* [EP:set value] TPowerOn(130 usec) */
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0, WIFI_L1SS_CONTROL2,
							     4, PORT_LINK_TPOWERON_130US);

				/* [EP:set value] LTR Latency(max snoop(3 ms)/no-snoop(3 ms))
				 * Reported_LTR value in LTR message
				 */
				val = MAX_NO_SNOOP_LAT_SCALE_MS | MAX_NO_SNOOP_LAT_VALUE_3 |
					      MAX_SNOOP_LAT_SCALE_MS | MAX_SNOOP_LAT_VALUE_3;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     WIFI_L1SS_LTR_LATENCY, 4, val);

				/* [EP:set enable bit] LTR Mechanism Enable */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
							     WIFI_PCI_EXP_DEVCTL2, 4, &val);
				val |= PCI_EXP_DEVCTL2_LTR_EN;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     WIFI_PCI_EXP_DEVCTL2, 4, val);

				/* [EP:set values] LTR_L1.2_Threshold(160 us) and TCommon(10 us)
				 * [EP:enable] WIFI_PM_ENALKBE(0xf)
				 */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0, WIFI_L1SS_CONTROL,
							     4, &val);
				val |= PORT_LINK_L12_LTR_THRESHOLD | WIFI_COMMON_RESTORE_TIME |
				       WIFI_ALL_PM_ENABEL;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0, WIFI_L1SS_CONTROL,
							     4, val);
				dev_dbg(dev, "WIFIen:2EP:L1SS_CTRL(0x248)=0x%x\n", val);

				/* 3. to enable ASPM RC */
				exynos_pcie_rc_rd_own_conf(pp, exp_cap_off + PCI_EXP_LNKCTL,
							   4, &val);
				val &= ~PCI_EXP_LNKCTL_ASPMC;
				/* PCI_EXP_LNKCTL_CCC: Common Clock Configuration */
				val |= PCI_EXP_LNKCTL_CCC | PCI_EXP_LNKCTL_ASPM_L1;
				exynos_pcie_rc_wr_own_conf(pp, exp_cap_off + PCI_EXP_LNKCTL,
							   4, val);
				dev_dbg(dev, "WIFIen:3RC:ASPM(0x70+16)=0x%x\n", val);

				/* 4. to enable ASPM EP */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0, WIFI_L1SS_LINKCTRL,
							     4, &val);
				val &= ~(WIFI_ASPM_CONTROL_MASK);
				val |= WIFI_CLK_REQ_EN | WIFI_USE_SAME_REF_CLK |
				       WIFI_ASPM_L1_ENTRY_EN;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0, WIFI_L1SS_LINKCTRL,
							     4, val);
				dev_dbg(dev, "WIFIen:4EP:ASPM(0xBC)=0x%x\n", val);
			} else if (exynos_pcie->ep_device_type == EP_QC_WIFI) {
				dev_dbg(dev, "%s: #2 enable WIFI L1.2\n", __func__);

				/* enable sequence:
				 * 1. PCIPM RC
				 * 2. PCIPM EP
				 * 3. ASPM RC
				 * 4. ASPM EP
				 */

				/* 1. to enable PCIPM RC */
				/* [RC:set value] TPowerOn(10 usec) */
				exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL2, 4,
							   PORT_LINK_TPOWERON_10US);

				/* [RC:set enable bit] LTR Mechanism Enable */
				exynos_pcie_rc_rd_own_conf(pp, exp_cap_off + PCI_EXP_DEVCTL2,
							   4, &val);
				val |= PCI_EXP_DEVCTL2_LTR_EN;
				exynos_pcie_rc_wr_own_conf(pp, exp_cap_off + PCI_EXP_DEVCTL2,
							   4, val);

				/* [RC:set value] LTR_L1.2_Threshold(150(0x96) us)
				 * and TCommon is 42usec (val = (0xa | 0x20) << 8)
				 * [RC:enable] L1SS_ENABLE(0xf)
				 */
				exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, &val);
				val |= WIFI_QC_L12_LTR_THRESHOLD | PORT_LINK_TCOMMON_32US |
				       PORT_LINK_L1SS_ENABLE;
				exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, val);
				dev_dbg(dev, "%s: WIFIen:1RC:L1SS_CTRL(0x19C)=0x%x\n",
						__func__, val);

				/* 2. to enable PCIPM EP */
				/* [EP:set value] TPowerOn(10 usec) */
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
						exynos_pcie->ep_l1ss_ctrl2_off,
						4, PORT_LINK_TPOWERON_10US);
				dev_dbg(dev, "%s: WIFIen:2EP:L1SS_CTRL2(0x%x)=0x%x\n",
						__func__, exynos_pcie->ep_l1ss_ctrl2_off,
						PORT_LINK_TPOWERON_10US);

				/* [EP:set enable bit] LTR Mechanism Enable */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
							     exp_cap_off+PCI_EXP_DEVCTL2, 4, &val);
				val |= PCI_EXP_DEVCTL2_LTR_EN;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     exp_cap_off+PCI_EXP_DEVCTL2, 4, val);
				dev_dbg(dev, "%s: WIFIen:2EP:EXP_DEVCTL2(0x%x)=0x%x\n",
						__func__, exp_cap_off+PCI_EXP_DEVCTL2, val);

				/* [EP:set values] LTR_L1.2_Threshold(150 us) and TCommon(0 us)
				 * [EP:enable] WIFI_PM_ENALKBE(0xf)
				 */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
						exynos_pcie->ep_l1ss_ctrl1_off, 4, &val);
				val |= WIFI_QC_L12_LTR_THRESHOLD | PORT_LINK_L1SS_ENABLE;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
						exynos_pcie->ep_l1ss_ctrl1_off, 4, val);
				dev_dbg(dev, "%s: WIFIen:2EP:L1SS_CTRL(0x%x)=0x%x\n",
						__func__, exynos_pcie->ep_l1ss_ctrl1_off, val);

				/* 3. to enable ASPM RC */
				exynos_pcie_rc_rd_own_conf(pp, exp_cap_off + PCI_EXP_LNKCTL,
							   4, &val);
				val &= ~PCI_EXP_LNKCTL_ASPMC;
				/* PCI_EXP_LNKCTL_CCC: Common Clock Configuration */
				val |= PCI_EXP_LNKCTL_CCC | PCI_EXP_LNKCTL_ASPM_L1;
				exynos_pcie_rc_wr_own_conf(pp, exp_cap_off + PCI_EXP_LNKCTL,
							   4, val);
				dev_dbg(dev, "%s: WIFIen:3RC:ASPM(0x70+16)=0x%x\n", __func__, val);

				/* 4. to enable ASPM EP */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
						exp_cap_off+PCI_EXP_LNKCTL, 4, &val);
				val |= PCI_EXP_LNKCTL_CCC | PCI_EXP_LNKCTL_CLKREQ_EN |
					PCI_EXP_LNKCTL_ASPM_L1;
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
						exp_cap_off+PCI_EXP_LNKCTL, 4, val);
				dev_dbg(dev, "%s: WIFIen:4EP:ASPM(0x%x)=0x%x\n",
						__func__,exp_cap_off+PCI_EXP_LNKCTL, val);
			} else {
				dev_err(dev, "[ERR] EP: L1SS not supported\n");
			}
		}
	} else {	/* enable == 0 */
		if (exynos_pcie->l1ss_ctrl_id_state) {
			exynos_pcie->l1ss_ctrl_id_state |= id;
		} else {
			exynos_pcie->l1ss_ctrl_id_state |= id;

			if (exynos_pcie->ep_device_type == EP_SAMSUNG_MODEM) {
				dev_dbg(dev, "%s: #3 disable CP L1.2\n", __func__);

				/* 1) [EP] disable ASPM */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
							     exynos_pcie->ep_link_ctrl_off, 4, &val);
				val &= ~(PCI_EXP_LNKCTL_ASPMC);
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     exynos_pcie->ep_link_ctrl_off, 4, val);
				dev_dbg(dev, "CPdis:1EP:ASPM(0x80)=0x%x\n", val);

				/* 2) [RC] disable ASPM */
				exynos_pcie_rc_rd_own_conf(pp, exp_cap_off +
							   PCI_EXP_LNKCTL, 4, &val);
				val &= ~PCI_EXP_LNKCTL_ASPMC;
				exynos_pcie_rc_wr_own_conf(pp, exp_cap_off +
							   PCI_EXP_LNKCTL, 4, val);
				dev_dbg(dev, "CPdis:2RC:ASPM(0x70+16)=0x%x\n", val);

				/* 3) [EP] disable L1SS */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
							     exynos_pcie->ep_l1ss_ctrl1_off, 4, &val);
				val &= ~(PORT_LINK_L1SS_ENABLE);
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
							     exynos_pcie->ep_l1ss_ctrl1_off, 4, val);
				dev_dbg(dev, "CPdis:3EP:L1SS_CTRL(0x19C)=0x%x\n", val);

				/* 4) [RC] disable L1SS */
				exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, &val);
				val &= ~(PORT_LINK_L1SS_ENABLE);
				exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, val);
				dev_dbg(dev, "CPdis:4RC:L1SS_CTRL(0x19C)=0x%x\n", val);
			} else if (exynos_pcie->ep_device_type == EP_BCM_WIFI) {
				dev_dbg(dev, "%s: #4 disable WIFI L1.2\n", __func__);

				/* disable sequence:
				 * 1. ASPM EP
				 * 2. ASPM RC
				 * 3. PCIPM EP
				 * 4. PCIPM RC
				 */
				/* 1) [EP] disable ASPM */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0, WIFI_L1SS_LINKCTRL,
							     4, &val);
				val &= ~(WIFI_ASPM_CONTROL_MASK);
				/* val |= WIFI_CLK_REQ_EN | WIFI_USE_SAME_REF_CLK; */
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0, WIFI_L1SS_LINKCTRL,
							     4, val);
				dev_dbg(pci->dev, "WIFIdis:1EP:ASPM(0xBC)=0x%x\n", val);

				/* 2) [RC] disable ASPM */
				exynos_pcie_rc_rd_own_conf(pp, exp_cap_off + PCI_EXP_LNKCTL,
							   4, &val);
				val &= ~PCI_EXP_LNKCTL_ASPMC;
				exynos_pcie_rc_wr_own_conf(pp, exp_cap_off + PCI_EXP_LNKCTL,
							   4, val);
				dev_dbg(pci->dev, "WIFIdis:2RC:ASPM(0x70+16)=0x%x\n", val);

				/* 3) [EP] disable L1SS */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0, WIFI_L1SS_CONTROL,
							     4, &val);
				val &= ~(WIFI_ALL_PM_ENABEL);
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0, WIFI_L1SS_CONTROL,
							     4, val);
				dev_dbg(pci->dev, "WIFIdis:3EP:L1SS_CTRL(0x248)=0x%x\n", val);

				/* 4) [RC] disable L1SS */
				exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, &val);
				val &= ~(PORT_LINK_L1SS_ENABLE);
				exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, val);
				dev_dbg(pci->dev, "WIFIdis:4RC:L1SS_CTRL(0x19C)=0x%x\n", val);
			} else if (exynos_pcie->ep_device_type == EP_QC_WIFI) {
				dev_dbg(dev, "%s: #4 disable WIFI L1.2\n", __func__);

				/* disable sequence:
				 * 1. ASPM EP
				 * 2. ASPM RC
				 * 3. PCIPM EP
				 * 4. PCIPM RC
				 */
				/* 1) [EP] disable ASPM */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
						exp_cap_off+PCI_EXP_LNKCTL, 4, &val);
				val &= ~(PCI_EXP_LNKCTL_ASPMC);
				/* val |= WIFI_CLK_REQ_EN | WIFI_USE_SAME_REF_CLK; */
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
						exp_cap_off+PCI_EXP_LNKCTL, 4, val);
				dev_dbg(pci->dev, "%s: WIFIdis:1EP:ASPM(0x%x)=0x%x\n",
						__func__,exp_cap_off+PCI_EXP_LNKCTL,val);

				/* 2) [RC] disable ASPM */
				exynos_pcie_rc_rd_own_conf(pp, exp_cap_off + PCI_EXP_LNKCTL,
							   4, &val);
				val &= ~PCI_EXP_LNKCTL_ASPMC;
				exynos_pcie_rc_wr_own_conf(pp, exp_cap_off + PCI_EXP_LNKCTL,
							   4, val);
				dev_dbg(pci->dev, "%s: WIFIdis:2RC:ASPM(0x70+16)=0x%x\n",
						__func__, val);

				/* 3) [EP] disable L1SS */
				exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0,
						exynos_pcie->ep_l1ss_ctrl1_off, 4, &val);
				val &= ~(PORT_LINK_L1SS_ENABLE);
				exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0,
						exynos_pcie->ep_l1ss_ctrl1_off, 4, val);
				dev_dbg(pci->dev, "%s: WIFIdis:3EP:L1SS_CTRL(0x%x)=0x%x\n",
						__func__, exynos_pcie->ep_l1ss_ctrl1_off,val);

				/* 4) [RC] disable L1SS */
				exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, &val);
				val &= ~(PORT_LINK_L1SS_ENABLE);
				exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL, 4, val);
				dev_dbg(pci->dev, "%s: WIFIdis:4RC:L1SS_CTRL(0x19C)=0x%x\n",
						__func__, val);
			} else {
				dev_err(dev, "[ERR] EP: L1SS not supported\n");
			}
		}
	}

	spin_unlock_irqrestore(&exynos_pcie->conf_lock, flags);

	dev_dbg(dev, "%s:L1SS_END(l1ss_ctrl_id_state=0x%x, id=0x%x, enable=%d)\n",
		__func__, exynos_pcie->l1ss_ctrl_id_state, id, enable);

	return 0;
}

int exynos_pcie_rc_l1ss_ctrl(int enable, int id, int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;

	if (!exynos_pcie->use_l1ss) {
		pr_err("%s: 'use_l1ss' is false in DT(not support L1.2)\n", __func__);

		return -EINVAL;
	}

	if (pp)
		return	exynos_pcie_rc_set_l1ss(enable, pp, id);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_l1ss_ctrl);

/* to support CP driver */
int exynos_pcie_poweron(int ch_num, int spd)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;

	dev_dbg(pci->dev, "%s requested with link speed GEN%d\n", __func__, spd);
	exynos_pcie->max_link_speed = spd;

	return exynos_pcie_rc_poweron(ch_num);
}
EXPORT_SYMBOL_GPL(exynos_pcie_poweron);

void exynos_pcie_poweroff(int ch_num)
{
	return exynos_pcie_rc_poweroff(ch_num);
}
EXPORT_SYMBOL_GPL(exynos_pcie_poweroff);

/* PCIe link status check function */
int exynos_pcie_rc_chk_link_status(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci;
	struct device *dev;
	unsigned long flags;

	u32 val;
	int link_status;

	if (!exynos_pcie) {
		pr_err("%s: ch#%d PCIe device is not loaded\n", __func__, ch_num);

		return -ENODEV;
	}
	pci = exynos_pcie->pci;
	dev = pci->dev;

	if (exynos_pcie->state == STATE_LINK_DOWN)
		return 0;

	if (exynos_pcie->cpl_timeout_recovery) {
		spin_lock_irqsave(&exynos_pcie->reg_lock, flags);
		exynos_pcie->state = STATE_LINK_DOWN;
		spin_unlock_irqrestore(&exynos_pcie->reg_lock, flags);
		return 0;
	}

	if (exynos_pcie->ep_device_type == EP_SAMSUNG_MODEM) {
		val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP)
		      & PCIE_ELBI_LTSSM_STATE_MASK;
		if (val >= S_RCVRY_LOCK && val <= S_L1_IDLE) {
			link_status = 1;
		} else {
			dev_err(dev, "Check unexpected state - H/W:0x%x, S/W:%d\n",
				val, exynos_pcie->state);

			exynos_pcie_rc_print_link_history(&pci->pp);

			spin_lock_irqsave(&exynos_pcie->reg_lock, flags);
			exynos_pcie->state = STATE_LINK_DOWN;
			spin_unlock_irqrestore(&exynos_pcie->reg_lock, flags);
			link_status = 0;
		}

		return link_status;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_chk_link_status);

/* LINK SPEED: check and change */
int exynos_pcie_rc_check_link_speed(int ch_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	u32 current_speed = 0;

	if (exynos_pcie->state != STATE_LINK_UP) {
		dev_err(pci->dev, "Link is not up\n");

		return -EINVAL;
	}

	exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_CTRL_STAT, 4, &current_speed);
	current_speed = current_speed >> 16;
	current_speed &= PCIE_LINK_CTRL_LINK_SPEED_MASK;
	dev_info(pci->dev, "(%s) Current link speed(0x80): GEN%d\n", __func__, (int)current_speed);

	return current_speed;
}

int exynos_pcie_rc_change_link_speed(int ch_num, int target_speed)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct pci_bus *ep_pci_bus;
	int i;
	u32 val, current_speed, new_speed;

	if (exynos_pcie->state != STATE_LINK_UP) {
		dev_err(pci->dev, "Link is not up\n");
		return -EINVAL;
	}

	if (target_speed > 3 || target_speed < 1) {
		dev_err(pci->dev, "Invalid target speed: Unable to change\n");

		return -EINVAL;
	}

	current_speed = exynos_pcie_rc_check_link_speed(ch_num);
	if (current_speed == target_speed) {
		dev_err(pci->dev, "Already GEN%d(current), target: GEN%d\n",
			current_speed, target_speed);

		return -EINVAL;
	}

	/* make sure that the link state is L0 by accessing ep config register
	 * such as 'PCI_VENDOR_ID'.
	 */
	ep_pci_bus = pci_find_bus(exynos_pcie->pci_dev->bus->domain_nr, 1);
	exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0, PCI_VENDOR_ID, 4, &val);

	/* modify registers to change link speed:
	 * 1. PCIe_LINK_CTRL2_STAT2(offset: 0xA0)
	 */
	exynos_pcie_rc_rd_other_conf(pp, ep_pci_bus, 0, PCIE_LINK_CTRL2_STAT2, 4, &val);
	val = val & PCIE_LINK_CTRL2_TARGET_LINK_SPEED_MASK;
	val = val | 0x3;
	exynos_pcie_rc_wr_other_conf(pp, ep_pci_bus, 0, PCIE_LINK_CTRL2_STAT2, 4, val);

	exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_CTRL2_STAT2, 4, &val);
	val = val & PCIE_LINK_CTRL2_TARGET_LINK_SPEED_MASK;
	val = val | target_speed;
	exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_CTRL2_STAT2, 4, val);

	/* modify registers to change link speed:
	 * 2. PCIE_LINK_WIDTH_SPEED_CONTROL(offset: 0x80C)
	 */
	exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, &val);
	val = val & DIRECT_SPEED_CHANGE_ENABLE_MASK;
	exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, val);

	exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, &val);
	val = val | DIRECT_SPEED_CHANGE_ENABLE;
	exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, val);

	for (i = 0; i < MAX_TIMEOUT_SPEEDCHANGE; i++) {
		val = exynos_elbi_read(exynos_pcie, PCIE_ELBI_RDLH_LINKUP)
		      & PCIE_ELBI_LTSSM_STATE_MASK;
		if (val >= S_L0 && val <= S_L1_IDLE)
			break;
		usleep_range(80, 100);
	}

	for (i = 0; i < MAX_TIMEOUT_SPEEDCHANGE; i++) {
		exynos_pcie_rc_rd_own_conf(pp, PCIE_LINK_CTRL_STAT, 4, &new_speed);
		new_speed = new_speed >> 16;
		new_speed &= PCIE_LINK_CTRL_LINK_SPEED_MASK;
		if (new_speed == target_speed)
			break;
		usleep_range(80, 100);
	}

	if (new_speed != target_speed) {
		dev_err(pci->dev, "Fail: Unable to change to GEN%d\n", target_speed);

		return -EINVAL;
	}

	dev_info(pci->dev, "Link Speed Changed: from GEN%d to GEN%d\n", current_speed, new_speed);

	return 0;
}

int exynos_pcie_register_event(struct exynos_pcie_register_event *reg)
{
	int ret = 0;
	struct dw_pcie_rp *pp;
	struct exynos_pcie *exynos_pcie;
	struct dw_pcie *pci;

	if (!reg) {
		pr_err("PCIe: Event registration is NULL\n");

		return -ENODEV;
	}
	if (!reg->user) {
		pr_err("PCIe: User of event registration is NULL\n");

		return -ENODEV;
	}
	pp = PCIE_BUS_PRIV_DATA(((struct pci_dev *)reg->user));
	pci = to_dw_pcie_from_pp(pp);
	exynos_pcie = to_exynos_pcie(pci);

	if (pp) {
		exynos_pcie->event_reg = reg;
		dev_info(pci->dev, "Event 0x%x is registered for RC %d\n",
			 reg->events, exynos_pcie->ch_num);
	} else {
		pr_err("PCIe: did not find RC for pci endpoint device\n");
		ret = -ENODEV;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pcie_register_event);

int exynos_pcie_deregister_event(struct exynos_pcie_register_event *reg)
{
	int ret = 0;
	struct dw_pcie_rp *pp;
	struct exynos_pcie *exynos_pcie;
	struct dw_pcie *pci;

	if (!reg) {
		pr_err("PCIe: Event deregistration is NULL\n");

		return -ENODEV;
	}
	if (!reg->user) {
		pr_err("PCIe: User of event deregistration is NULL\n");

		return -ENODEV;
	}

	pp = PCIE_BUS_PRIV_DATA(((struct pci_dev *)reg->user));
	pci = to_dw_pcie_from_pp(pp);
	exynos_pcie = to_exynos_pcie(pci);

	if (pp) {
		exynos_pcie->event_reg = NULL;
		dev_info(pci->dev, "Event is deregistered for RC %d\n", exynos_pcie->ch_num);
	} else {
		pr_err("PCIe: did not find RC for pci endpoint device\n");
		ret = -ENODEV;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pcie_deregister_event);

int exynos_pcie_rc_set_affinity(int ch_num, int affinity)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci;
	struct dw_pcie_rp *pp;

	if (!exynos_pcie) {
		pr_err("%s: ch#%d PCIe device is not loaded\n", __func__, ch_num);

		return -ENODEV;
	}

	pci = exynos_pcie->pci;
	pp = &pci->pp;

	irq_set_affinity_hint(pp->irq, cpumask_of(affinity));

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_set_affinity);

int exynos_pcie_rc_set_enable_wake(struct irq_data *data, unsigned int enable)
{
	int ret = 0;
	struct dw_pcie_rp *pp = data->parent_data->domain->host_data;

	pr_debug("%s: enable = %d\n", __func__, enable);

	if (pp == NULL) {
		pr_err("Warning: exynos_pcie_rc_set_enable_wake: not exist pp\n");
		return -EINVAL;
	}

	if (enable)
		ret = enable_irq_wake(pp->irq);
	else
		ret = disable_irq_wake(pp->irq);

	return ret;
}

#if IS_ENABLED(CONFIG_CPU_IDLE)
static void __maybe_unused exynos_pcie_rc_set_tpoweron(struct dw_pcie_rp *pp, int max)
{
	void __iomem *ep_dbi_base = pp->va_cfg0_base;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	u32 val;

	if (exynos_pcie->state != STATE_LINK_UP)
		return;

	/* Disable ASPM */
	val = readl(ep_dbi_base + WIFI_L1SS_LINKCTRL);
	val &= ~(WIFI_ASPM_CONTROL_MASK);
	writel(val, ep_dbi_base + WIFI_L1SS_LINKCTRL);

	val = readl(ep_dbi_base + WIFI_L1SS_CONTROL);
	writel(val & ~(WIFI_ALL_PM_ENABEL), ep_dbi_base + WIFI_L1SS_CONTROL);

	if (max) {
		writel(PORT_LINK_TPOWERON_3100US, ep_dbi_base + WIFI_L1SS_CONTROL2);
		exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL2, 4,
					   PORT_LINK_TPOWERON_3100US);
	} else {
		writel(PORT_LINK_TPOWERON_130US, ep_dbi_base + WIFI_L1SS_CONTROL2);
		exynos_pcie_rc_wr_own_conf(pp, PCIE_LINK_L1SS_CONTROL2, 4,
					   PORT_LINK_TPOWERON_130US);
	}

	/* Enable L1ss */
	val = readl(ep_dbi_base + WIFI_L1SS_LINKCTRL);
	val |= WIFI_ASPM_L1_ENTRY_EN;
	writel(val, ep_dbi_base + WIFI_L1SS_LINKCTRL);

	val = readl(ep_dbi_base + WIFI_L1SS_CONTROL);
	val |= WIFI_ALL_PM_ENABEL;
	writel(val, ep_dbi_base + WIFI_L1SS_CONTROL);
}
#endif

static int exynos_pcie_msi_set_affinity(struct irq_data *irq_data, const struct cpumask *mask,
					bool force)
{
	struct dw_pcie_rp *pp;
	struct irq_data *idata = irq_data->parent_data;
	struct dw_pcie *pci;
	struct exynos_pcie *exynos_pcie;

	if (!idata)
		return -ENODEV;

	/* set affinity for PCIe IRQ */
	pp = (struct dw_pcie_rp *) idata->chip_data;
	if (!pp)
		return -ENODEV;

	pci = to_dw_pcie_from_pp(pp);
	if (!pci)
		return -ENODEV;

	exynos_pcie = to_exynos_pcie(pci);
	if (!exynos_pcie)
		return -ENODEV;

	/* modem driver sets msi irq affinity */
	if (exynos_pcie->ch_num == 0)
		return 0;

	idata = irq_get_irq_data(pp->irq);
	if (!idata || !idata->chip)
		return -ENODEV;

	if (idata->chip->irq_set_affinity)
		idata->chip->irq_set_affinity(idata, mask, force);

	return 0;
}

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
int exynos_pcie_rc_itmon_notifier(struct notifier_block *nb, unsigned long action, void *nb_data)
{
	struct exynos_pcie *exynos_pcie = container_of(nb, struct exynos_pcie, itmon_nb);
	struct device *dev = exynos_pcie->pci->dev;
	struct itmon_notifier *itmon_info = nb_data;
	unsigned int val;

	dev_info(dev, "### EXYNOS PCIE ITMON ###\n");

	if (IS_ERR_OR_NULL(itmon_info))
		return NOTIFY_DONE;

	if (exynos_pcie->phy_control == PCIE_PHY_ISOLATION)
		exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_BYPASS);

	/* only for 9830 HSI2 block */
	if (exynos_pcie->ip_ver == 0x983000) {
		if ((itmon_info->port && !strcmp(itmon_info->port, "HSI2")) ||
		    (itmon_info->dest && !strcmp(itmon_info->dest, "HSI2"))) {
			regmap_read(exynos_pcie->pmureg, exynos_pcie->pmu_offset, &val);
			dev_info(dev, "### PMU PHY Isolation : 0x%x\n", val);

			exynos_pcie_rc_register_dump(exynos_pcie->ch_num);
		}
	} else if (exynos_pcie->ip_ver == 0x984500){
		if ((itmon_info->port && !strcmp(itmon_info->port, "HSI2")) ||
		    (itmon_info->dest && !strcmp(itmon_info->dest, "HSI2"))) {
			regmap_read(exynos_pcie->pmureg, exynos_pcie->pmu_offset, &val);
			dev_info(dev, "### PMU PHY Isolation : 0x%x\n", val);

			exynos_pcie_rc_register_dump(exynos_pcie->ch_num);
		}
	} else {
		dev_info(dev, "skip register dump(ip_ver = 0x%x)\n", exynos_pcie->ip_ver);
	}

	return NOTIFY_DONE;
}
#endif

static const char *sep_irq_name[PCIE_MAX_SEPA_IRQ_NUM] = {
	"exynos-pcie-msi0", "exynos-pcie-msi1", "exynos-pcie-msi2",
	"exynos-pcie-msi3", "exynos-pcie-msi4" };

static irqreturn_t exynos_pcie_msi0_handler(int irq, void *arg)
{
	struct dw_pcie_rp *pp = arg;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct device *dev = pci->dev;
	int ch_num = exynos_pcie->ch_num;
	struct separated_msi_vector *msi_vec = &sep_msi_vec[ch_num][0];

	if (!msi_vec->is_used) {
		dev_err(dev, "Unexpected separated MSI0 interrupt!");
		return IRQ_HANDLED;
	}

	dw_handle_msi_irq(pp);

	if (!msi_vec->flags) {
		/* To set as handle_level_irq, get virq, mapped_irq, irq_data. */
		struct irq_data *irq_data;
		int i, virq, mapped_irq, changed_irq = 0;

		virq = irq_find_mapping(pp->irq_domain, 0);
		mapped_irq = pp->irq_domain->mapcount;

		dev_dbg(dev, "Start virq = %d, Total mapped irq = %d\n",
				virq, mapped_irq);

		for (i = 0; i < PCIE_DOMAIN_MAX_IRQ; i++) {
			irq_data = irq_domain_get_irq_data(pp->irq_domain, virq);
			if (irq_data == NULL) {
				virq++;
				continue;
			}

			dev_dbg(dev, "Change flow interrupt for virq(%d)\n", virq);
			irq_domain_set_info(pp->irq_domain, virq, irq_data->hwirq,
					pp->msi_irq_chip,
					pp, handle_level_irq,
					NULL, NULL);

			virq++;
			changed_irq++;

			if (changed_irq == mapped_irq)
				break;
		}
		msi_vec->flags = 1;
	}
	exynos_pcie_msi_post_process(pp);

	return IRQ_HANDLED;
}

static irqreturn_t exynos_pcie_msi1_handler(int irq, void *arg)
{
	struct dw_pcie_rp *pp = arg;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct device *dev = pci->dev;
	int ch_num = exynos_pcie->ch_num;
	struct separated_msi_vector *msi_vec = &sep_msi_vec[ch_num][1];
	int vec_num = 1;

	/* Clear MSI interrupt */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_STATUS +
			(vec_num * MSI_REG_CTRL_BLOCK_SIZE), 4, 0x1);

	if (!msi_vec->is_used) {
		dev_err(dev, "Unexpected separated MSI1 interrupt!");
		return IRQ_HANDLED;
	}

	if (msi_vec->msi_irq_handler != NULL)
		msi_vec->msi_irq_handler(irq, msi_vec->context);

	exynos_pcie_msi_post_process(pp);

	return IRQ_HANDLED;
}

static irqreturn_t exynos_pcie_msi2_handler(int irq, void *arg)
{
	struct dw_pcie_rp *pp = arg;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct device *dev = pci->dev;
	int ch_num = exynos_pcie->ch_num;
	struct separated_msi_vector *msi_vec = &sep_msi_vec[ch_num][2];
	int vec_num = 2;

	/* Clear MSI interrupt */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_STATUS +
			(vec_num * MSI_REG_CTRL_BLOCK_SIZE), 4, 0x1);

	if (!msi_vec->is_used) {
		dev_err(dev, "Unexpected separated MSI2 interrupt!");
		return IRQ_HANDLED;
	}

	if (msi_vec->msi_irq_handler != NULL)
		msi_vec->msi_irq_handler(irq, msi_vec->context);

	exynos_pcie_msi_post_process(pp);

	return IRQ_HANDLED;
}

static irqreturn_t exynos_pcie_msi3_handler(int irq, void *arg)
{
	struct dw_pcie_rp *pp = arg;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct device *dev = pci->dev;
	int ch_num = exynos_pcie->ch_num;
	struct separated_msi_vector *msi_vec = &sep_msi_vec[ch_num][3];
	int vec_num = 3;

	/* Clear MSI interrupt */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_STATUS +
			(vec_num * MSI_REG_CTRL_BLOCK_SIZE), 4, 0x1);

	if (!msi_vec->is_used) {
		dev_err(dev, "Unexpected separated MSI3 interrupt!");
		return IRQ_HANDLED;
	}

	if (msi_vec->msi_irq_handler != NULL)
		msi_vec->msi_irq_handler(irq, msi_vec->context);

	exynos_pcie_msi_post_process(pp);

	return IRQ_HANDLED;
}

static irqreturn_t exynos_pcie_msi4_handler(int irq, void *arg)
{
	struct dw_pcie_rp *pp = arg;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct device *dev = pci->dev;
	int ch_num = exynos_pcie->ch_num;
	struct separated_msi_vector *msi_vec = &sep_msi_vec[ch_num][4];
	int vec_num = 4;

	/* Clear MSI interrupt */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_STATUS +
			(vec_num * MSI_REG_CTRL_BLOCK_SIZE), 4, 0x1);

	if (!msi_vec->is_used) {
		dev_err(dev, "Unexpected separated MSI4 interrupt!");
		return IRQ_HANDLED;
	}

	if (msi_vec->msi_irq_handler != NULL)
		msi_vec->msi_irq_handler(irq, msi_vec->context);

	exynos_pcie_msi_post_process(pp);

	return IRQ_HANDLED;
}

irqreturn_t (*msi_handler[PCIE_MAX_SEPA_IRQ_NUM])(int , void *) = {
	exynos_pcie_msi0_handler, exynos_pcie_msi1_handler,  exynos_pcie_msi2_handler,
	exynos_pcie_msi3_handler, exynos_pcie_msi4_handler };

int register_separated_msi_vector(int ch_num, irq_handler_t handler, void *context,
		int *irq_num)
{
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	int i, ret;

	for (i = 1; i < PCIE_MAX_SEPA_IRQ_NUM; i++) {
		if (!sep_msi_vec[ch_num][i].is_used)
			break;
	}

	if (i == PCIE_MAX_SEPA_IRQ_NUM) {
		dev_info(pci->dev, "PCIe Ch%d : There is no empty MSI vector!\n", ch_num);
		return -EBUSY;
	}

	pr_info("PCIe Ch%d MSI%d vector is registered!\n", ch_num, i);
	sep_msi_vec[ch_num][i].is_used = true;
	sep_msi_vec[ch_num][i].context = context;
	sep_msi_vec[ch_num][i].msi_irq_handler = handler;
	*irq_num = sep_msi_vec[ch_num][i].irq;

	/* Enable MSI interrupt for separated MSI. */
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE +
			(i * MSI_REG_CTRL_BLOCK_SIZE), 4, 0x1);
	exynos_pcie_rc_wr_own_conf(pp, PCIE_MSI_INTR0_MASK +
			(i * MSI_REG_CTRL_BLOCK_SIZE), 4, ~(0x1));

	ret = devm_request_irq(pci->dev, sep_msi_vec[ch_num][i].irq,
			msi_handler[i],
			IRQF_SHARED | IRQF_TRIGGER_HIGH,
			sep_irq_name[i], pp);
	if (ret) {
		pr_err("failed to request MSI%d irq\n", i);

		return ret;
	}

	return i * PCIE_MSI_MAX_VEC_NUM;
}
EXPORT_SYMBOL_GPL(register_separated_msi_vector);

static int exynos_pcie_rc_add_port(struct platform_device *pdev, struct dw_pcie_rp *pp, int ch_num)
{
	struct irq_domain *msi_domain;
	struct msi_domain_info *msi_domain_info;
	int ret, i, sep_irq;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	pp->irq = platform_get_irq(pdev, 0);
	if (!pp->irq) {
		dev_err(&pdev->dev, "failed to get irq\n");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, pp->irq, exynos_pcie_rc_irq_handler,
			       IRQF_SHARED | IRQF_TRIGGER_HIGH, "exynos-pcie", pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");

		return ret;
	}

	if (exynos_pcie->separated_msi) {
		for (i = 0; i < PCIE_MAX_SEPA_IRQ_NUM; i++) {
			sep_irq = platform_get_irq(pdev, i + 1);
			if (sep_irq < 0)
				goto skip_sep_request_irq;

			dev_info(&pdev->dev, "%d separated MSI irq is defined.\n", sep_irq);
			sep_msi_vec[ch_num][i].irq = sep_irq;

			/* MSI vector 0 will used by default MSI */
			if (i == 0) {
				sep_msi_vec[ch_num][0].is_used = true;
				ret = devm_request_irq(pci->dev, sep_msi_vec[ch_num][0].irq,
						msi_handler[0],
						IRQF_SHARED | IRQF_TRIGGER_HIGH,
						sep_irq_name[0], pp);
				if (ret) {
					dev_err(&pdev->dev, "failed to request MSI%d irq\n", i);
					return ret;
				}
				continue;
			}
		}
	}

skip_sep_request_irq:
	exynos_pcie_setup_rc(pp);

	if (exynos_pcie->ep_device_type == EP_QC_WIFI) {
		/*
		 * Set DMA mask to 32-bit because
		 * devices only work with 32-bit MSI.
		 */
		ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(&pdev->dev, "Failed to set DMA mask to 32-bit.");
		}
	}

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to dw pcie host init\n");

		return ret;
	}
	// Reset this back to 36-bits
	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));

	if (pp->msi_domain) {
		msi_domain = pp->msi_domain;
		msi_domain_info = (struct msi_domain_info *)msi_domain->host_data;
		msi_domain_info->chip->irq_set_affinity = exynos_pcie_msi_set_affinity;
		msi_domain_info->chip->irq_set_wake = exynos_pcie_rc_set_enable_wake;
		if (exynos_pcie->ep_device_type == EP_QC_WIFI ||
				exynos_pcie->ep_device_type == EP_SAMSUNG_MODEM) {
			msi_domain_info->chip->irq_mask = pci_msi_mask_irq;
			msi_domain_info->chip->irq_unmask = pci_msi_unmask_irq;
		}
	}

	return 0;
}

static int exynos_pcie_rc_panic_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct exynos_pcie *exynos_pcie = container_of(nb, struct exynos_pcie, panic_nb);

	pr_err("%s ch%d: +++\n", __func__, exynos_pcie->ch_num);

	if (exynos_pcie->phy_control == PCIE_PHY_ISOLATION)
		exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_BYPASS);

	pr_err("trsv_reg38B [0x0E2C]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E2C));
	pr_err("cmn_reg17F [0x05FC]: %#x\n", exynos_phy_read(exynos_pcie, 0x05FC));
	pr_err("cmn_reg0FC [0x03F0]: %#x\n", exynos_phy_read(exynos_pcie, 0x03F0));
	pr_err("trsv_reg386 [0x0E18]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E18));
	pr_err("trsv_reg39D [0x0E74]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E74));
	pr_err("trsv_reg383 [0x0E0C]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E0C));
	pr_err("cmn_reg1D8 [0x0760]: %#x\n", exynos_phy_read(exynos_pcie, 0x0760));
	pr_err("trsv_reg38A [0x0E28]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E28));
	pr_err("trsv_reg36D [0x0DB4]: %#x\n", exynos_phy_read(exynos_pcie, 0x0DB4));
	pr_err("trsv_reg36E [0x0DB8]: %#x\n", exynos_phy_read(exynos_pcie, 0x0DB8));
	pr_err("trsv_reg39E [0x0E78]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E78));
	pr_err("trsv_reg39F [0x0E7C]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E7C));

	pr_err("trsv_reg36F [0x0DBC]: %#x\n", exynos_phy_read(exynos_pcie, 0x0DBC));
	pr_err("trsv_reg379 [0x0DE4]: %#x\n", exynos_phy_read(exynos_pcie, 0x0DE4));
	pr_err("trsv_reg392 [0x0E48]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E48));
	pr_err("trsv_reg393 [0x0E4C]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E4C));
	pr_err("trsv_reg394 [0x0E50]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E50));
	pr_err("trsv_reg395 [0x0E54]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E54));
	pr_err("trsv_reg396 [0x0E58]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E58));
	pr_err("trsv_reg397 [0x0E5C]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E5C));
	pr_err("trsv_reg398 [0x0E60]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E60));
	pr_err("trsv_reg399 [0x0E64]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E64));
	pr_err("trsv_reg39A [0x0E68]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E68));
	pr_err("trsv_reg39B [0x0E6C]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E6C));
	pr_err("trsv_reg39C [0x0E70]: %#x\n", exynos_phy_read(exynos_pcie, 0x0E70));
	pr_err("trsv_reg3B3 [0x0ECC]: %#x\n", exynos_phy_read(exynos_pcie, 0x0ECC));
	pr_err("trsv_reg3B4 [0x0ED0]: %#x\n", exynos_phy_read(exynos_pcie, 0x0ED0));
	pr_err("trsv_reg3B5 [0x0ED4]: %#x\n", exynos_phy_read(exynos_pcie, 0x0ED4));
	pr_err("trsv_reg3B6 [0x0ED8]: %#x\n", exynos_phy_read(exynos_pcie, 0x0ED8));
	pr_err("trsv_reg3B7 [0x0EDC]: %#x\n", exynos_phy_read(exynos_pcie, 0x0EDC));
	pr_err("trsv_reg3B8 [0x0EE0]: %#x\n", exynos_phy_read(exynos_pcie, 0x0EE0));
	pr_err("trsv_reg3B9 [0x0EE4]: %#x\n", exynos_phy_read(exynos_pcie, 0x0EE4));
	pr_err("trsv_reg3BA [0x0EE8]: %#x\n", exynos_phy_read(exynos_pcie, 0x0EE8));

	pr_err("%s: ---\n", __func__);

	return NOTIFY_DONE;
}

static void exynos_pcie_rc_pcie_ops_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	struct exynos_pcie_ops *pcie_ops = &exynos_pcie->exynos_pcie_ops;
	struct device *dev = pci->dev;

	dev_info(dev, "Initialize PCIe function.\n");

	pcie_ops->poweron = exynos_pcie_rc_poweron;
	pcie_ops->poweroff = exynos_pcie_rc_poweroff;
	pcie_ops->rd_own_conf = exynos_pcie_rc_rd_own_conf;
	pcie_ops->wr_own_conf = exynos_pcie_rc_wr_own_conf;
	pcie_ops->rd_other_conf = exynos_pcie_rc_rd_other_conf;
	pcie_ops->wr_other_conf = exynos_pcie_rc_wr_other_conf;
}

static int exynos_pcie_rc_make_reg_tb(struct device *dev, struct exynos_pcie *exynos_pcie)
{
	unsigned int pos, val, id;
	int i;

	/* initialize the reg table */
	for (i = 0; i < 48; i++) {
		exynos_pcie->pci_cap[i] = 0;
		exynos_pcie->pci_ext_cap[i] = 0;
	}

	pos = 0xFF & readl(exynos_pcie->rc_dbi_base + PCI_CAPABILITY_LIST);

	while (pos) {
		val = readl(exynos_pcie->rc_dbi_base + pos);
		id = val & CAP_ID_MASK;
		exynos_pcie->pci_cap[id] = pos;
		pos = (readl(exynos_pcie->rc_dbi_base + pos) & CAP_NEXT_OFFSET_MASK) >> 8;
		dev_dbg(dev, "Next Cap pointer : 0x%x\n", pos);
	}

	pos = PCI_CFG_SPACE_SIZE;

	while (pos) {
		val = readl(exynos_pcie->rc_dbi_base + pos);
		if (val == 0) {
			dev_info(dev, "we have no ext capabilities!\n");

			break;
		}
		id = PCI_EXT_CAP_ID(val);
		exynos_pcie->pci_ext_cap[id] = pos;
		pos = PCI_EXT_CAP_NEXT(val);
		dev_dbg(dev, "Next ext Cap pointer : 0x%x\n", pos);
	}

	for (i = 0; i < 48; i++) {
		if (exynos_pcie->pci_cap[i])
			dev_info(dev, "PCIe cap [0x%x][%s]: 0x%x\n",
				 i, CAP_ID_NAME(i), exynos_pcie->pci_cap[i]);
	}
	for (i = 0; i < 48; i++) {
		if (exynos_pcie->pci_ext_cap[i])
			dev_info(dev, "PCIe ext cap [0x%x][%s]: 0x%x\n",
				 i, EXT_CAP_ID_NAME(i), exynos_pcie->pci_ext_cap[i]);
	}

	return 0;
}

u32 pcie_linkup_stat(void)
{
	pr_info("[%s] pcie_is_linkup : %d\n", __func__, pcie_is_linkup);

	return pcie_is_linkup;
}
EXPORT_SYMBOL_GPL(pcie_linkup_stat);

int exynos_pcie_set_msi_ctrl_addr(int ch_num, u64 msi_ctrl_addr) {
	struct exynos_pcie *exynos_pcie = &g_pcie_rc[ch_num];

        exynos_pcie->pci->pp.msi_data = msi_ctrl_addr;
        dev_info(exynos_pcie->pci->dev, "Updated MSI Control Addr: %pad\n",
                &exynos_pcie->pci->pp.msi_data);

        return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_set_msi_ctrl_addr);

#if IS_ENABLED(CONFIG_GS_S2MPU)
static int setup_s2mpu_mem(struct device *dev, struct exynos_pcie *exynos_pcie)
{
	struct device_node *np;
	struct resource res;
	struct phys_mem *pm;
	phys_addr_t addr;
	int ret;

	/* Parse the memory nodes in the device tree to determine which areas
	 * the s2mpu should protect.
	 */

	INIT_LIST_HEAD(&exynos_pcie->phys_mem_list);

	for_each_node_by_type(np, "memory") {
		ret = of_address_to_resource(np, 0, &res);
		if (ret)
			continue;

		if (list_empty(&exynos_pcie->phys_mem_list)) {
			pm = devm_kzalloc(dev, sizeof(*pm), GFP_KERNEL);
			if (!pm)
				return -ENOMEM;

			pm->start = res.start;
			pm->size = resource_size(&res);
			list_add(&pm->list, &exynos_pcie->phys_mem_list);
		} else {
			/* To simplify the code, assume that memory regions
			 * in the device tree are sorted in the descending
			 * order. If this is not the case, abort.
			 */
			pm = list_last_entry(&exynos_pcie->phys_mem_list,
					     struct phys_mem, list);
			if (res.end > pm->start) {
				dev_err(dev, "s2mpu memory sort invalid end=%pa start=%pa\n",
					&res.end, &pm->start);
				dev_err(dev, "This driver expects all DRAM ranges i.e. device tree nodes with device_type=\"memory\" to be defined in descending order in the device tree. Please change your device tree accordingly.\n");
				return -EINVAL;
			}

			/* If two memory regions are consecutive, merge them. */
			if (pm->start - res.end == 1) {
				pm->start = res.start;
				pm->size += resource_size(&res);
			} else {
				pm = devm_kzalloc(dev, sizeof(*pm), GFP_KERNEL);
				if (!pm)
					return -ENOMEM;

				pm->start = res.start;
				pm->size = resource_size(&res);
				list_add_tail(&pm->list,
					      &exynos_pcie->phys_mem_list);
			}
		}
	}

	list_for_each_entry(pm, &exynos_pcie->phys_mem_list, list) {
		pm->refcnt_array = devm_kzalloc(dev, pm->size / SZ_4K,
						GFP_KERNEL);
		if (!pm->refcnt_array)
			return -ENOMEM;

		/* Optimize s2mpu operation by setting up 1G page tables */
		addr = pm->start;
		while (addr <  pm->start + pm->size) {
			ret = s2mpu_close(exynos_pcie->s2mpu, addr, ALIGN_SIZE,
					  DMA_BIDIRECTIONAL);
			if (ret) {
				dev_err(dev,
					"probe s2mpu_close failed addr = 0x%pa\n",
					&addr);
			}
			addr += SZ_1G;
		}
	}

	return ret;
}
#endif

static int exynos_pcie_rc_probe(struct platform_device *pdev)
{
	struct exynos_pcie *exynos_pcie;
	struct dw_pcie *pci;
	struct dw_pcie_rp *pp;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int ch_num;
#if IS_ENABLED(CONFIG_GS_S2MPU)
	struct device_node *s2mpu_dn;
#endif

	dev_info(&pdev->dev, "## PCIe RC PROBE start\n");

	if (create_pcie_sys_file(&pdev->dev))
		dev_err(&pdev->dev, "Failed to create pcie sys file\n");

	if (of_property_read_u32(np, "ch-num", &ch_num)) {
		dev_err(&pdev->dev, "Failed to parse the channel number\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "## PCIe ch %d ##\n", ch_num);

	pci = devm_kzalloc(&pdev->dev, sizeof(*pci), GFP_KERNEL);
	if (!pci) {
		/* dev_err(&pdev->dev, "dw_pcie allocation is failed\n"); */

		return -ENOMEM;
	}

	exynos_pcie = &g_pcie_rc[ch_num];
	exynos_pcie->pci = pci;

	pci->dev = &pdev->dev;
	pci->ops = &dw_pcie_ops;

	pp = &pci->pp;
	pp->ops = &exynos_pcie_rc_ops;

	/* Reserve memory for pma_regs that may need to be restored on
	 * pcie link up failure.
	 */
	exynos_pcie->pma_regs = devm_kcalloc(&pdev->dev, NUM_PMA_REGS,
					     sizeof(u32), GFP_KERNEL);
	if (!exynos_pcie->pma_regs)
		return -ENOMEM;

	spin_lock_init(&exynos_pcie->pcie_l1_exit_lock);
	spin_lock_init(&exynos_pcie->conf_lock);
	spin_lock_init(&exynos_pcie->power_stats_lock);
	spin_lock_init(&exynos_pcie->reg_lock);
	spin_lock_init(&exynos_pcie->s2mpu_refcnt_lock);

	mutex_init(&exynos_pcie->power_onoff_lock);

	exynos_pcie->ch_num = ch_num;
	exynos_pcie->l1ss_enable = 1;
	exynos_pcie->state = STATE_LINK_DOWN;

	exynos_pcie->linkdown_cnt = 0;
	exynos_pcie->sudden_linkdown = 0;
	exynos_pcie->cpl_timeout_recovery = 0;
	exynos_pcie->l1ss_ctrl_id_state = 0;
	exynos_pcie->atu_ok = 0;

	exynos_pcie->app_req_exit_l1 = PCIE_APP_REQ_EXIT_L1;
	exynos_pcie->app_req_exit_l1_mode = PCIE_APP_REQ_EXIT_L1_MODE;
	exynos_pcie->linkup_offset = PCIE_ELBI_RDLH_LINKUP;

	exynos_pcie->log = NULL;
	if (ch_num == 0)
		exynos_pcie->log = logbuffer_register("pcie0");
	else if (ch_num == 1)
		exynos_pcie->log = logbuffer_register("pcie1");
	else
		dev_err(&pdev->dev, "invalid ch_num=%d for logbuffer registry\n", ch_num);
	if (IS_ERR_OR_NULL(exynos_pcie->log)) {
		dev_err(&pdev->dev, "logbuffer register failed\n");
		exynos_pcie->log = NULL;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	device_enable_async_suspend(&pdev->dev);

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));
	platform_set_drvdata(pdev, exynos_pcie);
	power_stats_init(exynos_pcie);
	link_stats_init(exynos_pcie);

#if IS_ENABLED(CONFIG_GS_S2MPU)
	s2mpu_dn = of_parse_phandle(np, "s2mpu", 0);
	if (s2mpu_dn) {
		memcpy(&fake_dma_dev, &pdev->dev, sizeof(fake_dma_dev));
		fake_dma_dev.dma_ops = NULL;

		exynos_pcie->s2mpu = s2mpu_fwnode_to_info(&s2mpu_dn->fwnode);
		if (!exynos_pcie->s2mpu) {
			dev_err(&pdev->dev, "Failed to get S2MPU\n");
			return -EPROBE_DEFER;
		}

		ret = setup_s2mpu_mem(&pdev->dev, exynos_pcie);
		if (ret) {
			dev_err(&pdev->dev, "failed to bind to s2mpu\n");
			goto probe_fail;
		}

		dev_info(&pdev->dev, "successfully bound to S2MPU\n");
	}
#endif

	/* parsing pcie dts data for exynos */
	ret = exynos_pcie_rc_parse_dt(&pdev->dev, exynos_pcie);
	if (ret)
		goto probe_fail;

	ret = exynos_pcie_rc_get_pin_state(pdev, exynos_pcie);
	if (ret)
		goto probe_fail;

	ret = exynos_pcie_rc_clock_get(pp);
	if (ret)
		goto probe_fail;

	ret = exynos_pcie_rc_get_resource(pdev, exynos_pcie);
	if (ret)
		goto probe_fail;
	pci->dbi_base = exynos_pcie->rc_dbi_base;

	/* NOTE: TDB */
	/* Mapping PHY functions */
	exynos_pcie_rc_phy_init(pp);

	exynos_pcie_rc_pcie_ops_init(pp);

	exynos_pcie_rc_resumed_phydown(pp);

	if (exynos_pcie->use_nclkoff_en)
		exynos_pcie_rc_nclkoff_ctrl(pdev, exynos_pcie);
#if IS_ENABLED(CONFIG_EXYNOS_PCIE_IOMMU)
	/* if it needed for msi init, property should be added on dt */
#if 0	/* to be updated when ready */
	set_dma_ops(&pdev->dev, &exynos_pcie_dma_ops);
	dev_info(&pdev->dev, "DMA operations are changed\n");
#endif	/* to be updated when ready */
#endif

	ret = exynos_pcie_rc_make_reg_tb(&pdev->dev, exynos_pcie);
	if (ret)
		goto probe_fail;

	ret = exynos_pcie_rc_add_port(pdev, pp, ch_num);
	if (ret)
		goto probe_fail;

	if (exynos_pcie->use_cache_coherency)
		exynos_pcie_rc_set_iocc(pp, 1);

	disable_irq(pp->irq);

#if IS_ENABLED(CONFIG_CPU_IDLE)
	exynos_pcie->idle_ip_index =
			exynos_get_idle_ip_index(dev_name(&pdev->dev));
	if (exynos_pcie->idle_ip_index < 0)
		dev_err(&pdev->dev, "Cant get idle_ip_dex!!!\n");
	else
		dev_err(&pdev->dev, "PCIE idle ip index : %d\n", exynos_pcie->idle_ip_index);

	exynos_update_ip_idle_status(exynos_pcie->idle_ip_index, PCIE_IS_IDLE);
	dev_info(&pdev->dev, "%s, ip idle status : %d, idle_ip_index: %d\n",
		 __func__, PCIE_IS_IDLE, exynos_pcie->idle_ip_index);
#endif

	exynos_pcie->pcie_wq = create_freezable_workqueue("pcie_wq");
	if (IS_ERR(exynos_pcie->pcie_wq)) {
		dev_err(&pdev->dev, "couldn't create workqueue\n");
		ret = EBUSY;

		goto probe_fail;
	}

	INIT_DELAYED_WORK(&exynos_pcie->dislink_work, exynos_pcie_rc_dislink_work);
	INIT_DELAYED_WORK(&exynos_pcie->cpl_timeout_work, exynos_pcie_rc_cpl_timeout_work);

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	exynos_pcie->itmon_nb.notifier_call = exynos_pcie_rc_itmon_notifier;
	itmon_notifier_chain_register(&exynos_pcie->itmon_nb);
#endif
	exynos_pcie->panic_nb.notifier_call = exynos_pcie_rc_panic_notifier;
	atomic_notifier_chain_register(&panic_notifier_list, &exynos_pcie->panic_nb);

	if (exynos_pcie->use_pcieon_sleep) {
		dev_info(&pdev->dev, "## register pcie connection function\n");
		register_pcie_is_connect(pcie_linkup_stat);
	}

	platform_set_drvdata(pdev, exynos_pcie);

probe_fail:
	if (exynos_pcie->use_phy_isol_con)
		exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_ISOLATION);

	if (ret) {
		dev_err(&pdev->dev, "## %s: PCIe probe failed\n", __func__);
		logbuffer_log(exynos_pcie->log, "PCIe %d: probe failed", ch_num);
	} else {
		dev_info(&pdev->dev, "## %s: PCIe probe success\n", __func__);
		logbuffer_log(exynos_pcie->log, "PCIe %d: probe done", ch_num);
	}

	return ret;
}

static int __exit exynos_pcie_rc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = container_of(&dev, struct dw_pcie, dev);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	dev_info(&pdev->dev, "%s\n", __func__);

	mutex_destroy(&exynos_pcie->power_onoff_lock);

	return 0;
}

#if IS_ENABLED(CONFIG_PM)
static int exynos_pcie_rc_suspend_noirq(struct device *dev)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);
	u32 val, val_irq0, val_irq1, val_irq2;

	logbuffer_log(exynos_pcie->log, "pm_suspend_no_irq called");
	if (exynos_pcie->state == STATE_LINK_DOWN) {
		dev_dbg(dev, "PCIe PMU ISOLATION\n");
		exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_ISOLATION);

		return 0;
	} else if (exynos_pcie->separated_msi && exynos_pcie->use_pcieon_sleep) {
		dev_info(dev, "PCIe on sleep... suspend\n");

		/* handle IRQ0 interrupt */
		val_irq0 = exynos_elbi_read(exynos_pcie, PCIE_IRQ0);
		exynos_elbi_write(exynos_pcie, val_irq0, PCIE_IRQ0);
		dev_info(dev, "IRQ0 0x%x\n", val_irq0);

		/* handle IRQ1 interrupt */
		val_irq1 = exynos_elbi_read(exynos_pcie, PCIE_IRQ1);
		exynos_elbi_write(exynos_pcie, val_irq1, PCIE_IRQ1);
		dev_info(dev, "IRQ1 0x%x\n", val_irq1);

		/* handle IRQ2 interrupt */
		val_irq2 = exynos_elbi_read(exynos_pcie, PCIE_IRQ2);
		exynos_elbi_write(exynos_pcie, val_irq2, PCIE_IRQ2);
		dev_info(dev, "IRQ2 0x%x\n", val_irq2);

		val = exynos_elbi_read(exynos_pcie, PCIE_IRQ2_EN);
		val |= IRQ_MSI_CTRL_EN_RISING_EDG;
		exynos_elbi_write(exynos_pcie, val, PCIE_IRQ2_EN);
	}

	if (exynos_pcie->use_pcieon_sleep && exynos_pcie->ep_pci_dev) {
		dev_info(dev, "Default must_resume value : %d\n",
				exynos_pcie->ep_pci_dev->dev.power.must_resume);
		exynos_pcie->pcie_must_resume = exynos_pcie->ep_pci_dev->dev.power.must_resume;
		if (exynos_pcie->ep_pci_dev->dev.power.must_resume)
			exynos_pcie->ep_pci_dev->dev.power.must_resume = false;

		dev_info(dev, "restore enable cnt = %d\n", exynos_pcie->pcieon_sleep_enable_cnt);
		atomic_set(&exynos_pcie->ep_pci_dev->enable_cnt,
				exynos_pcie->pcieon_sleep_enable_cnt);
	}

	return 0;
}

static int exynos_pcie_rc_resume_noirq(struct device *dev)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = exynos_pcie->pci;
	u32 val;

	dev_dbg(dev, "## RESUME[%s] pcie_is_linkup: %d)\n", __func__, pcie_is_linkup);
	logbuffer_log(exynos_pcie->log, "pm_resume_no_irq called");

	if (exynos_pcie->state == STATE_LINK_DOWN) {
		dev_dbg(dev, "[%s] dislink state after resume -> phy pwr off\n", __func__);
		exynos_pcie_rc_resumed_phydown(&pci->pp);
	} else if (exynos_pcie->separated_msi && exynos_pcie->use_pcieon_sleep) {
		dev_info(dev, "PCIe on sleep resume...\n");
		val = exynos_elbi_read(exynos_pcie, PCIE_IRQ2_EN);
		val &= ~IRQ_MSI_CTRL_EN_RISING_EDG;
		exynos_elbi_write(exynos_pcie, val, PCIE_IRQ2_EN);
	}

	return 0;
}

static int exynos_pcie_suspend_prepare(struct device *dev)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	logbuffer_log(exynos_pcie->log, "suspend_prepare called");
	if (exynos_pcie->use_phy_isol_con)
		exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_BYPASS);

	if (exynos_pcie->use_pcieon_sleep && exynos_pcie->ep_pci_dev) {
		exynos_pcie->pcieon_sleep_enable_cnt =
			atomic_read(&exynos_pcie->ep_pci_dev->enable_cnt);
		dev_info(dev, "remove enable cnt to fake enable = %d\n",
				exynos_pcie->pcieon_sleep_enable_cnt);
		atomic_set(&exynos_pcie->ep_pci_dev->enable_cnt, 0);
	}

	return 0;
}

static void exynos_pcie_resume_complete(struct device *dev)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	logbuffer_log(exynos_pcie->log, "resume_complete called");
	if (exynos_pcie->use_phy_isol_con &&
	    exynos_pcie->state == STATE_LINK_DOWN)
		exynos_pcie_phy_isolation(exynos_pcie, PCIE_PHY_ISOLATION);
	else if (exynos_pcie->use_pcieon_sleep && exynos_pcie->ep_pci_dev) {
		exynos_pcie->ep_pci_dev->dev.power.must_resume = exynos_pcie->pcie_must_resume;
		dev_info(dev, "Default must_resume value : %d\n",
				exynos_pcie->ep_pci_dev->dev.power.must_resume);
	}
}

#endif

static const struct dev_pm_ops exynos_pcie_rc_pm_ops = {
	.suspend_noirq	= exynos_pcie_rc_suspend_noirq,
	.resume_noirq	= exynos_pcie_rc_resume_noirq,
	.prepare	= exynos_pcie_suspend_prepare,
	.complete	= exynos_pcie_resume_complete,
};

static const struct of_device_id exynos_pcie_rc_of_match[] = {
	{ .compatible = "samsung,exynos-pcie-rc", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_pcie_rc_of_match);

static struct platform_driver exynos_pcie_rc_driver = {
	.probe		= exynos_pcie_rc_probe,
	.remove		= exynos_pcie_rc_remove,
	.driver = {
		.name		= "exynos-pcie-rc",
		.owner		= THIS_MODULE,
		.of_match_table = exynos_pcie_rc_of_match,
		.pm		= &exynos_pcie_rc_pm_ops,
	},
};
module_platform_driver(exynos_pcie_rc_driver);

MODULE_AUTHOR("Hongseock Kim <hongpooh.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung PCIe RC(RootComplex) controller driver");
MODULE_LICENSE("GPL v2");
