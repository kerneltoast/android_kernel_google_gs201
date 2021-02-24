// SPDX-License-Identifier: GPL-2.0
/**
 * dwc3-exynos.c - Samsung Exynos DWC3 Specific Glue layer
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Anton Tikhomirov <av.tikhomirov@samsung.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/usb/otg.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/workqueue.h>
#include <linux/usb/gadget.h>

#include <linux/usb/of.h>

#include "core.h"
#include "core_exynos.h"
#include "dwc3-exynos-ldo.h"
#include "io.h"
#include "gadget.h"

#include <linux/io.h>
#include <linux/usb/otg-fsm.h>
#include <linux/extcon.h>

#include <linux/suspend.h>

#include "otg.h"
#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif

#include <soc/google/exynos-cpupm.h>

#define DWC3_DEFAULT_AUTOSUSPEND_DELAY	5000 /* ms */

static const struct of_device_id exynos_dwc3_match[] = {
	{
		.compatible = "samsung,exynos-dwusb",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_dwc3_match);

/**
 * of_usb_get_suspend_clk_freq - Get suspend clock frequency
 *
 * USB3 core needs 16KHz clock for a small part that operates
 * when the SS PHY is in its lowest power (P3) state.
 * USB3 core receives suspend clock and divides it to make 16KHz clock.
 */
unsigned int of_usb_get_suspend_clk_freq(struct device *dev)
{
	unsigned int freq;
	int err;

	err = device_property_read_u32(dev, "suspend_clk_freq", &freq);
	if (err < 0)
		return 0;

	return freq;
}
EXPORT_SYMBOL_GPL(of_usb_get_suspend_clk_freq);

static int dwc3_exynos_clk_get(struct dwc3_exynos *exynos)
{
	struct device *dev = exynos->dev;
	const char **clk_ids;
	struct clk *clk;
	int clk_count;
	int ret, i;

	clk_count = of_property_count_strings(dev->of_node, "clock-names");
	if (IS_ERR_VALUE((unsigned long)clk_count)) {
		dev_err(dev, "invalid clk list in %s node", dev->of_node->name);
		return -EINVAL;
	}

	clk_ids = devm_kmalloc(dev,
			       (clk_count + 1) * sizeof(const char *),
				GFP_KERNEL);
	if (!clk_ids) {
		dev_err(dev, "failed to alloc for clock ids");
		return -ENOMEM;
	}

	for (i = 0; i < clk_count; i++) {
		ret = of_property_read_string_index(dev->of_node, "clock-names",
						    i, &clk_ids[i]);
		if (ret) {
			dev_err(dev, "failed to read clocks name %d from %s node\n",
				i, dev->of_node->name);
			return ret;
		}
		/*
		 * Check Bus clock to get clk node from DT.
		 * CAUTION : Bus clock SHOULD be defiend at the last.
		 */
		if (!strncmp(clk_ids[i], "bus", 3)) {
			dev_info(dev, "BUS clock is defined.\n");
			exynos->bus_clock = devm_clk_get(exynos->dev, clk_ids[i]);
			if (IS_ERR_OR_NULL(exynos->bus_clock))
				dev_err(dev, "Can't get Bus clock.\n");
			else
				clk_count--;
		}
	}
	clk_ids[clk_count] = NULL;

	exynos->clocks = devm_kmalloc(exynos->dev,
				      clk_count * sizeof(struct clk *), GFP_KERNEL);
	if (!exynos->clocks)
		return -ENOMEM;

	for (i = 0; clk_ids[i]; i++) {
		clk = devm_clk_get(exynos->dev, clk_ids[i]);
		if (IS_ERR_OR_NULL(clk))
			goto err;

		exynos->clocks[i] = clk;
	}
	exynos->clocks[i] = NULL;

	return 0;

err:
	dev_err(exynos->dev, "couldn't get %s clock\n", clk_ids[i]);
	return -EINVAL;
}

static int dwc3_exynos_clk_prepare(struct dwc3_exynos *exynos)
{
	int i;
	int ret;

	for (i = 0; exynos->clocks[i]; i++) {
		ret = clk_prepare(exynos->clocks[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	dev_err(exynos->dev, "couldn't prepare clock[%d]\n", i);

	/* roll back */
	for (i = i - 1; i >= 0; i--)
		clk_unprepare(exynos->clocks[i]);

	return ret;
}

static int dwc3_exynos_clk_enable(struct dwc3_exynos *exynos)
{
	int i;
	int ret;

	for (i = 0; exynos->clocks[i]; i++) {
		ret = clk_enable(exynos->clocks[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	dev_err(exynos->dev, "couldn't enable clock[%d]\n", i);

	/* roll back */
	for (i = i - 1; i >= 0; i--)
		clk_disable(exynos->clocks[i]);

	return ret;
}

static void dwc3_exynos_clk_unprepare(struct dwc3_exynos *exynos)
{
	int i;

	for (i = 0; exynos->clocks[i]; i++)
		clk_unprepare(exynos->clocks[i]);
}

static void dwc3_exynos_clk_disable(struct dwc3_exynos *exynos)
{
	int i;

	for (i = 0; exynos->clocks[i]; i++)
		clk_disable(exynos->clocks[i]);
}

/* core setting */
void dwc3_core_config(struct dwc3 *dwc)
{
	u32 reg;

	/* AHB bus configuration */
	reg = dwc3_readl(dwc->regs, DWC3_GSBUSCFG0);
	reg |= (DWC3_GSBUSCFG0_INCRBRSTEN | DWC3_GSBUSCFG0_INCR16BRSTEN);
	/**
	 * AXI Bus' cache type configuration for DMA transfer.
	 * By below setting, cache type was set to Cacheable/Modifiable.
	 * From DWC USB3.0 Link version 2.20A, this cache type could be set.
	 */
	if (dwc->revision >= DWC3_REVISION_220A)
		reg |= (DWC3_GSBUSCFG0_DESWRREQINFO |
			DWC3_GSBUSCFG0_DATWRREQINFO |
			DWC3_GSBUSCFG0_DESRDREQINFO |
			DWC3_GSBUSCFG0_DATRDREQINFO);
	dwc3_writel(dwc->regs, DWC3_GSBUSCFG0, reg);

	reg = dwc3_readl(dwc->regs, DWC3_GSBUSCFG1);

#if defined(CONFIG_PHY_SAMSUNG_USB_GEN2)
	/*
	 * Setting MO request limit to 8 resolved ITMON issue
	 * on MTP and DM functions. Further investigation
	 * should be done by design team.
	 */
	reg |= (DWC3_GSBUSCFG1_BREQLIMIT(0x8));
	dwc3_writel(dwc->regs, DWC3_GSBUSCFG1, reg);
#endif

	/*
	 * WORKAROUND:
	 * For ss bulk-in data packet, when the host detects
	 * a DPP error or the internal buffer becomes full,
	 * it retries with an ACK TP Retry=1. Under the following
	 * conditions, the Retry=1 is falsely carried over to the next
	 * DWC3_GUCTL_USBHSTINAUTORETRYEN should be set to a one
	 * regardless of revision
	 * - There is only single active asynchronous SS EP at the time.
	 * - The active asynchronous EP is a Bulk IN EP.
	 * - The burst with the correctly Retry=1 ACK TP and
	 *   the next burst belong to the same transfer.
	 */
	reg = dwc3_readl(dwc->regs, DWC3_GUCTL);
	reg |= (DWC3_GUCTL_USBHSTINAUTORETRYEN);

	/* fix ITP interval time to 125us */
#if defined(CONFIG_PHY_SAMSUNG_USB_GEN2)
	reg &= ~DWC3_GUCTL_REFCLKPER_MASK;
	reg |= DWC3_GUCTL_REFCLKPER(0xF);
#elif defined(CONFIG_PHY_SAMSUNG_USB_GEN2_V4)
	reg &= ~DWC3_GUCTL_REFCLKPER_MASK;
	reg |= DWC3_GUCTL_REFCLKPER(0x34);
#endif

	if (dwc->sparse_transfer_control)
		reg |= DWC3_GUCTL_SPRSCTRLTRANSEN;

	if (dwc->no_extra_delay)
		reg |= DWC3_GUCTL_NOEXTRDL;

	if (dwc->usb_host_device_timeout) {
		reg &= ~DWC3_GUCTL_DTOUT_MASK;
		reg |= DWC3_GUCTL_DTOUT(dwc->usb_host_device_timeout);
	}

	dwc3_writel(dwc->regs, DWC3_GUCTL, reg);
	if (dwc->revision >= DWC3_REVISION_190A &&
	    dwc->revision <= DWC3_REVISION_210A) {
		reg = dwc3_readl(dwc->regs, DWC3_GRXTHRCFG);
		reg &= ~(DWC3_GRXTHRCFG_USBRXPKTCNT_MASK |
			DWC3_GRXTHRCFG_USBMAXRXBURSTSIZE_MASK);
		reg |= (DWC3_GRXTHRCFG_USBRXPKTCNTSEL |
			DWC3_GRXTHRCFG_USBRXPKTCNT(3) |
			DWC3_GRXTHRCFG_USBMAXRXBURSTSIZE(3));
		dwc3_writel(dwc->regs, DWC3_GRXTHRCFG, reg);
	}

	/*
	 * WORKAROUND: DWC3 revisions 2.10a and earlier have a bug
	 * The delay of the entry to a low power state such that
	 * for applications where the link stays in a non-U0 state
	 * for a short duration(< 1 microsecond),
	 * the local PHY does not enter the low power state prior
	 * to receiving a potential LFPS wakeup.
	 * This causes the PHY CDR (Clock and Data Recovery) operation
	 * to be unstable for some Synopsys PHYs.
	 * The proposal now is to change the default and the recommended value
	 * for GUSB3PIPECTL[21:19] in the RTL from 3'b100 to a minimum of 3'b001
	 */
	if (dwc->revision <= DWC3_REVISION_210A) {
		reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
		reg &= ~(DWC3_GUSB3PIPECTL_DEP1P2P3_MASK);
		reg |= (DWC3_GUSB3PIPECTL_DEP1P2P3_EN);
		dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);
	}

	if (dwc->revision >= DWC3_REVISION_250A) {
		reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
		reg |= DWC3_GUSB3PIPECTL_DISRXDETINP3;
		dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);
	}

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	if (dwc->ux_exit_in_px_quirk)
		reg |= DWC3_GUSB3PIPECTL_UX_EXIT_PX;
	if (dwc->elastic_buf_mode_quirk)
		reg |= DWC3_ELASTIC_BUFFER_MODE;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	if (dwc->revision > DWC31_REVISION_120A) {
#if defined(CONFIG_PHY_SAMSUNG_USB_GEN2)
		reg = dwc3_readl(dwc->regs, DWC3_LLUCTL);
		reg |= (DWC3_PENDING_HP_TIMER_US(0xb) | DWC3_EN_US_HP_TIMER);
		reg |= DWC3_FORCE_GEN1;
		dwc3_writel(dwc->regs, DWC3_LLUCTL, reg);

		reg = dwc3_readl(dwc->regs, DWC3_LSKIPFREQ);
		reg |= (DWC3_PM_ENTRY_TIMER_US(0x9) |
			DWC3_PM_LC_TIMER_US(0x5) | DWC3_EN_PM_TIMER_US);
		dwc3_writel(dwc->regs, DWC3_LSKIPFREQ, reg);
#elif defined(CONFIG_PHY_SAMSUNG_USB_GEN2_V4)
		reg = dwc3_readl(dwc->regs, DWC3_LLUCTL);
		reg &= ~(DWC3_LLUCTL_TX_TS1_CNT_MASK);
		reg |= (DWC3_PENDING_HP_TIMER_US(0xb) | DWC3_EN_US_HP_TIMER) |
		    (DWC3_LLUCTL_PIPE_RESET) | (DWC3_LLUCTL_LTSSM_TIMER_OVRRD) |
		    (DWC3_LLUCTL_TX_TS1_CNT(0x0));

		if (dwc->force_gen1)
			reg |= DWC3_FORCE_GEN1;

		dwc3_writel(dwc->regs, DWC3_LLUCTL, reg);

		reg = dwc3_readl(dwc->regs, DWC3_LSKIPFREQ);
		reg &= ~(DWC3_PM_LC_TIMER_US_MASK |
				DWC3_PM_ENTRY_TIMER_US_MASK);
		reg |= (DWC3_PM_ENTRY_TIMER_US(0x9) |
			DWC3_PM_LC_TIMER_US(0x5) | DWC3_EN_PM_TIMER_US);
		dwc3_writel(dwc->regs, DWC3_LSKIPFREQ, reg);

		reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
		reg &= ~DWC3_GUSB3PIPECTL_DISRXDETINP3;
		reg &= ~DWC3_GUSB3PIPECTL_RX_DETOPOLL;
		dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

		/*
		 * Use Default Value
		 * reg = dwc3_readl(dwc->regs, DWC3_LU3LFPSRXTIM);
		 * dwc3_writel(dwc->regs, DWC3_LU3LFPSRXTIM, reg);
		 */
		reg = dwc3_readl(dwc->regs, DWC3_GSBUSCFG0);
		reg |= (DWC3_GSBUSCFG0_INCR8BRSTEN |
				DWC3_GSBUSCFG0_INCR4BRSTEN);
		dwc3_writel(dwc->regs, DWC3_GSBUSCFG0, reg);

		reg = dwc3_readl(dwc->regs, DWC3_BU31RHBDBG);
		reg |= DWC3_BU31RHBDBG_TOUTCTL;
		dwc3_writel(dwc->regs, DWC3_BU31RHBDBG, reg);

		reg = dwc3_readl(dwc->regs, DWC3_GUCTL1);
		reg &= ~DWC3_GUCTL1_IP_GAP_ADD_ON_MASK;
		reg |= DWC3_GUCTL1_IP_GAP_ADD_ON(0x1);
		dwc3_writel(dwc->regs, DWC3_GUCTL1, reg);
#endif
	}

	/*
	 * WORKAROUND: DWC3 revisions 2.10a and earlier have a bug
	 * Race Condition in PORTSC Write Followed by Read
	 * If the software quickly does a read to the PORTSC,
	 * some fields (port status change related fields
	 * like OCC, etc.) may not have correct value
	 * due to the current way of handling these bits.
	 * After clearing the status register (for example, OCC) bit
	 * by writing PORTSC tregister, software can insert some delay
	 * (for example, 5 mac2_clk -> UTMI clock = 60 MHz ->
	 * (16.66 ns x 5 = 84ns)) before reading the PORTSC to check status.
	 */
	if (dwc->revision <= DWC3_REVISION_210A) {
		reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
		reg |= (DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT));
		dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
	}
}

/**
 * dwc3_soft_reset - Issue soft reset
 * @dwc: Pointer to our controller context structure
 */
int dwc3_soft_reset(struct dwc3 *dwc)
{
	unsigned long timeout;
	u32 reg;

	timeout = jiffies + msecs_to_jiffies(500);
	dwc3_writel(dwc->regs, DWC3_DCTL, DWC3_DCTL_CSFTRST);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			break;

		if (time_after(jiffies, timeout)) {
			dev_err(dwc->dev, "Reset Timed Out\n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	} while (true);

	return 0;
}

void dwc3_exynos_phy_setup(struct dwc3 *dwc)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));

	if (dwc->ux_exit_in_px_quirk)
		reg |= DWC3_GUSB3PIPECTL_UX_EXIT_PX;

	if (dwc->u1u2_exitfail_to_recov_quirk)
		reg |= DWC3_GUSB3PIPECTL_U1U2EXITFAIL_TO_RECOV;

	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));

	if (!dwc->dis_enblslpm_quirk)
		reg |= DWC3_GUSB2PHYCFG_ENBLSLPM;

	if (dwc->adj_sof_accuracy)
		reg &= ~DWC3_GUSB2PHYCFG_U2_FREECLK_EXISTS;

	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
}

int dwc3_exynos_core_init(struct dwc3 *dwc)
{
	u32 reg;

	dwc3_exynos_phy_setup(dwc);

	dwc3_core_config(dwc);

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);

	if (dwc->dis_u2_freeclk_exists_quirk)
		reg |= DWC3_GCTL_SOFITPSYNC;
	else
		reg &= ~DWC3_GCTL_SOFITPSYNC;

	if (dwc->adj_sof_accuracy)
		reg &= ~DWC3_GCTL_SOFITPSYNC;

	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	return 0;
}

int dwc3_gadget_vbus_session(struct usb_gadget *g, int is_active)
{
	struct dwc3 *dwc = gadget_to_dwc(g);
	unsigned long flags;
	int ret = 0;

	if (!dwc->dotg)
		return -EPERM;

	is_active = !!is_active;

	spin_lock_irqsave(&dwc->lock, flags);

	/* Mark that the vbus was powered */
	dwc->vbus_session = is_active;

	/*
	 * Check if upper level usb_gadget_driver was already registered with
	 * this udc controller driver (if dwc3_gadget_start was called).
	 * removed checking dwc->softconnect in vbus to resolve that
	 * run_stop isn't called permanantely when pm_runtime check
	 * is implemented in gadget_pullup and gadget_set_speed
	 */
	if (dwc->gadget_driver) {
		if (dwc->vbus_session) {
			/*
			 * Both vbus was activated by otg and pullup was
			 * signaled by the gadget driver.
			 * In this point, dwc->softconnect should be one
			 * thus set dwc->softconnect even if setting it here
			 * is conceptually wrong.
			 */
			dwc->softconnect = dwc->vbus_session;
			ret = dwc3_gadget_run_stop_vbus(dwc, 1, false);
		} else {
			ret = dwc3_gadget_run_stop_vbus(dwc, 0, false);
		}
	}

	spin_unlock_irqrestore(&dwc->lock, flags);

	if (ret)
		dev_err(dwc->dev, "dwc3 gadget run/stop error:%d\n", ret);

	return ret;
}

void dwc3_gadget_disconnect_proc(struct dwc3 *dwc)
{
	int			reg;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_INITU1ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	reg &= ~DWC3_DCTL_INITU2ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	if (dwc->gadget_driver && dwc->gadget_driver->disconnect)
		dwc->gadget_driver->disconnect(&dwc->gadget);

	dwc->start_config_issued = false;

	dwc->gadget.speed = USB_SPEED_UNKNOWN;
	dwc->setup_packet_pending = false;

	complete(&dwc->disconnect);
}

/* -------------------------------------------------------------------------- */

static struct dwc3_exynos *dwc3_exynos_match(struct device *dev)
{
	const struct of_device_id *matches = NULL;
	struct dwc3_exynos *exynos = NULL;

	if (!dev)
		return NULL;

	matches = exynos_dwc3_match;

	if (of_match_device(matches, dev))
		exynos = dev_get_drvdata(dev);

	return exynos;
}

bool dwc3_exynos_rsw_available(struct device *dev)
{
	struct dwc3_exynos *exynos;

	exynos = dwc3_exynos_match(dev);
	if (!exynos)
		return false;

	return true;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_rsw_available);

int dwc3_exynos_rsw_start(struct device *dev)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_exynos_rsw	*rsw = &exynos->rsw;

	dev_info(dev, "%s\n", __func__);

	/* B-device by default */
	rsw->fsm->id = 1;
	rsw->fsm->b_sess_vld = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_rsw_start);

void dwc3_exynos_rsw_stop(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
}
EXPORT_SYMBOL_GPL(dwc3_exynos_rsw_stop);

int dwc3_exynos_get_idle_ip_index(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	return exynos->idle_ip_index;
}

int dwc3_exynos_set_bus_clock(struct device *dev, int clk_level)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	if (!IS_ERR_OR_NULL(exynos->bus_clock)) {
		if (clk_level < 0) {
			dev_info(dev, "Set USB Bus clock to 66Mhz\n");
			clk_set_rate(exynos->bus_clock, 66666666);
		} else if (clk_level == 1) {
			dev_info(dev, "Set USB Bus clock to 177Mhz\n");
			clk_set_rate(exynos->bus_clock, 177750000);
		} else if (clk_level == 0) {
			dev_info(dev, "Set USB Bus clock to 266Mhz\n");
			clk_set_rate(exynos->bus_clock, 266625000);
		} else {
			dev_info(dev, "Unsupported clock level");
		}

		dev_info(dev, "Changed USB Bus clock %d\n",
			 clk_get_rate(exynos->bus_clock));
	}

	return 0;
}

static void dwc3_exynos_rsw_work(struct work_struct *w)
{
	struct dwc3_exynos_rsw	*rsw = container_of(w,
					struct dwc3_exynos_rsw, work);
	struct dwc3_exynos	*exynos = container_of(rsw,
					struct dwc3_exynos, rsw);

	dev_info(exynos->dev, "%s\n", __func__);

	dwc3_otg_run_sm(rsw->fsm);
}

int dwc3_exynos_rsw_setup(struct device *dev, struct otg_fsm *fsm)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_exynos_rsw	*rsw = &exynos->rsw;

	dev_dbg(dev, "%s\n", __func__);

	INIT_WORK(&rsw->work, dwc3_exynos_rsw_work);

	rsw->fsm = fsm;

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_rsw_setup);

void dwc3_exynos_rsw_exit(struct device *dev)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_exynos_rsw	*rsw = &exynos->rsw;

	dev_dbg(dev, "%s\n", __func__);

	cancel_work_sync(&rsw->work);

	rsw->fsm = NULL;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_rsw_exit);

/**
 * dwc3_exynos_id_event - receive ID pin state change event.
 * @state : New ID pin state.
 */
int dwc3_exynos_id_event(struct device *dev, int state)
{
	struct dwc3_exynos	*exynos;
	struct dwc3_exynos_rsw	*rsw;
	struct otg_fsm		*fsm;

	dev_dbg(dev, "EVENT: ID: %d\n", state);

	exynos = dev_get_drvdata(dev);
	if (!exynos)
		return -ENOENT;

	rsw = &exynos->rsw;

	fsm = rsw->fsm;
	if (!fsm)
		return -ENOENT;

	if (fsm->id != state) {
		fsm->id = state;
		schedule_work(&rsw->work);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_id_event);

/**
 * dwc3_exynos_vbus_event - receive VBus change event.
 * vbus_active : New VBus state, true if active, false otherwise.
 */
int dwc3_exynos_vbus_event(struct device *dev, bool vbus_active)
{
	struct dwc3_exynos	*exynos;
	struct dwc3_exynos_rsw	*rsw;
	struct otg_fsm		*fsm;

	dev_dbg(dev, "EVENT: VBUS: %sactive\n", vbus_active ? "" : "in");

	exynos = dev_get_drvdata(dev);
	if (!exynos)
		return -ENOENT;

	rsw = &exynos->rsw;

	fsm = rsw->fsm;
	if (!fsm)
		return -ENOENT;

	if (fsm->b_sess_vld != vbus_active) {
		fsm->b_sess_vld = vbus_active;
		schedule_work(&rsw->work);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_vbus_event);

/**
 * dwc3_exynos_phy_enable - received combo phy control.
 */
int dwc3_exynos_phy_enable(int owner, bool on)
{
	struct dwc3_exynos	*exynos;
	struct dwc3_exynos_rsw	*rsw;
	struct otg_fsm		*fsm;
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	int ret = 0;

	pr_info("%s owner=%d (usb:0 dp:1) on=%d +\n", __func__, owner, on);

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos-dwusb");
	if (np) {
		pdev = of_find_device_by_node(np);
		if (!pdev) {
			pr_err("%s we can't get platform device\n", __func__);
			ret = -ENODEV;
			goto err;
		}
		of_node_put(np);
	} else {
		pr_err("%s we can't get np\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	exynos = platform_get_drvdata(pdev);
	if (!exynos) {
		pr_err("%s we can't get drvdata\n", __func__);
		ret = -ENOENT;
		goto err;
	}

	rsw = &exynos->rsw;

	fsm = rsw->fsm;
	if (!fsm) {
		pr_err("%s we can't get fsm\n", __func__);
		ret = -ENOENT;
		goto err;
	}

	dwc3_otg_phy_enable(fsm, owner, on);
err:
	pr_info("%s -\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_phy_enable);

static int dwc3_exynos_register_phys(struct dwc3_exynos *exynos)
{
	struct platform_device	*pdev;
	int			ret;

	pdev = platform_device_alloc("usb_phy_generic", PLATFORM_DEVID_AUTO);
	if (!pdev)
		return -ENOMEM;

	exynos->usb2_phy = pdev;

	pdev = platform_device_alloc("usb_phy_generic", PLATFORM_DEVID_AUTO);
	if (!pdev) {
		ret = -ENOMEM;
		goto err1;
	}

	exynos->usb3_phy = pdev;

	ret = platform_device_add(exynos->usb2_phy);
	if (ret)
		goto err2;

	ret = platform_device_add(exynos->usb3_phy);
	if (ret)
		goto err3;

	return 0;

err3:
	platform_device_del(exynos->usb2_phy);

err2:
	platform_device_put(exynos->usb3_phy);

err1:
	platform_device_put(exynos->usb2_phy);

	return ret;
}

static int dwc3_exynos_remove_child(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static int dwc3_exynos_vbus_notifier(struct notifier_block *nb,
				     unsigned long action, void *dev)
{
	struct dwc3_exynos *exynos = container_of(nb, struct dwc3_exynos, vbus_nb);

	if (!exynos->usb_data_enabled)
		return NOTIFY_OK;

	dev_info(exynos->dev, "%s: vbus:%d\n", __func__, action);

	dwc3_exynos_vbus_event(exynos->dev, action);

	return NOTIFY_OK;
}

static int dwc3_exynos_id_notifier(struct notifier_block *nb,
				   unsigned long action, void *dev)
{
	struct dwc3_exynos *exynos = container_of(nb, struct dwc3_exynos, id_nb);

	if (!exynos->usb_data_enabled)
		return NOTIFY_OK;

	dev_info(exynos->dev, "%s: host enabled:%d\n", __func__, action);

	dwc3_exynos_id_event(exynos->dev, !action);

	return NOTIFY_OK;
}

static int dwc3_exynos_extcon_register(struct dwc3_exynos *exynos)
{
	struct device *dev = exynos->dev;
	int ret = 0;

	if (!of_property_read_bool(dev->of_node, "extcon"))
		return -EINVAL;

	exynos->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR_OR_NULL(exynos->edev)) {
		dev_err(exynos->dev, "couldn't get extcon\n");
		return exynos->edev ? PTR_ERR(exynos->edev) : -ENODEV;
	}

	exynos->vbus_nb.notifier_call = dwc3_exynos_vbus_notifier;
	ret = extcon_register_notifier(exynos->edev, EXTCON_USB, &exynos->vbus_nb);

	if (ret < 0) {
		dev_err(exynos->dev, "couldn't register notifier for EXTCON_USB\n");
		return ret;
	}

	exynos->id_nb.notifier_call = dwc3_exynos_id_notifier;
	ret = extcon_register_notifier(exynos->edev, EXTCON_USB_HOST, &exynos->id_nb);

	if (ret < 0)
		dev_err(exynos->dev, "couldn't register notifier for EXTCON_USB_HOST\n");

	return ret;
}

static int dwc3_exynos_probe(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos;
	struct device		*dev = &pdev->dev;
	//struct device_node	*node = dev->of_node;
#ifdef USB_USE_IOCOHERENCY
	struct regmap *reg_sysreg;
#endif
	int			ret;

	pr_info("%s: +++\n", __func__);
	exynos = devm_kzalloc(dev, sizeof(*exynos), GFP_KERNEL);
	if (!exynos)
		return -ENOMEM;

	ret = dma_set_mask(dev, DMA_BIT_MASK(36));
	if (ret) {
		pr_err("dma set mask ret = %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, exynos);

	exynos->dev = dev;

	exynos->idle_ip_index = exynos_get_idle_ip_index(dev_name(dev));
	pr_info("%s, usb idle ip = %d\n", __func__,
			exynos->idle_ip_index);
	exynos_update_ip_idle_status(exynos->idle_ip_index, 0);

	ret = dwc3_exynos_clk_get(exynos);
	if (ret)
		return ret;

	ret = dwc3_exynos_clk_prepare(exynos);
	if (ret)
		return ret;

	ret = dwc3_exynos_clk_enable(exynos);
	if (ret) {
		dwc3_exynos_clk_unprepare(exynos);
		return ret;
	}

	ret = dwc3_exynos_extcon_register(exynos);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon\n");
		ret = -EPROBE_DEFER;
		goto err2;
	}

	ret = dwc3_exynos_register_phys(exynos);
	if (ret) {
		dev_err(dev, "couldn't register PHYs\n");
		goto err2;
	}

	pm_runtime_set_active(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, DWC3_DEFAULT_AUTOSUSPEND_DELAY);
	pm_runtime_enable(dev);

	exynos_usbdrd_ldo_manual_control(1);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto err1;

	ret = dwc3_probe(pdev, exynos);
	if (ret < 0)
		goto err1;

	ret = pm_runtime_put_sync(dev);
	dev_dbg(dev, "%s, pm_runtime_put_sync = %d\n",
		__func__, ret);

#ifdef USB_USE_IOCOHERENCY
	dev_info(dev, "Configure USB sharability.\n");
	reg_sysreg = syscon_regmap_lookup_by_phandle(dev->of_node,
						     "samsung,sysreg-hsi0");
	if (IS_ERR(reg_sysreg))
		dev_err(dev, "Failed to lookup Sysreg regmap\n");
	regmap_update_bits(reg_sysreg, 0x704, 0x6, 0x6);
#endif

	/* set the initial value */
	exynos->usb_data_enabled = true;

	/*
	 * To avoid missing notification in kernel booting check extcon
	 * state to run state machine.
	 */
	if (extcon_get_state(exynos->edev, EXTCON_USB))
		dwc3_exynos_vbus_event(exynos->dev, 1);
	else if (extcon_get_state(exynos->edev, EXTCON_USB_HOST))
		dwc3_exynos_id_event(exynos->dev, 0);

	pr_info("%s: ---\n", __func__);
	return 0;

err1:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
err2:
	dwc3_exynos_clk_disable(exynos);
	dwc3_exynos_clk_unprepare(exynos);
	pr_info("%s err = %d\n", __func__, ret);

	return ret;
}

static int dwc3_exynos_remove(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos = platform_get_drvdata(pdev);

	device_for_each_child(&pdev->dev, NULL, dwc3_exynos_remove_child);
	platform_device_unregister(exynos->usb2_phy);
	platform_device_unregister(exynos->usb3_phy);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev)) {
		dwc3_exynos_clk_disable(exynos);
		pm_runtime_set_suspended(&pdev->dev);
	}

	dwc3_exynos_clk_unprepare(exynos);

	return 0;
}

#ifdef CONFIG_PM
static int dwc3_exynos_runtime_suspend(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);

	if (!exynos)
		return 0;

	dwc3_exynos_clk_disable(exynos);

	/* inform what USB state is idle to IDLE_IP */
	exynos_update_ip_idle_status(exynos->idle_ip_index, 1);

	return 0;
}

static int dwc3_exynos_runtime_resume(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	int ret = 0;

	dev_info(dev, "%s\n", __func__);
	if (!exynos)
		return 0;

	/* inform what USB state is not idle to IDLE_IP */
	exynos_update_ip_idle_status(exynos->idle_ip_index, 0);

	ret = dwc3_exynos_clk_enable(exynos);
	if (ret) {
		dev_err(dev, "%s: clk_enable failed\n", __func__);
		return ret;
	}

	pm_runtime_mark_last_busy(dev);
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int dwc3_exynos_suspend(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_suspended(dev))
		return 0;

	pr_info("[%s]\n", __func__);

	pinctrl_pm_select_sleep_state(dev);

	dwc3_exynos_clk_disable(exynos);

	return 0;
}

static int dwc3_exynos_resume(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	int		ret;

	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_suspended(dev))
		return 0;

	ret = dwc3_exynos_clk_enable(exynos);
	if (ret) {
		dev_err(dev, "%s: clk_enable failed\n", __func__);
		return ret;
	}

	/* runtime set active to reflect active state. */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
}

static const struct dev_pm_ops dwc3_exynos_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_exynos_suspend, dwc3_exynos_resume)
	SET_RUNTIME_PM_OPS(dwc3_exynos_runtime_suspend,
			   dwc3_exynos_runtime_resume, NULL)
};

#define DEV_PM_OPS	(&dwc3_exynos_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver dwc3_exynos_driver = {
	.probe		= dwc3_exynos_probe,
	.remove		= dwc3_exynos_remove,
	.driver		= {
		.name	= "exynos-dwc3",
		.of_match_table = exynos_dwc3_match,
		.pm	= DEV_PM_OPS,
	},
};

module_platform_driver(dwc3_exynos_driver);

MODULE_AUTHOR("Anton Tikhomirov <av.tikhomirov@samsung.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 Exynos Glue Layer");
