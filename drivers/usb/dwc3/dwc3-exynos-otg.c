// SPDX-License-Identifier: GPL-2.0
/**
 * dwc3-exynos-otg.c - DesignWare Exynos USB3 DRD Controller OTG
 *
 * Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/suspend.h>

#include "core.h"
#include "core-exynos.h"
#include "exynos-otg.h"
#include "io.h"
#include <linux/of_device.h>
#include "../../base/base.h"
#include <linux/usb/composite.h>
#include <linux/reboot.h>
#include "dwc3-exynos.h"
#include "dwc3-exynos-ldo.h"

#define OTG_NO_CONNECT		0
#define OTG_CONNECT_ONLY	1
#define OTG_DEVICE_CONNECT	2
#define LINK_DEBUG_L		(0x0C)
#define LINK_DEBUG_H		(0x10)
#define BUS_ACTIVITY_CHECK	(0x3F << 16)
#define READ_TRANS_OFFSET	10

/* -------------------------------------------------------------------------- */
static int dwc3_otg_reboot_notify(struct notifier_block *nb, unsigned long event, void *buf);
static struct notifier_block dwc3_otg_reboot_notifier = {
	.notifier_call = dwc3_otg_reboot_notify,
};

static int dwc3_otg_statemachine(struct otg_fsm *fsm)
{
	struct usb_otg *otg = fsm->otg;
	struct dwc3_otg	*dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3_exynos *exynos = dotg->exynos;
	struct device *dev = dotg->dwc->dev;
	enum usb_otg_state prev_state = otg->state;
	int ret = 0;

	if (dotg->fsm_reset) {
		if (otg->state == OTG_STATE_A_HOST) {
			otg_drv_vbus(fsm, 0);
			otg_start_host(fsm, 0);
		} else if (otg->state == OTG_STATE_B_PERIPHERAL) {
			otg_start_gadget(fsm, 0);
		}

		otg->state = OTG_STATE_UNDEFINED;
		goto exit;
	}

	switch (otg->state) {
	case OTG_STATE_UNDEFINED:
		if (fsm->id)
			otg->state = OTG_STATE_B_IDLE;
		else
			otg->state = OTG_STATE_A_IDLE;
		break;
	case OTG_STATE_B_IDLE:
		if (!fsm->id) {
			otg->state = OTG_STATE_A_IDLE;
		} else if (fsm->b_sess_vld) {
			exynos->retry_cnt = 0;
			ret = otg_start_gadget(fsm, 1);
			if (!ret)
				otg->state = OTG_STATE_B_PERIPHERAL;
			else
				dev_err(dev, "OTG SM: cannot start gadget\n");
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (!fsm->id || !fsm->b_sess_vld) {
			exynos->retry_cnt = REMOVED_RETRY_CNT;
			ret = otg_start_gadget(fsm, 0);
			if (!ret)
				otg->state = OTG_STATE_B_IDLE;
			else
				dev_err(dev, "OTG SM: cannot stop gadget\n");
		}
		break;
	case OTG_STATE_A_IDLE:
		if (fsm->id) {
			otg->state = OTG_STATE_B_IDLE;
		} else {
			ret = otg_start_host(fsm, 1);
			if (!ret) {
				otg_drv_vbus(fsm, 1);
				otg->state = OTG_STATE_A_HOST;
			} else {
				dev_err(dev, "OTG SM: cannot start host\n");
			}
		}
		break;
	case OTG_STATE_A_HOST:
		if (fsm->id) {
			otg_drv_vbus(fsm, 0);
			ret = otg_start_host(fsm, 0);
			if (!ret)
				otg->state = OTG_STATE_A_IDLE;
			else
				dev_err(dev, "OTG SM: cannot stop host\n");
		}
		break;
	default:
		dev_err(dev, "OTG SM: invalid state\n");
	}

exit:
	if (!ret)
		ret = (otg->state != prev_state);

	dev_dbg(dev, "OTG SM: %s => %s\n", usb_otg_state_string(prev_state),
		(ret > 0) ? usb_otg_state_string(otg->state) : "(no change)");

	return ret;
}

/* -------------------------------------------------------------------------- */

static struct dwc3_ext_otg_ops *dwc3_otg_exynos_rsw_probe(struct dwc3 *dwc)
{
	struct dwc3_ext_otg_ops *ops;
	bool ext_otg;

	ext_otg = dwc3_exynos_rsw_available(dwc->dev->parent);
	if (!ext_otg) {
		dev_err(dwc->dev, "failed to get ext_otg\n");
		return NULL;
	}

	/* Allocate and init otg instance */
	ops = devm_kzalloc(dwc->dev, sizeof(struct dwc3_ext_otg_ops),
			GFP_KERNEL);
	if (!ops)
		return NULL;

	ops->setup = dwc3_exynos_rsw_setup;
	ops->exit = dwc3_exynos_rsw_exit;
	ops->start = dwc3_exynos_rsw_start;

	dev_dbg(dwc->dev, "rsw_probe done\n");

	return ops;
}

static void dwc3_otg_set_mode(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

static void dwc3_otg_set_host_mode(struct dwc3_otg *dotg)
{
	struct dwc3 *dwc = dotg->dwc;
	u32 reg;

	if (dotg->regs) {
		reg = dwc3_readl(dotg->regs, DWC3_OCTL);
		reg &= ~DWC3_OTG_OCTL_PERIMODE;
		dwc3_writel(dotg->regs, DWC3_OCTL, reg);
	} else {
		dwc3_otg_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
	}
}

static void dwc3_otg_set_peripheral_mode(struct dwc3_otg *dotg)
{
	struct dwc3 *dwc = dotg->dwc;
	u32 reg;

	if (dotg->regs) {
		reg = dwc3_readl(dotg->regs, DWC3_OCTL);
		reg |= DWC3_OTG_OCTL_PERIMODE;
		dwc3_writel(dotg->regs, DWC3_OCTL, reg);
	} else {
		dwc3_otg_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
	}
}

static void dwc3_otg_drv_vbus(struct otg_fsm *fsm, int on)
{
	struct dwc3_otg	*dotg = container_of(fsm, struct dwc3_otg, fsm);
	int ret;

	if (IS_ERR(dotg->vbus_reg)) {
		dev_err(dotg->dwc->dev, "vbus regulator is not available\n");
		return;
	}

	if (on)
		ret = regulator_enable(dotg->vbus_reg);
	else
		ret = regulator_disable(dotg->vbus_reg);

	if (ret)
		dev_err(dotg->dwc->dev, "failed to turn Vbus %s\n",
						on ? "on" : "off");
}

void dwc3_otg_pm_ctrl(struct dwc3 *dwc, int onoff)
{
	u32 reg;

	if (onoff == 0) {
		/* Disable U1U2 */
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~(DWC3_DCTL_INITU1ENA | DWC3_DCTL_ACCEPTU1ENA |
				DWC3_DCTL_INITU2ENA | DWC3_DCTL_ACCEPTU2ENA);
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	} else {
		/* Enable U1U2 */
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg |= (DWC3_DCTL_INITU1ENA | DWC3_DCTL_ACCEPTU1ENA |
				DWC3_DCTL_INITU2ENA | DWC3_DCTL_ACCEPTU2ENA);
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	}
}

/*
 * owner 0 - USB
 * owner 1 - DP
 */
static void usb3_phy_control(struct dwc3_otg	*dotg,
		int owner, int on)
{
	struct dwc3	*dwc = dotg->dwc;
	struct device	*dev = dwc->dev;

	dev_dbg(dev, "USB3.0 PHY %s\n", on ? "on" : "off");

	if (on) {
		dwc3_core_susphy_set(dwc, 0);
		exynos_usbdrd_pipe3_enable(dwc->usb3_generic_phy);
		dwc3_core_susphy_set(dwc, 1);
	} else {
		dwc3_core_susphy_set(dwc, 0);
		exynos_usbdrd_pipe3_disable(dwc->usb3_generic_phy);
		dwc3_core_susphy_set(dwc, 1);
	}
}

int dwc3_otg_phy_enable(struct otg_fsm *fsm, int owner, bool on)
{
	struct usb_otg	*otg = fsm->otg;
	struct dwc3_otg	*dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3	*dwc = dotg->dwc;
	struct dwc3_exynos *exynos = dotg->exynos;
	struct device	*dev = dotg->dwc->dev;
	int ret = 0;
	u8 owner_bit = 0;
	int wait_counter = 0;

	mutex_lock(&dotg->lock);

	dev_dbg(dev, "combo phy=%d usb2 phy=%d owner=%d (usb:0 dp:1) on=%d\n",
		      dotg->combo_phy_control, dotg->usb2_phy_control,
		      owner, on);

	if (owner > 1)
		goto out;

	owner_bit = (1 << owner);

	/* To initialize dwc3_core, it always set DEVICE role */
	exynos->dwc->current_dr_role = DWC3_GCTL_PRTCAP_DEVICE;

	if (on) {
		if (dotg->combo_phy_control && dotg->usb2_phy_control) {
			dotg->combo_phy_control |= owner_bit;
			dotg->usb2_phy_control |= owner_bit;
			goto out;
		} else if (dotg->usb2_phy_control == 0) {
			if (dotg->pm_qos_int_val) {
				dev_dbg(dev, "pm_qos set value = %d\n", dotg->pm_qos_int_val);
				exynos_pm_qos_update_request(&dotg->pm_qos_int_req,
						dotg->pm_qos_int_val);
			}

			/*
			 * dwc3_otg_phy_enable function enables
			 * both combo phy and usb2 phy
			 */
			ret = pm_runtime_get_sync(dev);
			if (ret < 0) {
				dev_err(dev, "failed to resume exynos device\n");
				goto err;
			}

			exynos_usbdrd_phy_tune(dwc->usb2_generic_phy,
							dotg->otg.state);
			exynos_usbdrd_phy_tune(dwc->usb3_generic_phy,
							dotg->otg.state);

			ret = dwc3_exynos_core_init(dwc, exynos);
			if (ret) {
				dev_err(dev, "failed to reinitialize exynos core\n");
				goto err;
			}

			dotg->combo_phy_control |= owner_bit;
			dotg->usb2_phy_control |= owner_bit;
		} else { /* dotg->combo_phy_control == 0 */
			usb3_phy_control(dotg, owner, on);
			dotg->combo_phy_control |= owner_bit;
		}
	} else {
		dotg->combo_phy_control &= ~(owner_bit);
		dotg->usb2_phy_control &= ~(owner_bit);

		if (dotg->usb2_phy_control == 0
				&& dotg->combo_phy_control == 0) {

			/*
			 * Need to check why phy init_count has mismatch.
			 * Phy_init is called from usb_phy_roothub_init() in
			 * usb_add_hcd function(usb/core/hcd.c)
			 */
			dwc->usb2_generic_phy->init_count = 1;
			dwc->usb3_generic_phy->init_count = 1;
			dwc->usb2_generic_phy->power_count = 1;
			dwc->usb3_generic_phy->power_count = 1;

err:
			/* We must disable gadget here. (Ignore other job) */
			atomic_set(&dev->power.usage_count, 1);
			pm_runtime_put_sync_suspend(dev);

			if (dotg->pm_qos_int_val)
				exynos_pm_qos_update_request(&dotg->pm_qos_int_req, 0);

			msleep(500);

			/* Wait for end of runtime suspend */
			wait_counter = 0;
			while (exynos->dwc->current_dr_role !=
					DWC3_EXYNOS_IGNORE_CORE_OPS) {
				wait_counter++;
				msleep(20);

				if (wait_counter > 10) {
					dev_err(dev, "Can't wait runtime suspend!!!!\n");
					dev_err(dev, "RPM Usage Count : %d",
							dev->power.usage_count);
					break;
				}
			}
		}
	}
out:
	mutex_unlock(&dotg->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(dwc3_otg_phy_enable);

static int dwc3_otg_start_host(struct otg_fsm *fsm, int on)
{
	struct usb_otg	*otg = fsm->otg;
	struct dwc3_otg	*dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3	*dwc = dotg->dwc;
	struct device	*dev = dotg->dwc->dev;
	struct dwc3_exynos *exynos = dotg->exynos;
	static struct usb_gadget_driver *temp_gadget_driver;
	struct usb_composite_driver *composite;
	int ret = 0;
	int ret1 = -1;
	int wait_counter = 0;

	__pm_stay_awake(dotg->wakelock);

	if (on) {
		dotg->otg_connection = 1;
		exynos->need_dr_role = 1;
		while (dwc->gadget_driver == NULL) {
			wait_counter++;
			msleep(20);

			if (wait_counter > 50) {
				dev_err(dev, "Can't wait host start!\n");
				break;
			}
		}

		if (!dwc->xhci) {
			ret = dwc3_exynos_host_init(exynos);
			if (ret) {
				exynos->need_dr_role = 0;
				dev_err(dev, "%s: failed to init dwc3 host\n", __func__);
				goto err1;
			}
		}

		/* To ignore gadget operation, it set gadget_driver to NULL */
		temp_gadget_driver = dwc->gadget_driver;
		dwc->gadget_driver = NULL;

		ret = dwc3_otg_phy_enable(fsm, 0, on);
		exynos->need_dr_role = 0;
		if (ret) {
			dev_err(dwc->dev, "%s: failed to reinitialize core\n",
					__func__);
			goto err1;
		}

		/* To ignore gadget suspend/resume on host l2 suspend */
		exynos->dwc->current_dr_role = DWC3_EXYNOS_IGNORE_CORE_OPS;

		dwc3_otg_set_host_mode(dotg);

		ret = platform_device_add(dwc->xhci);
		if (ret) {
			dev_err(dev, "%s: cannot add xhci\n", __func__);
			goto err2;
		}

	} else {
		dotg->otg_connection = 0;

		if (!dwc->xhci) {
			dev_err(dev, "%s: stop USB host without xhci device\n",
				__func__);
			return -EINVAL;
		}

		if (dotg->dwc3_suspended) {
			dev_dbg(dev, "wait resume completion\n");
			ret1 = wait_for_completion_timeout(&dotg->resume_cmpl,
							msecs_to_jiffies(5000));
		}

		if (temp_gadget_driver) {
			composite = to_cdriver(temp_gadget_driver);
			if (composite && composite->gadget_driver.udc_name)
				dwc->gadget_driver = temp_gadget_driver;
		}

		dwc3_exynos_host_exit(exynos);
		dwc->xhci = NULL;

err2:
		ret = dwc3_otg_phy_enable(fsm, 0, on);
	}
err1:
	__pm_relax(dotg->wakelock);
	return ret;
}

static void dwc3_otg_retry_configuration(struct timer_list *t)
{
	struct dwc3_exynos *exynos = from_timer(exynos, t, usb_connect_timer);
	struct usb_gadget *gadget = exynos->dwc->gadget;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);

	dev_dbg(exynos->dev, "retry start: +++\n");

	if (cdev == NULL || exynos->retry_cnt == REMOVED_RETRY_CNT) {
		dev_dbg(exynos->dev, "Stop retry configuration(cdev is NULL) or Removed\n");
		return;
	}

	if (!cdev->config) {
		if (exynos->retry_cnt >= MAX_RETRY_CNT) {
			dev_err(exynos->dev, "%s: Re-try 5 times, But usb enumeration fail\n",
					__func__);
			return;
		}

		dev_dbg(exynos->dev, "%s: retry USB enumeration, retry count : %d\n",
				__func__, exynos->retry_cnt);

		dwc3_otg_usb_recovery_reconn(exynos);

		exynos->retry_cnt += 1;
	} else {
		exynos->retry_cnt = 0;
		dev_dbg(exynos->dev, "%s: already configuration done!!\n", __func__);
		return;
	}

	dev_dbg(exynos->dev, "retry done\n");
}

static int dwc3_otg_start_gadget(struct otg_fsm *fsm, int on)
{
	struct usb_otg	*otg = fsm->otg;
	struct dwc3_otg	*dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3	*dwc = dotg->dwc;
	struct dwc3_exynos *exynos = dotg->exynos;
	struct device	*dev = dotg->dwc->dev;
	int ret = 0;
	int wait_counter = 0;
	u32 evt_count;

	if (!otg->gadget) {
		dev_err(dev, "%s does not have any gadget\n", __func__);
		return -EINVAL;
	}

	if (on) {
		__pm_stay_awake(dotg->wakelock);
		exynos->vbus_state = true;
		while (dwc->gadget_driver == NULL) {
			wait_counter++;
			usleep_range(100, 200);

			if (wait_counter > 500) {
				dev_err(dev, "Can't wait gadget start!\n");
				break;
			}
		}

		exynos->need_dr_role = 1;
		ret = dwc3_otg_phy_enable(fsm, 0, on);
		exynos->need_dr_role = 0;
		if (ret) {
			dev_err(dev, "%s: failed to reinitialize core\n",
					__func__);
			goto err1;
		}

		dwc3_otg_set_peripheral_mode(dotg);

		dev_dbg(dev, "%s: start check usb configuration timer\n", __func__);
		timer_setup(&exynos->usb_connect_timer, dwc3_otg_retry_configuration, 0);
		mod_timer(&exynos->usb_connect_timer,
				jiffies + CHG_CONNECTED_DELAY_TIME);
	} else {
		exynos->vbus_state = false;
		del_timer_sync(&exynos->usb_connect_timer);

		/*
		 * we can extra work corresponding each functions by
		 * the following function.
		 */
		if (exynos->config.is_not_vbus_pad && exynos_usbdrd_get_ldo_status() &&
				!dotg->in_shutdown)
			dwc3_exynos_gadget_disconnect_proc(dwc);

		/* Wait until dwc connected is off */
		evt_count = dwc3_readl(dwc->regs, DWC3_GEVNTCOUNT(0));
		evt_count &= DWC3_GEVNTCOUNT_MASK;
		while (dwc->connected || evt_count) {
			wait_counter++;
			msleep(20);

			if (wait_counter > 20) {
				dev_err(dev, "Can't wait dwc disconnect!\n");
				break;
			}
			evt_count = dwc3_readl(dwc->regs, DWC3_GEVNTCOUNT(0));
			evt_count &= DWC3_GEVNTCOUNT_MASK;
			dev_dbg(dev, "%s: evt = %d\n", __func__, evt_count);
		}

		/*
		 * We can block udc core operation by the following flags.
		 *  - gadget->connected and gadget->deactivated
		 */
		if (exynos->extra_delay)
			msleep(100);

		ret = dwc3_otg_phy_enable(fsm, 0, on);
err1:
		__pm_relax(dotg->wakelock);
	}

	return 0;
}

static struct otg_fsm_ops dwc3_otg_fsm_ops = {
	.drv_vbus	= dwc3_otg_drv_vbus,
	.start_host	= dwc3_otg_start_host,
	.start_gadget	= dwc3_otg_start_gadget,
};

/* -------------------------------------------------------------------------- */

void dwc3_otg_run_sm(struct otg_fsm *fsm)
{
	struct dwc3_otg	*dotg = container_of(fsm, struct dwc3_otg, fsm);
	int		state_changed;

	/* Prevent running SM on early system resume */
	if (!dotg->ready)
		return;

	mutex_lock(&fsm->lock);

	do {
		state_changed = dwc3_otg_statemachine(fsm);
	} while (state_changed > 0);

	mutex_unlock(&fsm->lock);
}

/* Bind/Unbind the peripheral controller driver */
static int dwc3_otg_set_peripheral(struct usb_otg *otg,
				struct usb_gadget *gadget)
{
	struct dwc3_otg	*dotg = container_of(otg, struct dwc3_otg, otg);
	struct otg_fsm	*fsm = &dotg->fsm;
	struct device	*dev = dotg->dwc->dev;

	if (gadget) {
		dev_dbg(dev, "Binding gadget %s\n", gadget->name);

		otg->gadget = gadget;
	} else {
		dev_dbg(dev, "Unbinding gadget\n");

		mutex_lock(&fsm->lock);

		if (otg->state == OTG_STATE_B_PERIPHERAL) {
			/* Reset OTG Statemachine */
			dotg->fsm_reset = 1;
			dwc3_otg_statemachine(fsm);
			dotg->fsm_reset = 0;
		}
		otg->gadget = NULL;

		mutex_unlock(&fsm->lock);

		dwc3_otg_run_sm(fsm);
	}

	return 0;
}

static int dwc3_otg_get_id_state(struct dwc3_otg *dotg)
{
	u32 reg = dwc3_readl(dotg->regs, DWC3_OSTS);

	return !!(reg & DWC3_OTG_OSTS_CONIDSTS);
}

static int dwc3_otg_get_b_sess_state(struct dwc3_otg *dotg)
{
	u32 reg = dwc3_readl(dotg->regs, DWC3_OSTS);

	return !!(reg & DWC3_OTG_OSTS_BSESVALID);
}

static irqreturn_t dwc3_otg_interrupt(int irq, void *_dotg)
{
	struct dwc3_otg	*dotg = (struct dwc3_otg *)_dotg;
	struct otg_fsm	*fsm = &dotg->fsm;
	u32 oevt, handled_events = 0;
	irqreturn_t ret = IRQ_NONE;

	oevt = dwc3_readl(dotg->regs, DWC3_OEVT);

	/* ID */
	if (oevt & DWC3_OEVTEN_OTGCONIDSTSCHNGEVNT) {
		fsm->id = dwc3_otg_get_id_state(dotg);
		handled_events |= DWC3_OEVTEN_OTGCONIDSTSCHNGEVNT;
	}

	/* VBus */
	if (oevt & DWC3_OEVTEN_OTGBDEVVBUSCHNGEVNT) {
		fsm->b_sess_vld = dwc3_otg_get_b_sess_state(dotg);
		handled_events |= DWC3_OEVTEN_OTGBDEVVBUSCHNGEVNT;
	}

	if (handled_events) {
		dwc3_writel(dotg->regs, DWC3_OEVT, handled_events);
		ret = IRQ_WAKE_THREAD;
	}

	return ret;
}

static irqreturn_t dwc3_otg_thread_interrupt(int irq, void *_dotg)
{
	struct dwc3_otg	*dotg = (struct dwc3_otg *)_dotg;

	dwc3_otg_run_sm(&dotg->fsm);

	return IRQ_HANDLED;
}

static void dwc3_otg_enable_irq(struct dwc3_otg *dotg)
{
	/* Enable only connector ID status & VBUS change events */
	dwc3_writel(dotg->regs, DWC3_OEVTEN,
			DWC3_OEVTEN_OTGCONIDSTSCHNGEVNT |
			DWC3_OEVTEN_OTGBDEVVBUSCHNGEVNT);
}

static void dwc3_otg_disable_irq(struct dwc3_otg *dotg)
{
	dwc3_writel(dotg->regs, DWC3_OEVTEN, 0x0);
}

static void dwc3_otg_reset(struct dwc3_otg *dotg)
{
	/*
	 * OCFG[2] - OTG-Version = 0
	 * OCFG[1] - HNPCap = 0
	 * OCFG[0] - SRPCap = 0
	 */
	dwc3_writel(dotg->regs, DWC3_OCFG, 0x0);

	/*
	 * OCTL[6] - PeriMode = 1
	 * OCTL[5] - PrtPwrCtl = 0
	 * OCTL[4] - HNPReq = 0
	 * OCTL[3] - SesReq = 0
	 * OCTL[2] - TermSelDLPulse = 0
	 * OCTL[1] - DevSetHNPEn = 0
	 * OCTL[0] - HstSetHNPEn = 0
	 */
	dwc3_writel(dotg->regs, DWC3_OCTL, DWC3_OTG_OCTL_PERIMODE);

	/* Clear all otg events (interrupts) indications  */
	dwc3_writel(dotg->regs, DWC3_OEVT, (u32)DWC3_OEVT_CLEAR_ALL);
}

/**
 * dwc3_otg_start
 * @dwc: pointer to our controller context structure
 */
int dwc3_otg_start(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	struct dwc3_otg	*dotg = exynos->dotg;
	struct otg_fsm	*fsm = &dotg->fsm;
	int		ret;

	if (dotg->ext_otg_ops) {
		ret = dwc3_ext_otg_start(dotg);
		if (ret) {
			dev_err(dwc->dev, "failed to start external OTG\n");
			return ret;
		}
	} else {
		dotg->regs = dwc->regs;

		dwc3_otg_reset(dotg);

		dotg->fsm.id = dwc3_otg_get_id_state(dotg);
		dotg->fsm.b_sess_vld = dwc3_otg_get_b_sess_state(dotg);

		dotg->irq = platform_get_irq(to_platform_device(dwc->dev), 0);
		ret = devm_request_threaded_irq(dwc->dev, dotg->irq,
				dwc3_otg_interrupt,
				dwc3_otg_thread_interrupt,
				IRQF_SHARED, "dwc3-otg", dotg);
		if (ret) {
			dev_err(dwc->dev, "failed to request irq #%d --> %d\n",
					dotg->irq, ret);
			return ret;
		}

		dwc3_otg_enable_irq(dotg);
	}

	dotg->ready = 1;

	dwc3_otg_run_sm(fsm);

	return 0;
}

/**
 * dwc3_otg_stop
 * @dwc: pointer to our controller context structure
 */
void dwc3_otg_stop(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	struct dwc3_otg         *dotg = exynos->dotg;

	if (dotg->ext_otg_ops) {
		dwc3_ext_otg_stop(dotg);
	} else {
		dwc3_otg_disable_irq(dotg);
		free_irq(dotg->irq, dotg);
	}

	dotg->ready = 0;
}

/* -------------------------------------------------------------------------- */
static struct device_node *exynos_dwusb_parse_dt(void)
{
	struct device_node *np = NULL;

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9-dwusb");
	if (!np) {
		pr_err("%s: failed to get the usbdrd node\n", __func__);
		goto err;
	}
	return np;
err:
	return NULL;
}

static struct dwc3_exynos *exynos_dwusb_get_struct(void)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct device *dev;
	struct dwc3_exynos *exynos;

	np = exynos_dwusb_parse_dt();
	if (np) {
		pdev = of_find_device_by_node(np);
		dev = &pdev->dev;
		of_node_put(np);
		if (pdev) {
			exynos = dev->driver_data;
			return exynos;
		}
	}

	pr_err("%s: failed to get the platform_device\n", __func__);
	return NULL;
}

int dwc3_otg_host_enable(bool enabled)
{
	struct dwc3_exynos *exynos;
	struct otg_fsm  *fsm;

	exynos = exynos_dwusb_get_struct();
	if (!exynos) {
		pr_err("%s: error exynos_dwusb_get_struct\n", __func__);
		return -ENODEV;
	}

	fsm = &exynos->dotg->fsm;
	fsm->id = !enabled;

	dev_dbg(exynos->dev, "%s: %s host\n", __func__, enabled ? "enable" : "disable");
	dwc3_otg_run_sm(fsm);

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_otg_host_enable);

static int dwc3_otg_reboot_notify(struct notifier_block *nb, unsigned long event, void *buf)
{
	struct dwc3_exynos *exynos;
	struct dwc3_otg *dotg;

	exynos = exynos_dwusb_get_struct();
	if (!exynos)
		return -ENODEV;

	dotg = exynos->dotg;

	switch (event) {
	case SYS_HALT:
	case SYS_RESTART:
	case SYS_POWER_OFF:
		exynos->dwc->current_dr_role = DWC3_EXYNOS_IGNORE_CORE_OPS;
		dotg->in_shutdown = true;
		break;
	}

	return 0;
}

u32 dwc3_otg_is_connect(void)
{
	struct dwc3_exynos *exynos;
	struct dwc3_otg *dotg;

	exynos = exynos_dwusb_get_struct();
	if (!exynos) {
		dev_err(exynos->dev, "[%s] error\n", __func__);
		return -ENODEV;
	}
	dotg = exynos->dotg;

	if (!dotg->otg_connection)
		return OTG_NO_CONNECT;
	else
		return OTG_DEVICE_CONNECT;
}
EXPORT_SYMBOL_GPL(dwc3_otg_is_connect);

int dwc3_otg_get_idle_ip_index(void)
{
	struct dwc3_exynos *exynos;

	exynos = exynos_dwusb_get_struct();

	if (exynos == NULL)
		return -ENODEV;

	return exynos->idle_ip_index;
}
EXPORT_SYMBOL_GPL(dwc3_otg_get_idle_ip_index);

static void dwc3_otg_recovery_reconnection(struct work_struct *w)
{
	struct dwc3_otg *dotg = container_of(w, struct dwc3_otg,
					     recov_work);
	struct dwc3_exynos *exynos = dotg->exynos;
	struct otg_fsm	*fsm = &dotg->fsm;
	int ret = 0;

	__pm_stay_awake(dotg->reconn_wakelock);
	/* Lock to avoid real cable insert/remove operation. */
	mutex_lock(&fsm->lock);

	/* To ignore PHY disable */
	dwc3_otg_phy_enable(fsm, DWC3_PHY_OWNER_EMEG, 1);

	if (dotg->otg_connection == 1) {
		pr_err("Recovery Host Reconnection\n");
		ret = dwc3_otg_start_host(fsm, 0);
		if (ret < 0) {
			pr_err("Cable was already disconnected!!\n");
			goto emeg_out;
		}
	} else {
		pr_err("Recovery Gadget Reconnection\n");
		if (exynos->vbus_state == false) {
			pr_err("Cable was already disconnected!!\n");
			goto emeg_out;
		}
		dwc3_otg_start_gadget(fsm, 0);
	}

	msleep(50);
	if (dotg->otg_connection == 1)
		dwc3_otg_start_host(fsm, 1);
	else
		dwc3_otg_start_gadget(fsm, 1);

emeg_out:
	dwc3_otg_phy_enable(fsm, DWC3_PHY_OWNER_EMEG, 0);

	mutex_unlock(&fsm->lock);
	__pm_relax(dotg->reconn_wakelock);
}

int dwc3_otg_usb_recovery_reconn(struct dwc3_exynos *exynos)
{
	struct dwc3_otg *dotg = exynos->dotg;

	if (exynos == NULL) {
		pr_err("WARNING : exynos is NULL\n");
		return -ENODEV;
	}

	schedule_work(&dotg->recov_work);

	return 0;
}

static int dwc3_otg_pm_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct dwc3_otg *dotg
		= container_of(nb, struct dwc3_otg, pm_nb);

	switch (action) {
	case PM_SUSPEND_PREPARE:
		dotg->dwc3_suspended = 1;
		reinit_completion(&dotg->resume_cmpl);
		break;
	case PM_POST_SUSPEND:
		dotg->dwc3_suspended = 0;
		complete(&dotg->resume_cmpl);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

int dwc3_exynos_otg_init(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	struct dwc3_otg *dotg = exynos->dotg;
	struct dwc3_ext_otg_ops *ops = NULL;
	int ret = 0;

	/* EXYNOS SoCs don't have HW OTG, but it supports SW OTG. */
	ops = dwc3_otg_exynos_rsw_probe(dwc);
	if (!ops)
		return 0;

	/* Allocate and init otg instance */
	dotg = devm_kzalloc(dwc->dev, sizeof(struct dwc3_otg), GFP_KERNEL);
	if (!dotg)
		return -ENOMEM;

	/* This reference is used by dwc3 modules for checking otg existence */
	exynos->dotg = dotg;
	dotg->dwc = dwc;
	dotg->exynos = exynos;

	ret = of_property_read_u32(exynos->dev->of_node, "usb-pm-qos-int",
				   &dotg->pm_qos_int_val);
	if (ret < 0) {
		dev_err(dwc->dev, "couldn't read usb-pm-qos-int %s node, error = %d\n",
			dwc->dev->of_node->name, ret);
		dotg->pm_qos_int_val = 0;
	} else {
		exynos_pm_qos_add_request(&dotg->pm_qos_int_req,
					  PM_QOS_DEVICE_THROUGHPUT, 0);
	}

	dotg->ext_otg_ops = ops;

	dotg->otg.set_peripheral = dwc3_otg_set_peripheral;
	dotg->otg.set_host = NULL;

	dotg->otg.state = OTG_STATE_UNDEFINED;
	dotg->in_shutdown = false;

	mutex_init(&dotg->fsm.lock);
	dotg->fsm.ops = &dwc3_otg_fsm_ops;
	dotg->fsm.otg = &dotg->otg;

	INIT_WORK(&dotg->recov_work, dwc3_otg_recovery_reconnection);
	dotg->vbus_reg = devm_regulator_get(dwc->dev, "dwc3-vbus");
	if (IS_ERR(dotg->vbus_reg))
		dev_err(dwc->dev, "failed to obtain vbus regulator\n");

	if (dotg->ext_otg_ops) {
		dev_dbg(dwc->dev, "%s, dwc3_ext_otg_setup call\n", __func__);
		ret = dwc3_ext_otg_setup(dotg);
		if (ret) {
			dev_err(dwc->dev, "failed to setup OTG\n");
			return ret;
		}
	}

	dotg->wakelock = wakeup_source_register(dwc->dev, "dwc3-otg");
	dotg->reconn_wakelock = wakeup_source_register(dwc->dev,
				"dwc3-reconnection");
	mutex_init(&dotg->lock);

	init_completion(&dotg->resume_cmpl);
	dotg->dwc3_suspended = 0;
	dotg->pm_nb.notifier_call = dwc3_otg_pm_notifier;
	register_pm_notifier(&dotg->pm_nb);

	ret = register_reboot_notifier(&dwc3_otg_reboot_notifier);
	if (ret)
		dev_err(dwc->dev, "failed register reboot notifier\n");

	dev_dbg(dwc->dev, "otg_init done\n");

	return 0;
}

void dwc3_exynos_otg_exit(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	struct dwc3_otg *dotg = exynos->dotg;

	if (!dotg->ext_otg_ops)
		return;

	unregister_pm_notifier(&dotg->pm_nb);

	dwc3_ext_otg_exit(dotg);

	wakeup_source_unregister(dotg->wakelock);
	wakeup_source_unregister(dotg->reconn_wakelock);
	free_irq(dotg->irq, dotg);
	dotg->otg.state = OTG_STATE_UNDEFINED;
	kfree(dotg);
	exynos->dotg = NULL;
}
