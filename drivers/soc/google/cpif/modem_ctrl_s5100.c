// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/modem_notifier.h>
#include <linux/clk.h>
#include <linux/pci.h>
#include <linux/regulator/consumer.h>
#include <soc/google/acpm_mfd.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/time.h>
#include <linux/timer.h>

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic.h>
#include <linux/muic/muic_notifier.h>
#endif

#include <linux/exynos-pci-ctrl.h>
#include <linux/shm_ipc.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_ctrl.h"
#include "link_device.h"
#include "link_device_memory.h"
#include "s51xx_pcie.h"

#if IS_ENABLED(CONFIG_EXYNOS_BUSMONITOR)
#include <linux/exynos-busmon.h>
#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_S2MPU)
#include <soc/google/exynos-s2mpu.h>
#endif

#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
#include "../../../video/fbdev/exynos/dpu30/decon.h"
static int s5100_lcd_notifier(struct notifier_block *notifier,
		unsigned long event, void *v);
#endif /* CONFIG_CP_LCD_NOTIFIER */

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

#define RUNTIME_PM_AFFINITY_CORE 2

static struct modem_ctl *g_mc;

static int register_phone_active_interrupt(struct modem_ctl *mc);
static int register_cp2ap_wakeup_interrupt(struct modem_ctl *mc);

#if defined(MODULE)
/* GKI TODO */
#else /* MODULE */
static int sys_rev;
static int __init console_setup(char *str)
{
	get_option(&str, &sys_rev);
	mif_info("board_rev : %d\n", sys_rev);

	return 0;
}
__setup("androidboot.revision=", console_setup);
#endif /* MODULE */

static int s5100_reboot_handler(struct notifier_block *nb,
				    unsigned long l, void *p)
{
	struct modem_ctl *mc = container_of(nb, struct modem_ctl, reboot_nb);

	mif_info("Now is device rebooting..\n");

	mutex_lock(&mc->pcie_check_lock);
	mc->device_reboot = true;
	mutex_unlock(&mc->pcie_check_lock);

	return 0;
}

static void print_mc_state(struct modem_ctl *mc)
{
	int pwr  = mif_gpio_get_value(&mc->s5100_gpio_cp_pwr, false);
	int reset = mif_gpio_get_value(&mc->s5100_gpio_cp_reset, false);
	int pshold = mif_gpio_get_value(&mc->s5100_gpio_cp_ps_hold, false);

	int ap_wakeup = mif_gpio_get_value(&mc->s5100_gpio_ap_wakeup, false);
	int cp_wakeup = mif_gpio_get_value(&mc->s5100_gpio_cp_wakeup, false);

	int dump = mif_gpio_get_value(&mc->s5100_gpio_cp_dump_noti, false);
	int ap_status = mif_gpio_get_value(&mc->s5100_gpio_ap_status, false);
	int phone_active = mif_gpio_get_value(&mc->s5100_gpio_phone_active, false);

	mif_debug("%s: %ps:GPIO - pwr:%d rst:%d phd:%d aw:%d cw:%d dmp:%d ap_status:%d phone_state:%d\n",
		mc->name, CALLER, pwr, reset, pshold, ap_wakeup, cp_wakeup,
		dump, ap_status, phone_active);
}

static void pcie_clean_dislink(struct modem_ctl *mc)
{
#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
	if (mc->pcie_voice_call_on) {
		modem_notify_event(MODEM_EVENT_RESET, mc);
		mc->pcie_voice_call_on = false;
	}
#endif

	if (mc->pcie_powered_on)
		s5100_poweroff_pcie(mc, true);

	if (!mc->pcie_powered_on)
		mif_err("Link is disconnected!!!\n");
}

static void cp2ap_wakeup_work(struct work_struct *work)
{
	struct modem_ctl *mc = container_of(work, struct modem_ctl, wakeup_work);
	static ktime_t cp2ap_wakeup_time;
	unsigned long flags;

	if (mc->phone_state == STATE_CRASH_EXIT)
		return;

	cp2ap_wakeup_time = ktime_get_boottime();

	spin_lock_irqsave(&mc->power_stats_lock, flags);
	mc->cp_power_stats.last_exit_timestamp_usec = ktime_to_us(cp2ap_wakeup_time);
	mc->cp_power_stats.duration_usec += (mc->cp_power_stats.last_exit_timestamp_usec -
			mc->cp_power_stats.last_entry_timestamp_usec);
	spin_unlock_irqrestore(&mc->power_stats_lock, flags);

	s5100_poweron_pcie(mc);
}

static void cp2ap_suspend_work(struct work_struct *work)
{
	struct modem_ctl *mc = container_of(work, struct modem_ctl, suspend_work);
	static ktime_t cp2ap_suspend_time;
	unsigned long flags;

	if (mc->phone_state == STATE_CRASH_EXIT)
		return;

	cp2ap_suspend_time = ktime_get_boottime();

	spin_lock_irqsave(&mc->power_stats_lock, flags);
	mc->cp_power_stats.last_entry_timestamp_usec = ktime_to_us(cp2ap_suspend_time);
	mc->cp_power_stats.count++;
	spin_unlock_irqrestore(&mc->power_stats_lock, flags);

	s5100_poweroff_pcie(mc, false);
}

static ssize_t power_stats_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	ssize_t count = 0;
	unsigned long flags;
	u64 adjusted_duration_usec = mc->cp_power_stats.duration_usec;

	spin_lock_irqsave(&mc->power_stats_lock, flags);
	if (mc->cp_power_stats.last_entry_timestamp_usec >
			mc->cp_power_stats.last_exit_timestamp_usec) {
		u64 now_usec = ktime_to_us(ktime_get_boottime());
		adjusted_duration_usec += now_usec -
			mc->cp_power_stats.last_entry_timestamp_usec;
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count, "SLEEP:\n");
	count += scnprintf(&buf[count], PAGE_SIZE - count, " count: 0x%llx\n",
		mc->cp_power_stats.count);
	count += scnprintf(&buf[count], PAGE_SIZE - count, " duration_usec: 0x%llx\n",
		adjusted_duration_usec);
	count += scnprintf(&buf[count], PAGE_SIZE - count, " last_entry_timestamp_usec: 0x%llx\n",
		mc->cp_power_stats.last_entry_timestamp_usec);
	count += scnprintf(&buf[count], PAGE_SIZE - count, " last_exit_timestamp_usec: 0x%llx\n",
		mc->cp_power_stats.last_exit_timestamp_usec);
	spin_unlock_irqrestore(&mc->power_stats_lock, flags);

	return count;
}

static DEVICE_ATTR_RO(power_stats);

static struct attribute *modem_attrs[] = {
	&dev_attr_power_stats.attr,
	NULL,
};

static const struct attribute_group modem_group = {
	.attrs = modem_attrs,
	.name = "modem",
};

#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
static void voice_call_on_work(struct work_struct *work)
{
	struct modem_ctl *mc = container_of(work, struct modem_ctl, call_on_work);

	mutex_lock(&mc->pcie_check_lock);
	if (!mc->pcie_voice_call_on)
		goto exit;

	if (mc->pcie_powered_on &&
			(s51xx_check_pcie_link_status(mc->pcie_ch_num) != 0)) {
		if (cpif_wake_lock_active(mc->ws)) {
			mif_info("voice call on release wakelock\n");
			cpif_wake_unlock(mc->ws);
		}
	}

exit:
	mif_info("wakelock active = %d, voice status = %d\n",
		cpif_wake_lock_active(mc->ws), mc->pcie_voice_call_on);
	mutex_unlock(&mc->pcie_check_lock);
}

static void voice_call_off_work(struct work_struct *work)
{
	struct modem_ctl *mc = container_of(work, struct modem_ctl, call_off_work);

	mutex_lock(&mc->pcie_check_lock);
	if (mc->pcie_voice_call_on)
		goto exit;

	if (mc->pcie_powered_on &&
			(s51xx_check_pcie_link_status(mc->pcie_ch_num) != 0)) {
		if (!cpif_wake_lock_active(mc->ws)) {
			mif_info("voice call off acquire wakelock\n");
			cpif_wake_lock(mc->ws);
		}
	}

exit:
	mif_info("wakelock active = %d, voice status = %d\n",
		cpif_wake_lock_active(mc->ws), mc->pcie_voice_call_on);
	mutex_unlock(&mc->pcie_check_lock);
}
#endif

/* It means initial GPIO level. */
static int check_link_order = 1;
static irqreturn_t ap_wakeup_handler(int irq, void *data)
{
	struct modem_ctl *mc = (struct modem_ctl *)data;
	int gpio_val = mif_gpio_get_value(&mc->s5100_gpio_ap_wakeup, true);
	unsigned long flags;

	mif_disable_irq(&mc->s5100_irq_ap_wakeup);

	if (mc->device_reboot) {
		mif_err("skip : device is rebooting..!!!\n");
		return IRQ_HANDLED;
	}

	if (gpio_val == check_link_order)
		mif_err("cp2ap_wakeup val is the same with before : %d\n", gpio_val);
	check_link_order = gpio_val;

	spin_lock_irqsave(&mc->pcie_pm_lock, flags);
	if (mc->pcie_pm_suspended) {
		if (gpio_val == 1) {
			/* try to block system suspend */
			if (!cpif_wake_lock_active(mc->ws))
				cpif_wake_lock(mc->ws);
		}

		mif_err("cp2ap_wakeup work pending. gpio_val : %d\n", gpio_val);
		mc->pcie_pm_resume_wait = true;
		mc->pcie_pm_resume_gpio_val = gpio_val;

		spin_unlock_irqrestore(&mc->pcie_pm_lock, flags);
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&mc->pcie_pm_lock, flags);

	mc->apwake_irq_chip->irq_set_type(
		irq_get_irq_data(mc->s5100_irq_ap_wakeup.num),
		(gpio_val == 1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH));
	mif_enable_irq(&mc->s5100_irq_ap_wakeup);

	queue_work_on(RUNTIME_PM_AFFINITY_CORE, mc->wakeup_wq,
			(gpio_val == 1 ? &mc->wakeup_work : &mc->suspend_work));

	return IRQ_HANDLED;
}

static irqreturn_t cp_active_handler(int irq, void *data)
{
	struct modem_ctl *mc = (struct modem_ctl *)data;
	struct link_device *ld;
	struct mem_link_device *mld;
	int cp_active;
	enum modem_state old_state;
	enum modem_state new_state;

	if (mc == NULL) {
		mif_err_limited("modem_ctl is NOT initialized - IGNORING interrupt\n");
		goto irq_done;
	}

	ld = get_current_link(mc->iod);
	mld = to_mem_link_device(ld);

	if (mc->s51xx_pdev == NULL) {
		mif_err_limited("S5100 is NOT initialized - IGNORING interrupt\n");
		goto irq_done;
	}

	if (mc->phone_state != STATE_ONLINE) {
		mif_err_limited("Phone_state is NOT ONLINE - IGNORING interrupt\n");
		goto irq_done;
	}

	cp_active = mif_gpio_get_value(&mc->s5100_gpio_phone_active, true);
	mif_err("[PHONE_ACTIVE Handler] state:%s cp_active:%d\n",
			cp_state_str(mc->phone_state), cp_active);

	if (cp_active == 1) {
		mif_err("ERROR - cp_active is not low, state:%s cp_active:%d\n",
				cp_state_str(mc->phone_state), cp_active);
		return IRQ_HANDLED;
	}

	if (timer_pending(&mld->crash_ack_timer))
		del_timer(&mld->crash_ack_timer);

	mif_stop_logging();

	old_state = mc->phone_state;
	new_state = STATE_CRASH_EXIT;

	if (ld->crash_reason.type == CRASH_REASON_NONE)
		ld->crash_reason.type = CRASH_REASON_CP_ACT_CRASH;

	mif_info("Set s5100_cp_reset_required to false\n");
	mc->s5100_cp_reset_required = false;

	if (old_state != new_state) {
		mif_err("new_state = %s\n", cp_state_str(new_state));

		if (old_state == STATE_ONLINE)
			modem_notify_event(MODEM_EVENT_EXIT, mc);

		change_modem_state(mc, new_state);
	}

	atomic_set(&mld->forced_cp_crash, 0);

irq_done:
	mif_disable_irq(&mc->s5100_irq_phone_active);

	return IRQ_HANDLED;
}

static int register_phone_active_interrupt(struct modem_ctl *mc)
{
	int ret;

	if (mc == NULL)
		return -EINVAL;

	if (mc->s5100_irq_phone_active.registered == true)
		return 0;

	mif_info("Register PHONE ACTIVE interrupt.\n");
	mif_init_irq(&mc->s5100_irq_phone_active, mc->s5100_irq_phone_active.num,
			"phone_active", IRQF_TRIGGER_LOW);

	ret = mif_request_irq(&mc->s5100_irq_phone_active, cp_active_handler, mc);
	if (ret) {
		mif_err("%s: ERR! request_irq(%s#%d) fail (%d)\n",
			mc->name, mc->s5100_irq_phone_active.name,
			mc->s5100_irq_phone_active.num, ret);
		mif_err("xxx\n");
		return ret;
	}

	return ret;
}

static int register_cp2ap_wakeup_interrupt(struct modem_ctl *mc)
{
	int ret;

	if (mc == NULL)
		return -EINVAL;

	if (mc->s5100_irq_ap_wakeup.registered == true) {
		mif_info("Set IRQF_TRIGGER_LOW to cp2ap_wakeup gpio\n");
		check_link_order = 1;
		ret = mc->apwake_irq_chip->irq_set_type(
				irq_get_irq_data(mc->s5100_irq_ap_wakeup.num),
				IRQF_TRIGGER_LOW);
		return ret;
	}

	mif_info("Register CP2AP WAKEUP interrupt.\n");
	mif_init_irq(&mc->s5100_irq_ap_wakeup, mc->s5100_irq_ap_wakeup.num, "cp2ap_wakeup",
			IRQF_TRIGGER_LOW);

	ret = mif_request_irq(&mc->s5100_irq_ap_wakeup, ap_wakeup_handler, mc);
	if (ret) {
		mif_err("%s: ERR! request_irq(%s#%d) fail (%d)\n",
			mc->name, mc->s5100_irq_ap_wakeup.name,
			mc->s5100_irq_ap_wakeup.num, ret);
		mif_err("xxx\n");
		return ret;
	}

	return ret;
}

static int ds_detect = 2;
module_param(ds_detect, int, 0664);
MODULE_PARM_DESC(ds_detect, "Dual SIM detect");

static ssize_t ds_detect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", ds_detect);
}

static ssize_t ds_detect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	int value;

	ret = kstrtoint(buf, 0, &value);
	if (ret != 0) {
		mif_err("invalid value:%d with %d\n", value, ret);
		return -EINVAL;
	}

	ds_detect = value;
	mif_info("set ds_detect: %d\n", ds_detect);

	return count;
}
static DEVICE_ATTR_RW(ds_detect);

static struct attribute *sim_attrs[] = {
	&dev_attr_ds_detect.attr,
	NULL,
};

static const struct attribute_group sim_group = {
	.attrs = sim_attrs,
	.name = "sim",
};

static ssize_t s5100_wake_lock_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", cpif_wake_lock_active(mc->ws));
}

static ssize_t s5100_wake_lock_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	long op_num;

	if (kstrtol(buf, 0, &op_num))
		return -EINVAL;

	if (op_num)
		cpif_wake_lock(mc->ws);
	else
		cpif_wake_unlock(mc->ws);

	return count;
}

DEVICE_ATTR_RW(s5100_wake_lock);

static int get_ds_detect(void)
{
	if (ds_detect > 2 || ds_detect < 1)
		ds_detect = 2;

	mif_info("Dual SIM detect = %d\n", ds_detect);
	return ds_detect - 1;
}

static int init_control_messages(struct modem_ctl *mc)
{
	struct modem_data *modem = mc->mdm_data;
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	int ds_det;

	if (modem->offset_srinfo_offset)
		iowrite32(modem->srinfo_offset, mld->srinfo_offset);

	set_ctrl_msg(&mld->ap2cp_united_status, 0);
	set_ctrl_msg(&mld->cp2ap_united_status, 0);

	if (ld->capability_check) {
		if (modem->offset_capability_offset)
			iowrite32(modem->capability_offset, mld->capability_offset);

		iowrite32(0, mld->ap_capability_0_offset);
		iowrite32(0, mld->cp_capability_0_offset);
		iowrite32(0, mld->ap_capability_1_offset);
		iowrite32(0, mld->cp_capability_1_offset);
	}

	ds_det = get_ds_detect();
	if (ds_det < 0) {
		mif_err("ds_det error:%d\n", ds_det);
		return -EINVAL;
	}

	update_ctrl_msg(&mld->ap2cp_united_status, ds_det, mc->sbi_ds_det_mask,
			mc->sbi_ds_det_pos);
	mif_info("ds_det:%d\n", ds_det);

	return 0;
}

static int power_on_cp(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct modem_data __maybe_unused *modem = mc->mdm_data;
	struct mem_link_device *mld = to_mem_link_device(ld);

	mif_info("%s: +++\n", mc->name);

	mc->receive_first_ipc = 0;

	mif_disable_irq(&mc->s5100_irq_phone_active);
	mif_disable_irq(&mc->s5100_irq_ap_wakeup);
	drain_workqueue(mc->wakeup_wq);

	print_mc_state(mc);

	if (!cpif_wake_lock_active(mc->ws))
		cpif_wake_lock(mc->ws);

	mc->phone_state = STATE_OFFLINE;
	pcie_clean_dislink(mc);

	mc->pcie_registered = false;

	mif_gpio_set_value(&mc->s5100_gpio_ap_status, 1, 0);
	mif_gpio_set_value(&mc->s5100_gpio_cp_dump_noti, 0, 0);

	/* Clear shared memory */
	init_ctrl_msg(&mld->ap2cp_msg);
	init_ctrl_msg(&mld->cp2ap_msg);

	print_mc_state(mc);

	mif_gpio_set_value(&mc->s5100_gpio_cp_reset, 0, 50);
#if IS_ENABLED(CONFIG_CP_WRESET_WA)
	mif_gpio_set_value(&mc->s5100_gpio_cp_pwr, 0, 0);
	udelay(100);
#endif
	mif_gpio_set_value(&mc->s5100_gpio_cp_pwr, 1, 50);
	mif_gpio_set_value(&mc->s5100_gpio_cp_reset, 1, 50);

	mif_info("GPIO status after S5100 Power on\n");
	print_mc_state(mc);

	mif_info("---\n");

	return 0;
}

static int power_off_cp(struct modem_ctl *mc)
{
	mif_info("%s: +++\n", mc->name);

	if (mc->phone_state == STATE_OFFLINE)
		goto exit;

	change_modem_state(mc, STATE_OFFLINE);

	pcie_clean_dislink(mc);

	mif_gpio_set_value(&mc->s5100_gpio_cp_reset, 0, 0);
	mif_gpio_set_value(&mc->s5100_gpio_cp_pwr, 0, 0);

	print_mc_state(mc);

exit:
	mif_info("---\n");

	return 0;
}

static int power_shutdown_cp(struct modem_ctl *mc)
{
	int i;

	mif_err("%s: +++\n", mc->name);

	if (mc->phone_state == STATE_OFFLINE)
		goto exit;

	mif_disable_irq(&mc->s5100_irq_phone_active);
	mif_disable_irq(&mc->s5100_irq_ap_wakeup);
	drain_workqueue(mc->wakeup_wq);

	/* wait for cp_active for 3 seconds */
	for (i = 0; i < 150; i++) {
		if (mif_gpio_get_value(&mc->s5100_gpio_phone_active, false) == 1) {
			mif_err("PHONE_ACTIVE pin is HIGH...\n");
			break;
		}
		msleep(20);
	}

	mif_gpio_set_value(&mc->s5100_gpio_cp_reset, 0, 0);
	mif_gpio_set_value(&mc->s5100_gpio_cp_pwr, 0, 0);

	print_mc_state(mc);

	pcie_clean_dislink(mc);

exit:
	mif_err("---\n");
	return 0;
}

static int power_reset_dump_cp(struct modem_ctl *mc)
{
	struct s51xx_pcie *s51xx_pcie = NULL;
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);

	mif_info("%s: +++\n", mc->name);

	mc->receive_first_ipc = 0;

	if (ld->sbd_ipc && hrtimer_active(&mld->sbd_print_timer))
		hrtimer_cancel(&mld->sbd_print_timer);

	mc->phone_state = STATE_CRASH_EXIT;
	mif_disable_irq(&mc->s5100_irq_phone_active);
	mif_disable_irq(&mc->s5100_irq_ap_wakeup);
	drain_workqueue(mc->wakeup_wq);
	pcie_clean_dislink(mc);

	if (mc->s51xx_pdev != NULL)
		s51xx_pcie = pci_get_drvdata(mc->s51xx_pdev);

	if (s51xx_pcie && s51xx_pcie->link_status == 1) {
		mif_err("link_satus:%d\n", s51xx_pcie->link_status);
		s51xx_pcie_save_state(mc->s51xx_pdev);
		pcie_clean_dislink(mc);
	}

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_GPIO_WA)
	if (mif_gpio_set_value(&mc->s5100_gpio_cp_dump_noti, 1, 10))
		mif_gpio_toggle_value(&mc->s5100_gpio_ap_status, 50);
#else
	mif_gpio_set_value(&mc->s5100_gpio_cp_dump_noti, 1, 0);
#endif

	mif_info("s5100_cp_reset_required:%d\n", mc->s5100_cp_reset_required);
	if (mc->s5100_cp_reset_required == true) {
		mif_gpio_set_value(&mc->s5100_gpio_cp_reset, 0, 50);
#if IS_ENABLED(CONFIG_CP_WRESET_WA)
		mif_gpio_set_value(&mc->s5100_gpio_cp_pwr, 0, 0);
		udelay(100);
		mif_gpio_set_value(&mc->s5100_gpio_cp_pwr, 1, 50);
#endif
		mif_gpio_set_value(&mc->s5100_gpio_cp_reset, 1, 50);
		print_mc_state(mc);
	}

	mif_gpio_set_value(&mc->s5100_gpio_ap_status, 1, 0);

	mif_err("---\n");

	return 0;
}

static int power_reset_cp(struct modem_ctl *mc)
{
	struct s51xx_pcie *s51xx_pcie = NULL;
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);

	mif_info("%s: +++\n", mc->name);

	mc->receive_first_ipc = 0;

	if (ld->sbd_ipc && hrtimer_active(&mld->sbd_print_timer))
		hrtimer_cancel(&mld->sbd_print_timer);

	mc->phone_state = STATE_OFFLINE;
	pcie_clean_dislink(mc);

	if (mc->s51xx_pdev != NULL)
		s51xx_pcie = pci_get_drvdata(mc->s51xx_pdev);

	if (s51xx_pcie && s51xx_pcie->link_status == 1) {
		/* save_s5100_status(); */
		mif_err("link_satus:%d\n", s51xx_pcie->link_status);
		pcie_clean_dislink(mc);
	}

	mif_gpio_set_value(&mc->s5100_gpio_cp_reset, 0, 50);
#if IS_ENABLED(CONFIG_CP_WRESET_WA)
	mif_gpio_set_value(&mc->s5100_gpio_cp_pwr, 0, 0);
	udelay(100);
	mif_gpio_set_value(&mc->s5100_gpio_cp_pwr, 1, 50);
#endif
	mif_gpio_set_value(&mc->s5100_gpio_cp_reset, 1, 50);
	print_mc_state(mc);

	mif_err("---\n");

	return 0;
}

static int check_cp_status(struct modem_ctl *mc, unsigned int count)
{
	int ret = 0;
	int cnt = 0;
	int val;

	while (1) {
		val = mif_gpio_get_value(&mc->s5100_gpio_ap_wakeup, false);
		mif_err_limited("CP2AP_WAKEUP == %d (cnt %d)\n", val, cnt);

		if (val != 0) {
			ret = 0;
			break;
		}

		if (++cnt >= count) {
			mif_err("ERR! CP2AP_WAKEUP == 0 (cnt %d)\n", cnt);
			ret = -EFAULT;
			break;
		}

		msleep(20);
	}

	if (ret == 0)
		mif_info("CP2AP_WAKEUP == 1 cnt: %d\n", cnt);
	else
		mif_err("ERR: Checking count after sending bootloader: %d\n", cnt);

	if (cnt == 0)
		msleep(20);

	return ret;
}

static int start_normal_boot(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->bootd);
	struct mem_link_device *mld = to_mem_link_device(ld);
	int ret = 0;

	mif_info("+++\n");

	if (init_control_messages(mc))
		mif_err("Failed to initialize control messages\n");

	/* 2cp dump WA */
	if (timer_pending(&mld->crash_ack_timer))
		del_timer(&mld->crash_ack_timer);
	atomic_set(&mld->forced_cp_crash, 0);

	mif_info("Set link mode to LINK_MODE_BOOT.\n");

	if (ld->link_prepare_normal_boot)
		ld->link_prepare_normal_boot(ld, mc->bootd);

	change_modem_state(mc, STATE_BOOTING);

	mif_info("Disable phone actvie interrupt.\n");
	mif_disable_irq(&mc->s5100_irq_phone_active);

	mif_gpio_set_value(&mc->s5100_gpio_ap_status, 1, 0);
	mc->phone_state = STATE_BOOTING;

	if (ld->link_start_normal_boot) {
		mif_info("link_start_normal_boot\n");
		ld->link_start_normal_boot(ld, mc->iod);
	}

	ret = modem_ctrl_check_offset_data(mc);
	if (ret) {
		mif_err("modem_ctrl_check_offset_data() error:%d\n", ret);
		return ret;
	}

	ret = check_cp_status(mc, 200);
	if (ret < 0) {
		mif_err("ERR! check_cp_status fail (err %d)\n", ret);
		if (cpif_wake_lock_active(mc->ws))
			cpif_wake_unlock(mc->ws);
		return ret;
	}

	if (ld->register_pcie) {
		mif_info("register_pcie\n");
		ld->register_pcie(ld);
	}

	mif_info("---\n");
	return 0;
}

static int complete_normal_boot(struct modem_ctl *mc)
{
	int err = 0;
	unsigned long remain;
#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	int __maybe_unused ret;
	struct modem_data __maybe_unused *modem = mc->mdm_data;
	struct mem_link_device __maybe_unused *mld = modem->mld;
#endif

	mif_info("+++\n");

	reinit_completion(&mc->init_cmpl);
	remain = wait_for_completion_timeout(&mc->init_cmpl, MIF_INIT_TIMEOUT);
	if (remain == 0) {
		mif_err("T-I-M-E-O-U-T\n");
		err = -EAGAIN;
		goto exit;
	}

	/* Enable L1.2 after CP boot */
	s51xx_pcie_l1ss_ctrl(1, mc->pcie_ch_num);

	/* Read cp_active before enabling irq */
	mif_gpio_get_value(&mc->s5100_gpio_phone_active, true);

	err = register_phone_active_interrupt(mc);
	if (err)
		mif_err("Err: register_phone_active_interrupt:%d\n", err);
	mif_enable_irq(&mc->s5100_irq_phone_active);

	err = register_cp2ap_wakeup_interrupt(mc);
	if (err)
		mif_err("Err: register_cp2ap_wakeup_interrupt:%d\n", err);
	mif_enable_irq(&mc->s5100_irq_ap_wakeup);

	print_mc_state(mc);

	mc->device_reboot = false;

	change_modem_state(mc, STATE_ONLINE);

	print_mc_state(mc);

#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	if (mc->lcd_notifier.notifier_call == NULL) {
		mif_info("Register lcd notifier\n");
		mc->lcd_notifier.notifier_call = s5100_lcd_notifier;
		ret = register_lcd_status_notifier(&mc->lcd_notifier);
		if (ret) {
			mif_err("failed to register LCD notifier");
			return ret;
		}
	}
#endif /* CONFIG_CP_LCD_NOTIFIER */

	mif_info("---\n");

exit:
	return err;
}

static int trigger_cp_crash_internal(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->bootd);
	struct mem_link_device *mld = to_mem_link_device(ld);
	u32 crash_type;

	if (ld->crash_reason.type == CRASH_REASON_NONE)
		ld->crash_reason.type = CRASH_REASON_MIF_FORCED;
	crash_type = ld->crash_reason.type;

	mif_err("+++\n");

	if (mc->device_reboot) {
		mif_err("skip cp crash : device is rebooting..!!!\n");
		goto exit;
	}

	print_mc_state(mc);

	if (mif_gpio_get_value(&mc->s5100_gpio_phone_active, true) == 1) {
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_GPIO_WA)
		if (atomic_inc_return(&mc->dump_toggle_issued) > 1) {
			atomic_dec(&mc->dump_toggle_issued);
			goto exit;
		}

		if (mif_gpio_set_value(&mc->s5100_gpio_cp_dump_noti, 1, 10))
			mif_gpio_toggle_value(&mc->s5100_gpio_ap_status, 50);

		atomic_dec(&mc->dump_toggle_issued);
#else
		mif_gpio_set_value(&mc->s5100_gpio_cp_dump_noti, 1, 0);
#endif
	} else {
		mif_err("do not need to set dump_noti\n");
	}

	if (ld->protocol == PROTOCOL_SIT &&
			crash_type == CRASH_REASON_RIL_TRIGGER_CP_CRASH)
		ld->link_trigger_cp_crash(mld, crash_type, ld->crash_reason.string);
	else
		ld->link_trigger_cp_crash(mld, crash_type, "Forced crash is called");

exit:
	mif_err("---\n");
	return 0;
}

static void trigger_cp_crash_work(struct work_struct *ws)
{
	struct modem_ctl *mc = container_of(ws, struct modem_ctl, crash_work);

	trigger_cp_crash_internal(mc);
}

static int trigger_cp_crash(struct modem_ctl *mc)
{
	queue_work(mc->crash_wq, &mc->crash_work);
	return 0;
}

int s5100_force_crash_exit_ext(void)
{
	if (g_mc)
		g_mc->ops.trigger_cp_crash(g_mc);

	return 0;
}

int modem_force_crash_exit_ext(void)
{
	return s5100_force_crash_exit_ext();
}
EXPORT_SYMBOL(modem_force_crash_exit_ext);

int s5100_send_panic_noti_ext(void)
{
	struct modem_data *modem;

	if (g_mc) {
		modem = g_mc->mdm_data;
		if (modem->mld) {
			mif_err("Send CMD_KERNEL_PANIC message to CP\n");
			send_ipc_irq(modem->mld, cmd2int(CMD_KERNEL_PANIC));
		}
	}

	return 0;
}

static int start_dump_boot(struct modem_ctl *mc)
{
	int err;
	struct link_device *ld = get_current_link(mc->bootd);

	mif_err("+++\n");

	/* Change phone state to CRASH_EXIT */
	mc->phone_state = STATE_CRASH_EXIT;

	if (!ld->link_start_dump_boot) {
		mif_err("%s: link_start_dump_boot is null\n", ld->name);
		return -EFAULT;
	}
	err = ld->link_start_dump_boot(ld, mc->bootd);
	if (err)
		return err;

	mif_gpio_set_value(&mc->s5100_gpio_ap_status, 1, 0);

	err = check_cp_status(mc, 200);
	if (err < 0) {
		mif_err("ERR! check_cp_status fail (err %d)\n", err);
		return err;
	}

	/* do not handle cp2ap_wakeup irq during dump process */
	mif_disable_irq(&mc->s5100_irq_ap_wakeup);

	if (ld->register_pcie) {
		mif_info("register_pcie\n");
		ld->register_pcie(ld);
	}

	mif_err("---\n");
	return err;
}

int s5100_poweroff_pcie(struct modem_ctl *mc, bool force_off)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	bool force_crash = false;
	bool in_pcie_recovery = false;
	unsigned long flags;

	mutex_lock(&mc->pcie_onoff_lock);
	mutex_lock(&mc->pcie_check_lock);
	mif_debug("+++\n");

	if (!mc->pcie_powered_on &&
			(s51xx_check_pcie_link_status(mc->pcie_ch_num) == 0)) {
		mif_err("skip pci power off : already powered off\n");
		goto exit;
	}

	/* CP reads Tx RP (or tail) after CP2AP_WAKEUP = 1.
	 * skip pci power off if CP2AP_WAKEUP = 1 or Tx pending.
	 */
	if (!force_off) {
		spin_lock_irqsave(&mc->pcie_tx_lock, flags);
		/* wait Tx done if it is running */
		spin_unlock_irqrestore(&mc->pcie_tx_lock, flags);
		msleep(30);
		if (check_mem_link_tx_pending(mld) ||
			mif_gpio_get_value(&mc->s5100_gpio_ap_wakeup, true) == 1) {
			mif_err("skip pci power off : condition not met\n");
			goto exit;
		}
	}

	if (mld->msi_irq_base_enabled == 1) {
		disable_irq(mld->msi_irq_base);
		mld->msi_irq_base_enabled = 0;
	}

	if (mc->device_reboot) {
		mif_err("skip pci power off : device is rebooting..!!!\n");
		goto exit;
	}

	/* recovery status is not valid after PCI link down requests from CP */
	if (mc->pcie_cto_retry_cnt > 0) {
		mif_info("clear cto_retry_cnt(%d)..!!!\n", mc->pcie_cto_retry_cnt);
		mc->pcie_cto_retry_cnt = 0;
	}

	mc->pcie_powered_on = false;

	if (mc->s51xx_pdev != NULL && (mc->phone_state == STATE_ONLINE ||
				mc->phone_state == STATE_BOOTING)) {
		mif_debug("save s5100_status - phone_state:%d\n",
				mc->phone_state);
		s51xx_pcie_save_state(mc->s51xx_pdev);
	} else
		mif_debug("ignore save_s5100_status - phone_state:%d\n",
				mc->phone_state);

	mif_gpio_set_value(&mc->s5100_gpio_cp_wakeup, 0, 5);
	print_mc_state(mc);

	exynos_pcie_poweroff(mc->pcie_ch_num);

	if (cpif_wake_lock_active(mc->ws))
		cpif_wake_unlock(mc->ws);

exit:
	mif_debug("---\n");
	mutex_unlock(&mc->pcie_check_lock);
	mutex_unlock(&mc->pcie_onoff_lock);

	spin_lock_irqsave(&mc->pcie_tx_lock, flags);
	if (in_pcie_recovery && !mc->reserve_doorbell_int && check_mem_link_tx_pending(mld))
		mc->reserve_doorbell_int = true;

	if ((mc->s51xx_pdev != NULL) && !mc->device_reboot && mc->reserve_doorbell_int) {
		mif_debug("DBG: doorbell_reserved = %d\n", mc->reserve_doorbell_int);
		if (mc->pcie_powered_on) {
			mc->reserve_doorbell_int = false;
			if (s51xx_pcie_send_doorbell_int(mc->s51xx_pdev,
						mld->intval_ap2cp_msg) != 0)
				force_crash = true;
		} else
			s5100_try_gpio_cp_wakeup(mc);
	}
	spin_unlock_irqrestore(&mc->pcie_tx_lock, flags);

	if (unlikely(force_crash))
		s5100_force_crash_exit_ext();

	return 0;
}

int s5100_poweron_pcie(struct modem_ctl *mc)
{
	struct link_device *ld;
	struct mem_link_device *mld;
	bool force_crash = false;
	unsigned long flags;
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_S2MPU)
	int ret;
	u32 cp_num;
	u32 shmem_idx;
#endif

	if (mc == NULL) {
		mif_info("Skip pci power on : mc is NULL\n");
		return 0;
	}

	ld = get_current_link(mc->iod);
	mld = to_mem_link_device(ld);

	if (mc->phone_state == STATE_OFFLINE) {
		mif_info("Skip pci power on : phone_state is OFFLINE\n");
		return 0;
	}

	mutex_lock(&mc->pcie_onoff_lock);
	mutex_lock(&mc->pcie_check_lock);
	mif_debug("+++\n");
	if (mc->pcie_powered_on &&
			(s51xx_check_pcie_link_status(mc->pcie_ch_num) != 0)) {
		mif_err("skip pci power on : already powered on\n");
		goto exit;
	}

	if (mif_gpio_get_value(&mc->s5100_gpio_ap_wakeup, true) == 0) {
		mif_err("skip pci power on : condition not met\n");
		goto exit;
	}

	if (mc->device_reboot) {
		mif_err("skip pci power on : device is rebooting..!!!\n");
		goto exit;
	}

	if (!cpif_wake_lock_active(mc->ws))
		cpif_wake_lock(mc->ws);

	mif_gpio_set_value(&mc->s5100_gpio_cp_wakeup, 1, 5);
	print_mc_state(mc);

	spin_lock_irqsave(&mc->pcie_tx_lock, flags);
	/* wait Tx done if it is running */
	spin_unlock_irqrestore(&mc->pcie_tx_lock, flags);

	if (exynos_pcie_poweron(mc->pcie_ch_num) != 0)
		goto exit;

	mc->pcie_powered_on = true;

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_S2MPU)
	if (!mc->s5100_s2mpu_enabled) {
		mc->s5100_s2mpu_enabled = true;
		cp_num = ld->mdm_data->cp_num;

		for (shmem_idx = 0 ; shmem_idx < MAX_CP_SHMEM ; shmem_idx++) {
			if (shmem_idx == SHMEM_MSI)
				continue;

			if (cp_shmem_get_base(cp_num, shmem_idx)) {
				ret = (int) exynos_set_dev_stage2_ap("hsi2", 0,
					cp_shmem_get_base(cp_num, shmem_idx),
					cp_shmem_get_size(cp_num, shmem_idx), ATTR_RW);
				mif_info("pcie s2mpu idx:%d - addr:0x%08lx size:0x%08x ret:%d\n",
					shmem_idx,
					cp_shmem_get_base(cp_num, shmem_idx),
					cp_shmem_get_size(cp_num, shmem_idx), ret);
			}
		}
	}
#endif

	if (mc->s51xx_pdev != NULL) {
		s51xx_pcie_restore_state(mc->s51xx_pdev);

		/* DBG: check MSI sfr setting values */
		print_msi_register(mc->s51xx_pdev);
	} else {
		mif_err("DBG: MSI sfr not set up, yet(s5100_pdev is NULL)");
	}

	if (mld->msi_irq_base_enabled == 0) {
		enable_irq(mld->msi_irq_base);
		mld->msi_irq_base_enabled = 1;
	}

	if ((mc->s51xx_pdev != NULL) && mc->pcie_registered) {
		/* DBG */
		mif_info("DBG: doorbell: pcie_registered = %d\n", mc->pcie_registered);
		if (s51xx_pcie_send_doorbell_int(mc->s51xx_pdev, mc->int_pcie_link_ack) != 0) {
			/* DBG */
			mif_err("DBG: s5100pcie_send_doorbell_int() func. is failed !!!\n");
			s5100_force_crash_exit_ext();
		}
	}

#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
	if (mc->pcie_voice_call_on && (mc->phone_state != STATE_CRASH_EXIT)) {
		if (cpif_wake_lock_active(mc->ws))
			cpif_wake_unlock(mc->ws);

		mif_info("wakelock active = %d, voice status = %d\n",
			cpif_wake_lock_active(mc->ws), mc->pcie_voice_call_on);
	}
#endif

exit:
	mif_debug("---\n");
	mutex_unlock(&mc->pcie_check_lock);
	mutex_unlock(&mc->pcie_onoff_lock);

	spin_lock_irqsave(&mc->pcie_tx_lock, flags);
	if ((mc->s51xx_pdev != NULL) && mc->pcie_powered_on && mc->reserve_doorbell_int) {
		mif_info("DBG: doorbell: doorbell_reserved = %d\n", mc->reserve_doorbell_int);
		mc->reserve_doorbell_int = false;
		if (s51xx_pcie_send_doorbell_int(mc->s51xx_pdev, mld->intval_ap2cp_msg) != 0)
			force_crash = true;
	}
	spin_unlock_irqrestore(&mc->pcie_tx_lock, flags);

	if (unlikely(force_crash))
		s5100_force_crash_exit_ext();

	return 0;
}

int s5100_set_outbound_atu(struct modem_ctl *mc, struct cp_btl *btl, loff_t *pos, u32 map_size)
{
	int ret = 0;
	u32 atu_grp = (*pos) / map_size;

	if (atu_grp != btl->last_pcie_atu_grp) {
		ret = exynos_pcie_rc_set_outbound_atu(
			mc->pcie_ch_num, btl->mem.cp_p_base, (atu_grp * map_size), map_size);
		btl->last_pcie_atu_grp = atu_grp;
	}

	return ret;
}

static int suspend_cp(struct modem_ctl *mc)
{
	if (!mc)
		return 0;

	do {
#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
		if (mc->pcie_voice_call_on)
			break;
#endif

		if (mif_gpio_get_value(&mc->s5100_gpio_ap_wakeup, true) == 1) {
			mif_err("abort suspend");
			return -EBUSY;
		}
	} while (0);

#if !IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	modem_ctrl_set_kerneltime(mc);
	mif_gpio_set_value(&mc->s5100_gpio_ap_status, 0, 0);
	mif_gpio_get_value(&mc->s5100_gpio_ap_status, true);
#endif

	return 0;
}

static int resume_cp(struct modem_ctl *mc)
{
#if IS_ENABLED(CONFIG_GS_S2MPU)
	int ret;
#endif
	if (!mc)
		return 0;

#if IS_ENABLED(CONFIG_GS_S2MPU)

	if (!mc->s2mpu)
		return 0;

	ret =  s2mpu_restore(mc->s2mpu);
	if (ret) {
		mif_err("S2MPU restore failed error=%d\n", ret);
		return -EINVAL;
	}
#endif

#if !IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	modem_ctrl_set_kerneltime(mc);
	mif_gpio_set_value(&mc->s5100_gpio_ap_status, 1, 0);
	mif_gpio_get_value(&mc->s5100_gpio_ap_status, true);
#endif
	return 0;
}

static int s5100_pm_notifier(struct notifier_block *notifier,
				       unsigned long pm_event, void *v)
{
	struct modem_ctl *mc;
	unsigned long flags;
	int gpio_val;

	mc = container_of(notifier, struct modem_ctl, pm_notifier);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		mif_info("Suspend prepare\n");

		spin_lock_irqsave(&mc->pcie_pm_lock, flags);
		mc->pcie_pm_suspended = true;
		spin_unlock_irqrestore(&mc->pcie_pm_lock, flags);
		break;
	case PM_POST_SUSPEND:
		mif_info("Resume done\n");

		spin_lock_irqsave(&mc->pcie_pm_lock, flags);
		mc->pcie_pm_suspended = false;
		if (mc->pcie_pm_resume_wait) {
			mc->pcie_pm_resume_wait = false;
			gpio_val = mc->pcie_pm_resume_gpio_val;

			mif_err("cp2ap_wakeup work resume. gpio_val : %d\n", gpio_val);

			mc->apwake_irq_chip->irq_set_type(
				irq_get_irq_data(mc->s5100_irq_ap_wakeup.num),
				(gpio_val == 1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH));
			mif_enable_irq(&mc->s5100_irq_ap_wakeup);

			queue_work_on(RUNTIME_PM_AFFINITY_CORE, mc->wakeup_wq,
				(gpio_val == 1 ? &mc->wakeup_work : &mc->suspend_work));
		}
		spin_unlock_irqrestore(&mc->pcie_pm_lock, flags);
		break;
	default:
		mif_info("pm_event %lu\n", pm_event);
		break;
	}

	return NOTIFY_OK;
}

int s5100_try_gpio_cp_wakeup(struct modem_ctl *mc)
{
	if ((mif_gpio_get_value(&mc->s5100_gpio_cp_wakeup, false) == 0) &&
			(mif_gpio_get_value(&mc->s5100_gpio_ap_wakeup, false) == 0) &&
			(s51xx_check_pcie_link_status(mc->pcie_ch_num) == 0)) {
		mif_gpio_set_value(&mc->s5100_gpio_cp_wakeup, 1, 0);
		return 0;
	}
	return -EPERM;
}

static void s5100_get_ops(struct modem_ctl *mc)
{
	mc->ops.power_on = power_on_cp;
	mc->ops.power_off = power_off_cp;
	mc->ops.power_shutdown = power_shutdown_cp;
	mc->ops.power_reset = power_reset_cp;
	mc->ops.power_reset_dump = power_reset_dump_cp;

	mc->ops.start_normal_boot = start_normal_boot;
	mc->ops.complete_normal_boot = complete_normal_boot;

	mc->ops.start_dump_boot = start_dump_boot;
	mc->ops.trigger_cp_crash = trigger_cp_crash;

	mc->ops.suspend = suspend_cp;
	mc->ops.resume = resume_cp;
}

static void s5100_get_pdata(struct modem_ctl *mc, struct modem_data *pdata)
{
	struct platform_device *pdev = to_platform_device(mc->dev);
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	/* CP Power */
	mc->s5100_gpio_cp_pwr.num = of_get_named_gpio(np, "gpio_ap2cp_cp_pwr_on", 0);
	if (mc->s5100_gpio_cp_pwr.num < 0) {
		mif_err("Can't Get s5100_gpio_cp_pwr!\n");
		return;
	}
	mc->s5100_gpio_cp_pwr.label = "AP2CP_CP_PWR_ON";
	gpio_request(mc->s5100_gpio_cp_pwr.num, mc->s5100_gpio_cp_pwr.label);
	gpio_direction_output(mc->s5100_gpio_cp_pwr.num, 0);

	/* CP Reset */
	mc->s5100_gpio_cp_reset.num = of_get_named_gpio(np, "gpio_ap2cp_nreset_n", 0);
	if (mc->s5100_gpio_cp_reset.num < 0) {
		mif_err("Can't Get gpio_cp_nreset_n!\n");
		return;
	}
	mc->s5100_gpio_cp_reset.label = "AP2CP_NRESET_N";
	gpio_request(mc->s5100_gpio_cp_reset.num, mc->s5100_gpio_cp_reset.label);
	gpio_direction_output(mc->s5100_gpio_cp_reset.num, 0);

	/* CP PS HOLD */
	mc->s5100_gpio_cp_ps_hold.num = of_get_named_gpio(np, "gpio_cp2ap_cp_ps_hold", 0);
	if (mc->s5100_gpio_cp_ps_hold.num < 0) {
		mif_err("Can't Get s5100_gpio_cp_ps_hold!\n");
		return;
	}
	mc->s5100_gpio_cp_ps_hold.label = "CP2AP_CP_PS_HOLD";
	gpio_request(mc->s5100_gpio_cp_ps_hold.num, mc->s5100_gpio_cp_ps_hold.label);
	gpio_direction_input(mc->s5100_gpio_cp_ps_hold.num);

	/* AP2CP WAKE UP */
	mc->s5100_gpio_cp_wakeup.num = of_get_named_gpio(np, "gpio_ap2cp_wake_up", 0);
	if (mc->s5100_gpio_cp_wakeup.num < 0) {
		mif_err("Can't Get s5100_gpio_cp_wakeup!\n");
		return;
	}
	mc->s5100_gpio_cp_wakeup.label = "AP2CP_WAKE_UP";
	gpio_request(mc->s5100_gpio_cp_wakeup.num, mc->s5100_gpio_cp_wakeup.label);
	gpio_direction_output(mc->s5100_gpio_cp_wakeup.num, 0);

	/* CP2AP WAKE UP */
	mc->s5100_gpio_ap_wakeup.num = of_get_named_gpio(np, "gpio_cp2ap_wake_up", 0);
	if (mc->s5100_gpio_ap_wakeup.num < 0) {
		mif_err("Can't Get gpio_cp2ap_wake_up!\n");
		return;
	}
	mc->s5100_gpio_ap_wakeup.label = "CP2AP_WAKE_UP";
	gpio_request(mc->s5100_gpio_ap_wakeup.num, mc->s5100_gpio_ap_wakeup.label);
	mc->s5100_irq_ap_wakeup.num = gpio_to_irq(mc->s5100_gpio_ap_wakeup.num);

	/* DUMP NOTI */
	mc->s5100_gpio_cp_dump_noti.num = of_get_named_gpio(np, "gpio_ap2cp_dump_noti", 0);
	if (mc->s5100_gpio_cp_dump_noti.num < 0) {
		mif_err("Can't Get gpio_ap2cp_dump_noti!\n");
		return;
	}
	mc->s5100_gpio_cp_dump_noti.label = "AP2CP_DUMP_NOTI";
	gpio_request(mc->s5100_gpio_cp_dump_noti.num, mc->s5100_gpio_cp_dump_noti.label);
	gpio_direction_output(mc->s5100_gpio_cp_dump_noti.num, 0);

	/* PDA ACTIVE */
	mc->s5100_gpio_ap_status.num = of_get_named_gpio(np, "gpio_ap2cp_pda_active", 0);
	if (mc->s5100_gpio_ap_status.num < 0) {
		mif_err("Can't Get s5100_gpio_ap_status!\n");
		return;
	}
	mc->s5100_gpio_ap_status.label = "AP2CP_PDA_ACTIVE";
	gpio_request(mc->s5100_gpio_ap_status.num, mc->s5100_gpio_ap_status.label);
	gpio_direction_output(mc->s5100_gpio_ap_status.num, 0);

	/* PHONE ACTIVE */
	mc->s5100_gpio_phone_active.num = of_get_named_gpio(np, "gpio_cp2ap_phone_active", 0);
	if (mc->s5100_gpio_phone_active.num < 0) {
		mif_err("Can't Get s5100_gpio_phone_active!\n");
		return;
	}
	mc->s5100_gpio_phone_active.label = "CP2AP_PHONE_ACTIVE";
	gpio_request(mc->s5100_gpio_phone_active.num, mc->s5100_gpio_phone_active.label);
	mc->s5100_irq_phone_active.num = gpio_to_irq(mc->s5100_gpio_phone_active.num);
	mc->s5100_irq_phone_active.not_alive = pdata->cp2ap_active_not_alive;

	ret = of_property_read_u32(np, "mif,int_ap2cp_pcie_link_ack",
				&mc->int_pcie_link_ack);
	if (ret) {
		mif_err("Can't Get PCIe Link ACK interrupt number!!!\n");
		return;
	}
	mc->int_pcie_link_ack += DOORBELL_INT_ADD;

	/* Get PCIe Channel Number */
	ret = of_property_read_u32(np, "pci_ch_num",
				&mc->pcie_ch_num);
	if (ret) {
		mif_err("Can't Get PCIe channel!!!\n");
		return;
	}
	mif_info("S5100 PCIe Channel Number : %d\n", mc->pcie_ch_num);

	mc->sbi_crash_type_mask = pdata->sbi_crash_type_mask;
	mc->sbi_crash_type_pos = pdata->sbi_crash_type_pos;

	mc->sbi_ds_det_mask = pdata->sbi_ds_det_mask;
	mc->sbi_ds_det_pos = pdata->sbi_ds_det_pos;
}

static int send_panic_to_cp_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	int ret = 0;

	ret = s5100_send_panic_noti_ext();

	return NOTIFY_DONE;
}

#if IS_ENABLED(CONFIG_EXYNOS_BUSMONITOR)
static int s5100_busmon_notifier(struct notifier_block *nb,
						unsigned long event, void *data)
{
	struct busmon_notifier *info = (struct busmon_notifier *)data;
	char *init_desc = info->init_desc;

	if (init_desc != NULL &&
		(strncmp(init_desc, "CP", strlen(init_desc)) == 0 ||
		strncmp(init_desc, "APB_CORE_CP", strlen(init_desc)) == 0 ||
		strncmp(init_desc, "MIF_CP", strlen(init_desc)) == 0)) {
		struct modem_ctl *mc =
			container_of(nb, struct modem_ctl, busmon_nfb);

		mc->ops.trigger_cp_crash(mc);
	}
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
static int s5100_call_state_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct modem_ctl *mc = container_of(nb, struct modem_ctl, call_state_nb);

	mif_info("call event = %lu\n", action);
	switch (action) {
	case MODEM_VOICE_CALL_OFF:
		mc->pcie_voice_call_on = false;
		queue_work_on(RUNTIME_PM_AFFINITY_CORE, mc->wakeup_wq,
			&mc->call_off_work);
		break;
	case MODEM_VOICE_CALL_ON:
		mc->pcie_voice_call_on = true;
		queue_work_on(RUNTIME_PM_AFFINITY_CORE, mc->wakeup_wq,
			&mc->call_on_work);
		break;
	default:
		mif_err("undefined call event = %lu\n", action);
		break;
	}

	return NOTIFY_DONE;
}
#endif

#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
static int s5100_lcd_notifier(struct notifier_block *notifier,
		unsigned long event, void *v)
{
	struct modem_ctl *mc =
		container_of(notifier, struct modem_ctl, lcd_notifier);

	switch (event) {
	case LCD_OFF:
		mif_info("LCD_OFF Notification\n");
		modem_ctrl_set_kerneltime(mc);
		mif_gpio_set_value(&mc->s5100_gpio_ap_status, 0, 0);
		mif_gpio_get_value(&mc->s5100_gpio_ap_status, true);
		break;

	case LCD_ON:
		mif_info("LCD_ON Notification\n");
		modem_ctrl_set_kerneltime(mc);
		mif_gpio_set_value(&mc->s5100_gpio_ap_status, 1, 0);
		mif_gpio_get_value(&mc->s5100_gpio_ap_status, true);
		break;

	default:
		mif_info("lcd_event %ld\n", event);
		break;
	}

	return NOTIFY_OK;
}
#endif /* CONFIG_CP_LCD_NOTIFIER */

int s5100_init_modemctl_device(struct modem_ctl *mc, struct modem_data *pdata)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(mc->dev);
	struct resource __maybe_unused *sysram_alive;

	g_mc = mc;

	s5100_get_ops(mc);
	s5100_get_pdata(mc, pdata);
	dev_set_drvdata(mc->dev, mc);

	mc->ws = cpif_wake_lock_register(&pdev->dev, "s5100_wake_lock");
	if (mc->ws == NULL) {
		mif_err("s5100_wake_lock: wakeup_source_register fail\n");
		return -EINVAL;
	}
	mutex_init(&mc->pcie_onoff_lock);
	mutex_init(&mc->pcie_check_lock);
	spin_lock_init(&mc->pcie_tx_lock);
	spin_lock_init(&mc->pcie_pm_lock);
	spin_lock_init(&mc->power_stats_lock);
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_GPIO_WA)
	atomic_set(&mc->dump_toggle_issued, 0);
#endif

	mif_gpio_set_value(&mc->s5100_gpio_cp_reset, 0, 0);

	mif_err("Register GPIO interrupts\n");
	mc->apwake_irq_chip = irq_get_chip(mc->s5100_irq_ap_wakeup.num);
	if (mc->apwake_irq_chip == NULL) {
		mif_err("Can't get irq_chip structure!!!!\n");
		return -EINVAL;
	}

	mc->wakeup_wq = create_singlethread_workqueue("cp2ap_wakeup_wq");
	if (!mc->wakeup_wq) {
		mif_err("%s: ERR! fail to create wakeup_wq\n", mc->name);
		return -EINVAL;
	}
	INIT_WORK(&mc->wakeup_work, cp2ap_wakeup_work);
	INIT_WORK(&mc->suspend_work, cp2ap_suspend_work);

	mc->crash_wq = create_singlethread_workqueue("trigger_cp_crash_wq");
	if (!mc->crash_wq) {
		mif_err("%s: ERR! fail to create crash_wq\n", mc->name);
		return -EINVAL;
	}
	INIT_WORK(&mc->crash_work, trigger_cp_crash_work);

	mc->reboot_nb.notifier_call = s5100_reboot_handler;
	register_reboot_notifier(&mc->reboot_nb);

	/* Register PM notifier_call */
	mc->pm_notifier.notifier_call = s5100_pm_notifier;
	ret = register_pm_notifier(&mc->pm_notifier);
	if (ret) {
		mif_err("failed to register PM notifier_call\n");
		return ret;
	}

	/* Register panic notifier_call*/
	mc->send_panic_nb.notifier_call = send_panic_to_cp_notifier;
	atomic_notifier_chain_register(&panic_notifier_list, &mc->send_panic_nb);

#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
	INIT_WORK(&mc->call_on_work, voice_call_on_work);
	INIT_WORK(&mc->call_off_work, voice_call_off_work);

	mc->call_state_nb.notifier_call = s5100_call_state_notifier;
	register_modem_voice_call_event_notifier(&mc->call_state_nb);
#endif

	if (sysfs_create_group(&pdev->dev.kobj, &sim_group))
		mif_err("failed to create sysfs node related sim\n");

	if (sysfs_create_group(&pdev->dev.kobj, &modem_group))
		mif_err("failed to create sysfs node related modem\n");

	ret = device_create_file(&pdev->dev, &dev_attr_s5100_wake_lock);
	if (ret) {
		mif_err("%s: couldn't create s5100_wake_lock(%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}
