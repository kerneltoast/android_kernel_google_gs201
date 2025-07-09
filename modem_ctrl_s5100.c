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
#include <linux/clk.h>
#include <linux/pci.h>
#include <linux/regulator/consumer.h>
#include <soc/google/acpm_mfd.h>
#include <soc/google/modem_notifier.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/panic_notifier.h>
#if IS_ENABLED(CONFIG_S5910)
#include <linux/s5910.h>
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

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOMMU)
#include "link_device_pcie_iommu.h"
#endif
#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
#include "../../../video/fbdev/exynos/dpu30/decon.h"
static int s5100_lcd_notifier(struct notifier_block *notifier,
		unsigned long event, void *v);
#endif /* CONFIG_CP_LCD_NOTIFIER */

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

#define RUNTIME_PM_AFFINITY_CORE 2

static struct modem_ctl *g_mc;

static int s5100_poweroff_pcie(struct modem_ctl *mc, bool force_off);

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
	int pwr = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_CP_PWR], false);
	int reset = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_NRESET], false);
	int pshold = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_PS_HOLD], false);

	int ap_wakeup = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP], false);
	int cp_wakeup = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_WAKEUP], false);

	int dump = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI], false);
	int ap_status = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], false);
	int phone_active = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_ACTIVE], false);
#if !IS_ENABLED(CONFIG_CP_WRESET_WA)
	int wrst = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_WRST_N], false);

	logbuffer_log(mc->log,
		"%s: %ps:GPIO pwr:%d rst:%d phd:%d c2aw:%d a2cw:%d dmp:%d ap_act:%d cp_act:%d wrst:%d",
		mc->name, CALLER, pwr, reset, pshold, ap_wakeup, cp_wakeup, dump,
		ap_status, phone_active, wrst);
#else
	logbuffer_log(mc->log,
		"%s: %ps:GPIO pwr:%d rst:%d phd:%d c2aw:%d a2cw:%d dmp:%d ap_act:%d cp_act:%d",
		mc->name, CALLER, pwr, reset, pshold, ap_wakeup, cp_wakeup, dump,
		ap_status, phone_active);
#endif
}

static void pcie_clean_dislink(struct modem_ctl *mc)
{
#if IS_ENABLED(CONFIG_CPIF_AP_SUSPEND_DURING_VOICE_CALL)
	if (mc->pcie_voice_call_on) {
		modem_notify_event(MODEM_EVENT_RESET, mc);
		mc->pcie_voice_call_on = false;
	} else {
		modem_notify_event(MODEM_EVENT_OFFLINE, mc);
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
	if (mc->cp_power_stats.suspended) {
		mc->cp_power_stats.last_exit_timestamp_usec = ktime_to_us(cp2ap_wakeup_time);
		mc->cp_power_stats.duration_usec += (mc->cp_power_stats.last_exit_timestamp_usec -
				mc->cp_power_stats.last_entry_timestamp_usec);
	}
	mc->cp_power_stats.suspended = false;
	spin_unlock_irqrestore(&mc->power_stats_lock, flags);

	s5100_poweron_pcie(mc, false);
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
	if (!mc->cp_power_stats.suspended) {
		mc->cp_power_stats.last_entry_timestamp_usec = ktime_to_us(cp2ap_suspend_time);
		mc->cp_power_stats.count++;
	}
	mc->cp_power_stats.suspended = true;
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
	if (mc->cp_power_stats.suspended) {
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

#define MAX_PCIE_EVENT_HISTORY 10
static int pcie_linkdown_history[MAX_PCIE_EVENT_HISTORY];
static int pcie_linkdown_count;
static int pcie_cto_history[MAX_PCIE_EVENT_HISTORY];
static int pcie_cto_count;
static ssize_t pcie_event_stats_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	ssize_t count = 0, i = 0;

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"Total linkdown retries: %d\n",
		mc->pcie_linkdown_retry_cnt_all);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"Total CPL timeout retries: %d\n",
		mc->pcie_cto_retry_cnt_all);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"Previous linkdown retries: %d\n",
		mc->pcie_linkdown_retry_cnt);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"Previous CPL timeout retries: %d\n",
		mc->pcie_cto_retry_cnt);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"Previous %d linkdown retries:",
		MAX_PCIE_EVENT_HISTORY);
	for (i = 0; i < MAX_PCIE_EVENT_HISTORY; i++) {
		count += scnprintf(&buf[count], PAGE_SIZE - count, " %d",
			pcie_linkdown_history[i]);
	}
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"\nTotal linkdown retries recorded: %d\n",
		pcie_linkdown_count);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"Previous %d CPL timeout retries:", MAX_PCIE_EVENT_HISTORY);
	for (i = 0; i < MAX_PCIE_EVENT_HISTORY; i++) {
		count += scnprintf(&buf[count], PAGE_SIZE - count, " %d",
			pcie_cto_history[i]);
	}
	count += scnprintf(&buf[count], PAGE_SIZE - count,
			"\nTotal CPL timeout retries recorded: %d\n",
			pcie_cto_count);

	return count;
}

static DEVICE_ATTR_RO(power_stats);
static DEVICE_ATTR_RO(pcie_event_stats);

static struct attribute *modem_attrs[] = {
	&dev_attr_power_stats.attr,
	&dev_attr_pcie_event_stats.attr,
	NULL,
};

static const struct attribute_group modem_group = {
	.attrs = modem_attrs,
	.name = "modem",
};

#if IS_ENABLED(CONFIG_CPIF_AP_SUSPEND_DURING_VOICE_CALL)
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
	struct modem_ctl *mc = data;
	int gpio_val = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP], true);
	unsigned long flags;

	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP]);

#if !IS_ENABLED(CONFIG_CP_WRESET_WA)
	if (mc->mdm_data->mif_off_during_volte) {
		int wrst_gpio_val = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_WRST_N], true);
		/* To avoid holding on to the wakesource in case of a race condition */
		if (wrst_gpio_val || !mc->pcie_voice_call_on)
			cpif_wake_unlock(mc->ws_wrst);
	}
#endif

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
		irq_get_irq_data(mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP].num),
		(gpio_val == 1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH));
	mif_enable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP]);

	queue_work_on(RUNTIME_PM_AFFINITY_CORE, mc->wakeup_wq,
			(gpio_val == 1 ? &mc->wakeup_work : &mc->suspend_work));

	return IRQ_HANDLED;
}

static irqreturn_t cp_active_handler(int irq, void *data)
{
	struct modem_ctl *mc = data;
	struct link_device *ld;
	struct mem_link_device *mld;
	int cp_active;
	enum modem_state old_state;
	enum modem_state new_state;
	struct legacy_link_device *bl;
	struct legacy_ipc_device *ipc_dev;
	int i;

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

	cp_active = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_ACTIVE], true);
	mif_err("[PHONE_ACTIVE Handler] state:%s cp_active:%d\n",
			cp_state_str(mc->phone_state), cp_active);

	if (cp_active == 1)
		mif_err("ERROR - cp_active is not low, state:%s cp_active:%d\n",
				cp_state_str(mc->phone_state), cp_active);

	if (timer_pending(&mld->crash_ack_timer))
		del_timer(&mld->crash_ack_timer);

	mif_stop_logging();

	old_state = mc->phone_state;
	new_state = STATE_CRASH_EXIT;

	if (ld->crash_reason.type == CRASH_REASON_NONE)
		ld->crash_reason.type = CRASH_REASON_CP_ACT_CRASH;

	mc->s5100_cp_reset_required = false;
	mif_info("Set s5100_cp_reset_required to 0\n");

	if (old_state != new_state) {
		mif_err("new_state = %s\n", cp_state_str(new_state));

		if (old_state == STATE_ONLINE)
			modem_notify_event(MODEM_EVENT_EXIT, mc);

		change_modem_state(mc, new_state);
	}

	bl = &mld->legacy_link_dev;

	for (i = 0; i < IPC_MAP_MAX; i++) {
		ipc_dev = bl->dev[i];
		mif_info("%s TX: head:%d tail:%d, RX: head: %d tail:%d\n",
			ipc_dev->name, get_txq_head(ipc_dev), get_txq_tail(ipc_dev),
			get_rxq_head(ipc_dev), get_rxq_tail(ipc_dev));
	}

	atomic_set(&mld->forced_cp_crash, 0);

irq_done:
	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE]);

	return IRQ_HANDLED;
}

#if !IS_ENABLED(CONFIG_CP_WRESET_WA)
static irqreturn_t cp_wrst_handler(int irq, void *data)
{
	struct modem_ctl *mc = data;
	int gpio_val = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_WRST_N], true);

	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N]);

	if (gpio_val == 0) {
		logbuffer_log(mc->log, "acquire wrst lock\n");
		cpif_wake_lock(mc->ws_wrst);
	} else {
		logbuffer_log(mc->log, "release wrst lock\n");
		cpif_wake_unlock(mc->ws_wrst);
	}

	mc->cp_wrst_irq_chip->irq_set_type(
		irq_get_irq_data(mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N].num),
		(gpio_val == 1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH));
	mif_enable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N]);

	return IRQ_HANDLED;
}

static int register_cp_wrst_interrupt(struct modem_ctl *mc)
{
	int ret;

	if (mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N].registered)
		return 0;

	mif_info("Register CP_WRST interrupt.\n");
	mif_init_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N],
		     mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N].num,
		     "cp_wrst", IRQF_TRIGGER_LOW);

	ret = mif_request_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N],
			cp_wrst_handler, mc);
	if (ret)
		mif_err("%s: ERR! request_irq(%s#%d) fail (%d)\n",
			mc->name, mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N].name,
			mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N].num, ret);

	return ret;
}
#endif

static int register_phone_active_interrupt(struct modem_ctl *mc)
{
	int ret;

	if (mc == NULL)
		return -EINVAL;

	if (mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE].registered)
		return 0;

	mif_info("Register PHONE ACTIVE interrupt.\n");
	mif_init_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE],
		     mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE].num,
		     "phone_active", IRQF_TRIGGER_LOW);

	ret = mif_request_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE], cp_active_handler, mc);
	if (ret) {
		mif_err("%s: ERR! request_irq(%s#%d) fail (%d)\n",
			mc->name, mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE].name,
			mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE].num, ret);
		return ret;
	}

	return ret;
}

static int register_cp2ap_wakeup_interrupt(struct modem_ctl *mc)
{
	int ret;

	if (mc == NULL)
		return -EINVAL;

	if (mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP].registered) {
		mif_info("Set IRQF_TRIGGER_LOW to cp2ap_wakeup gpio\n");
		check_link_order = 1;
		ret = mc->apwake_irq_chip->irq_set_type(
				irq_get_irq_data(mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP].num),
				IRQF_TRIGGER_LOW);
		return ret;
	}

	mif_info("Register CP2AP WAKEUP interrupt.\n");
	mif_init_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP],
		     mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP].num,
		     "cp2ap_wakeup", IRQF_TRIGGER_LOW);

	ret = mif_request_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP], ap_wakeup_handler, mc);
	if (ret) {
		mif_err("%s: ERR! request_irq(%s#%d) fail (%d)\n",
			mc->name, mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP].name,
			mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP].num, ret);
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
	return sysfs_emit(buf, "%d\n", ds_detect);
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

	return sysfs_emit(buf, "%d\n", cpif_wake_lock_active(mc->ws));
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

static ssize_t s5100_wrst_wake_lock_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", cpif_wake_lock_active(mc->ws_wrst));
}

static ssize_t s5100_wrst_wake_lock_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	long op_num;

	if (kstrtol(buf, 0, &op_num))
		return -EINVAL;

	if (op_num)
		cpif_wake_lock(mc->ws_wrst);
	else
		cpif_wake_unlock(mc->ws_wrst);

	return count;
}

DEVICE_ATTR_RW(s5100_wake_lock);
DEVICE_ATTR_RW(s5100_wrst_wake_lock);

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

	if (modem->offset_cmsg_offset)
		iowrite32(modem->cmsg_offset, mld->cmsg_offset);
	if (modem->offset_srinfo_offset)
		iowrite32(modem->srinfo_offset, mld->srinfo_offset);
	if (ld->capability_check && modem->offset_capability_offset)
		iowrite32(modem->capability_offset, mld->capability_offset);

	set_ctrl_msg(&mld->ap2cp_united_status, 0);
	set_ctrl_msg(&mld->cp2ap_united_status, 0);

	if (ld->capability_check) {
		int part;

		for (part = 0; part < AP_CP_CAP_PARTS; part++) {
			iowrite32(0, mld->ap_capability_offset[part]);
			iowrite32(0, mld->cp_capability_offset[part]);
		}
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

static void set_pcie_msi_int(struct link_device *ld, bool enabled)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	int irq;
	bool *irq_wake;
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct pktproc_adaptor *ppa = &mld->pktproc;
	unsigned int q_idx = 0;
#endif

	if (!mld->msi_irq_base)
		return;

	irq = mld->msi_irq_base;
	irq_wake = &mld->msi_irq_base_wake;

	do {
		if (enabled) {
			int err;

			if (!mld->msi_irq_enabled)
				enable_irq(irq);

			if (!*irq_wake) {
				err = enable_irq_wake(irq);
				*irq_wake = !err;
			}
		} else {
			if (mld->msi_irq_enabled)
				disable_irq(irq);

			if (*irq_wake) {
				disable_irq_wake(irq);
				*irq_wake = false;
			}
		}

		irq = 0;
#if IS_ENABLED(CONFIG_CP_PKTPROC)
		if (q_idx < ppa->num_queue) {
			struct pktproc_queue *q = ppa->q[q_idx];

			irq = q->irq;
			irq_wake = &q->msi_irq_wake;
			q_idx++;
		}
#endif
	} while (irq);

	mld->msi_irq_enabled = enabled;
}

static int request_pcie_int(struct link_device *ld, struct platform_device *pdev)
{
#define DOORBELL_INT_MASK(x)	((x) | 0x10000)

	static struct lock_class_key lock_class, request_class;
	int ret, base_irq;
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct device *dev = &pdev->dev;
	struct modem_ctl *mc = ld->mc;
	struct modem_data *modem = mc->mdm_data;
	int irq_offset = 0;

	/* Doorbell */
	mld->intval_ap2cp_msg = DOORBELL_INT_MASK(modem->mbx->int_ap2cp_msg);
	mld->intval_ap2cp_pcie_link_ack = DOORBELL_INT_MASK(modem->mbx->int_ap2cp_pcie_link_ack);

	/* MSI */
	base_irq = s51xx_pcie_request_msi_int(mc->s51xx_pdev, 4);
	if (base_irq <= 0) {
		mif_err("Can't get MSI IRQ!!!\n");
		return -EFAULT;
	}
	mif_info("MSI base_irq(%d)\n", base_irq);

	/*
	 * This lock class tells lockdep that base_irqs are in a different
	 * category than their parents, so it won't report false recursion.
	 */
	irq_set_lockdep_class(base_irq, &lock_class, &request_class);

	ret = devm_request_irq(dev, base_irq + irq_offset, shmem_irq_handler,
			       IRQF_SHARED, "mif_cp2ap_msg", mld);
	if (ret) {
		mif_err("Can't request cp2ap_msg interrupt!!!\n");
		return -EIO;
	}
	irq_offset++;

	ret = devm_request_irq(dev, base_irq + irq_offset, shmem_tx_state_handler,
			       IRQF_SHARED, "mif_cp2ap_status", mld);
	if (ret) {
		mif_err("Can't request cp2ap_status interrupt!!!\n");
		return -EIO;
	}
	irq_offset++;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (mld->pktproc.use_exclusive_irq) {
		struct pktproc_adaptor *ppa = &mld->pktproc;
		unsigned int i;

		for (i = 0; i < ppa->num_queue; i++) {
			struct pktproc_queue *q = ppa->q[i];

			ret = register_separated_msi_vector(mc->pcie_ch_num, q->irq_handler, q,
							    &q->irq);
			if (ret < 0) {
				mif_err("register_separated_msi_vector for pktproc q[%u] err:%d\n",
					i, ret);
				q->irq = 0;
				return -EIO;
			}
		}
	}
#endif

	mld->msi_irq_base = base_irq;
	mld->msi_irq_enabled = true;
	set_pcie_msi_int(ld, true);

	return base_irq;
}

static int register_pcie(struct link_device *ld)
{
	struct modem_ctl *mc = ld->mc;
	struct platform_device *pdev = to_platform_device(mc->dev);
	static int is_registered;
	struct mem_link_device *mld = to_mem_link_device(ld);
	u32 cp_num = ld->mdm_data->cp_num;

#if IS_ENABLED(CONFIG_GS_S2MPU)
	u32 shmem_idx;
	int ret;
	struct device_node *s2mpu_dn;
#endif
	mif_info("CP EP driver initialization start.\n");

	if (!mld->msi_reg_base && (mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE))
		mld->msi_reg_base = cp_shmem_get_region(cp_num, SHMEM_MSI);

#if IS_ENABLED(CONFIG_GS_S2MPU)

	s2mpu_dn = of_parse_phandle(mc->dev->of_node, "s2mpu", 0);
	if (!s2mpu_dn) {
		mif_err("Failed to find s2mpu from device tree\n");
		return -EINVAL;
	}

	mc->s2mpu = s2mpu_fwnode_to_info(&s2mpu_dn->fwnode);
	if (!mc->s2mpu) {
		mif_err("Failed to get S2MPU\n");
		return -EPROBE_DEFER;
	}

	for (shmem_idx = 0 ; shmem_idx < MAX_CP_SHMEM ; shmem_idx++) {
		if (shmem_idx == SHMEM_MSI && !(mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE))
			continue;

		if (cp_shmem_get_base(cp_num, shmem_idx)) {
			ret =  s2mpu_open(mc->s2mpu,
					  cp_shmem_get_base(cp_num, shmem_idx),
					  cp_shmem_get_size(cp_num, shmem_idx),
					  DMA_BIDIRECTIONAL);
			if (ret) {
				mif_err("S2MPU open failed error=%d\n", ret);
				return -EINVAL;
			}
		}
	}

	/* Also setup AoC window for voice calls */
	ret =  s2mpu_open(mc->s2mpu,
			  AOC_PCIE_WINDOW_START, AOC_PCIE_WINDOW_SIZE,
			  DMA_BIDIRECTIONAL);

	if (ret) {
		mif_err("S2MPU AoC open failed error=%d\n", ret);
		return -EINVAL;
	}

#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOMMU)
	cpif_pcie_iommu_enable_regions(mld);
#endif

	msleep(200);

	s5100_poweron_pcie(mc, !!(mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE));

	if (is_registered == 0) {
		/* initialize the pci_dev for modem_ctl */
		mif_info("s51xx_pcie_init start\n");
		s51xx_pcie_init(mc);
		if (!mc->s51xx_pdev) {
			mif_err("s51xx_pdev is NULL. Check if CP wake up is done.\n");
			return -EINVAL;
		}

		/* debug: check MSI 32bit or 64bit - should be set as 32bit before this point*/
		// debug: pci_read_config_dword(s51xx_pcie.s51xx_pdev, 0x50, &msi_val);
		// debug: mif_err("MSI Control Reg : 0x%x\n", msi_val);

		request_pcie_int(ld, pdev);
		first_save_s51xx_status(mc->s51xx_pdev);

		is_registered = 1;
	} else {
		if (mc->phone_state == STATE_CRASH_RESET) {
			print_msi_register(mc->s51xx_pdev);
			enable_irq(mld->msi_irq_base);
		}
	}

	print_msi_register(mc->s51xx_pdev);
	mc->pcie_registered = true;

	mif_info("CP EP driver initialization end.\n");

	return 0;
}

static void gpio_power_off_cp(struct modem_ctl *mc)
{
#if IS_ENABLED(CONFIG_CP_WRESET_WA)
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_NRESET], 0, 50);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_CP_PWR], 0, 0);
#else
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_WAKEUP], 1, 10);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_WAKEUP], 0, 0);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_NRESET], 0, 0);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_CP_WRST_N], 0, 0);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_CP_PWR], 0, 30);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_PM_WRST_N], 0, 50);
#endif
}

static void gpio_power_offon_cp(struct modem_ctl *mc)
{
	gpio_power_off_cp(mc);

#if IS_ENABLED(CONFIG_CP_WRESET_WA)
	udelay(50);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_CP_PWR], 1, 50);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_NRESET], 1, 50);
#else
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_PM_WRST_N], 1, 10);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_CP_PWR], 1, 10);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_NRESET], 1, 10);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_CP_WRST_N], 1, 0);
#endif
}

static void gpio_power_wreset_cp(struct modem_ctl *mc)
{
#if !IS_ENABLED(CONFIG_CP_WRESET_WA)
	int i = 0, val;

	mif_info("warm reset sequence start\n");
	val = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_WRST_N],
		false);
	if (!val)
		mif_err("cp2ap_cp_wrst level is low before warm reset\n");

	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_PM_WRST_N], 1, 5);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_CP_WRST_N], 0, 0);
	if (!val)
		udelay(1000);

	while (i++ < 20) {
		if (!mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_WRST_N],
				false))
			break;
		mif_info("Wait for cp2ap_cp_wrst pulled to low\n");
		usleep_range(1000, 1100);
	}

	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_PM_WRST_N], 0, 5);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_CP_WRST_N], 1, 45);

	mif_info("warm reset sequence end\n");
#endif
}

static void clear_boot_stage(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	int val;

	if (mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE) {
		if (!mld->msi_reg_base) {
			u32 cp_num = ld->mdm_data->cp_num;
			mld->msi_reg_base = cp_shmem_get_region(cp_num, SHMEM_MSI);
			if (!mld->msi_reg_base) {
				mif_err("Failed to get valid msi reg base.\n");
				return;
			}
		}

		iowrite32(0, mld->msi_reg_base +
			offsetof(struct msi_reg_type, boot_stage));
		val = ioread32(mld->msi_reg_base +
			offsetof(struct msi_reg_type, boot_stage));
		mif_info("Clear boot_stage == 0x%X\n", val);
	}
}

static int power_on_cp(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct modem_data __maybe_unused *modem = mc->mdm_data;
	struct mem_link_device *mld = to_mem_link_device(ld);

	mif_info("%s: +++\n", mc->name);

	clear_boot_stage(mc);

	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE]);
	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP]);

	drain_workqueue(mc->wakeup_wq);

	print_mc_state(mc);

	if (!cpif_wake_lock_active(mc->ws))
		cpif_wake_lock(mc->ws);

	if (mc->phone_state != STATE_OFFLINE) {
		change_modem_state(mc, STATE_RESET);
		msleep(STATE_RESET_INTERVAL_MS);
	}
	change_modem_state(mc, STATE_OFFLINE);

	pcie_clean_dislink(mc);

	mc->pcie_registered = false;

	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 1, 0);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI], 0, 0);

	/* Clear shared memory */
	init_ctrl_msg(&mld->ap2cp_msg);
	init_ctrl_msg(&mld->cp2ap_msg);

	print_mc_state(mc);
	gpio_power_offon_cp(mc);
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

	gpio_power_off_cp(mc);
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

	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE]);
	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP]);

	drain_workqueue(mc->wakeup_wq);

	/* wait for cp_active for 3 seconds */
	for (i = 0; i < 150; i++) {
		if (mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_ACTIVE], false) == 1) {
			mif_err("PHONE_ACTIVE pin is HIGH...\n");
			break;
		}
		msleep(20);
	}

#if IS_ENABLED(CONFIG_S5910)
	if (mc->s5910_dev) {
		mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_NRESET], 0, 50);
		s5910_shutdown_sequence(mc->s5910_dev);
	}
#endif

	gpio_power_off_cp(mc);
	print_mc_state(mc);

	pcie_clean_dislink(mc);

exit:
	mif_err("---\n");
	return 0;
}

static int power_reset_dump_cp(struct modem_ctl *mc, bool silent)
{
	struct s51xx_pcie *s51xx_pcie = NULL;
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);

	mif_info("%s: +++\n", mc->name);

	clear_boot_stage(mc);

	if (ld->sbd_ipc && hrtimer_active(&mld->sbd_print_timer))
		hrtimer_cancel(&mld->sbd_print_timer);

	mc->phone_state = STATE_CRASH_EXIT;
	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE]);
	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP]);

	drain_workqueue(mc->wakeup_wq);
	pcie_clean_dislink(mc);

	if (mc->s51xx_pdev != NULL)
		s51xx_pcie = pci_get_drvdata(mc->s51xx_pdev);

	if (s51xx_pcie && s51xx_pcie->link_status == 1) {
		mif_info("link_satus:%d\n", s51xx_pcie->link_status);
		s51xx_pcie_save_state(mc->s51xx_pdev);
		pcie_clean_dislink(mc);
	}

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_GPIO_WA)
	if (mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI], 1, 10))
		mif_gpio_toggle_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 50);
#else
	if (silent)
		mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI], 0, 0);
	else
		mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI], 1, 0);
#endif

	mif_info("s5100_cp_reset_required:%d\n", mc->s5100_cp_reset_required);
	if (mc->s5100_cp_reset_required)
		gpio_power_offon_cp(mc);
	else
		gpio_power_wreset_cp(mc);

	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 1, 0);
	print_mc_state(mc);

	mif_info("---\n");

	return 0;
}

static int power_reset_cp(struct modem_ctl *mc)
{
	struct s51xx_pcie *s51xx_pcie = NULL;
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);

	mif_info("%s: +++\n", mc->name);

	if (ld->sbd_ipc && hrtimer_active(&mld->sbd_print_timer))
		hrtimer_cancel(&mld->sbd_print_timer);

	mc->phone_state = STATE_OFFLINE;
	pcie_clean_dislink(mc);

	if (mc->s51xx_pdev != NULL)
		s51xx_pcie = pci_get_drvdata(mc->s51xx_pdev);

	if (s51xx_pcie && s51xx_pcie->link_status == 1) {
		/* save_s5100_status(); */
		mif_info("link_satus:%d\n", s51xx_pcie->link_status);
		pcie_clean_dislink(mc);
	}

	gpio_power_offon_cp(mc);
	print_mc_state(mc);

	mif_info("---\n");

	return 0;
}

static int silent_reset_cp(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	int ret = 0;

	mif_info("%s: +++\n", mc->name);

	if (!cpif_wake_lock_active(mc->ws))
		cpif_wake_lock(mc->ws);

	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI], 0, 0);

	/* Clear shared memory */
	init_ctrl_msg(&mld->ap2cp_msg);
	init_ctrl_msg(&mld->cp2ap_msg);

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

	mif_info("---\n");

	return 0;
}

static int check_cp_status(struct modem_ctl *mc, unsigned int count, bool check_msi)
{
#define STATUS_NAME(msi) (msi ? "boot_stage" : "CP2AP_WAKEUP")

	struct link_device *ld = get_current_link(mc->bootd);
	struct mem_link_device *mld = to_mem_link_device(ld);
	bool check_done = false;
	int cnt = 0;
	int val;

	do {
		if (check_msi) {
			val = (int)ioread32(mld->msi_reg_base +
				offsetof(struct msi_reg_type, boot_stage));
			if (val == BOOT_STAGE_DONE_MASK) {
				check_done = true;
				break;
			}
		} else {
			val = mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP], false);
			if (val == 1) {
				check_done = true;
				break;
			}
		}

		mif_info_limited("%s == 0x%X (cnt %d)\n", STATUS_NAME(check_msi), val, cnt);
		msleep(20);
	} while (++cnt < count);

	if (!check_done) {
		mif_err("ERR! %s == 0x%X (cnt %d)\n", STATUS_NAME(check_msi), val, cnt);
		return -EFAULT;
	}

	mif_info("%s == 0x%X (cnt %d)\n", STATUS_NAME(check_msi), val, cnt);
	if (cnt == 0)
		msleep(20);

	return 0;
}

static int set_cp_rom_boot_img(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	struct modem_data *modem = mc->mdm_data;
	unsigned long boot_img_addr;

	if (!(mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE)) {
		mif_err("Invalid attr:0x%lx\n", mld->attrs);
		return -EPERM;
	}

	if (!mld->msi_reg_base) {
		mif_err("MSI region is not assigned yet\n");
		return -EINVAL;
	}

	boot_img_addr = cp_shmem_get_base(modem->cp_num, SHMEM_IPC) + mld->boot_img_offset;

	iowrite32(PADDR_LO(boot_img_addr),
		  mld->msi_reg_base + offsetof(struct msi_reg_type, img_addr_lo));
	iowrite32(PADDR_HI(boot_img_addr),
		  mld->msi_reg_base + offsetof(struct msi_reg_type, img_addr_hi));
	iowrite32(mld->boot_img_size,
		  mld->msi_reg_base + offsetof(struct msi_reg_type, img_size));

	mif_info("boot_img addr:0x%lX size:0x%X\n", boot_img_addr, mld->boot_img_size);

	s51xx_pcie_send_doorbell_int(mc->s51xx_pdev, mld->intval_ap2cp_msg);

	return 0;
}

static void debug_cp_rom_boot_img(struct mem_link_device *mld)
{
	unsigned char str[64 * 3];
	u8 __iomem *img_base;
	u32 img_size;

	if (!(mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE)) {
		mif_err("Invalid attr:0x%lx\n", mld->attrs);
		return;
	}

	img_base = mld->base + mld->boot_img_offset;
	img_size = ioread32(mld->msi_reg_base + offsetof(struct msi_reg_type, img_size));

	mif_err("boot_stage:0x%X err_report:0x%X img_lo:0x%X img_hi:0x%X img_size:0x%X\n",
		ioread32(mld->msi_reg_base + offsetof(struct msi_reg_type, boot_stage)),
		ioread32(mld->msi_reg_base + offsetof(struct msi_reg_type, err_report)),
		ioread32(mld->msi_reg_base + offsetof(struct msi_reg_type, img_addr_lo)),
		ioread32(mld->msi_reg_base + offsetof(struct msi_reg_type, img_addr_hi)),
		img_size);

	if (img_size > 64)
		img_size = 64;

	dump2hex(str, (img_size ? img_size * 3 : 1), img_base, img_size);
	mif_err("img_content:%s\n", str);
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
	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE]);

	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 1, 0);
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

	if (mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE) {
		register_pcie(ld);
		if (mc->s51xx_pdev && mc->pcie_registered)
			set_cp_rom_boot_img(mld);

		ret = check_cp_status(mc, 200, true);
		if (ret < 0)
			goto status_error;

		s5100_poweroff_pcie(mc, false);

		ret = check_cp_status(mc, 200, false);
		if (ret < 0)
			goto status_error;

		s5100_poweron_pcie(mc, false);
	} else {
		ret = check_cp_status(mc, 200, false);
		if (ret < 0)
			goto status_error;

		register_pcie(ld);
	}

status_error:
	if (ret < 0) {
		mif_err("ERR! check_cp_status fail (err %d)\n", ret);
		if (mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE)
			debug_cp_rom_boot_img(mld);
		if (cpif_wake_lock_active(mc->ws))
			cpif_wake_unlock(mc->ws);

		return ret;
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
	mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_ACTIVE], true);

	err = register_phone_active_interrupt(mc);
	if (err)
		mif_err("Err: register_phone_active_interrupt:%d\n", err);
	mif_enable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_ACTIVE]);

	err = register_cp2ap_wakeup_interrupt(mc);
	if (err)
		mif_err("Err: register_cp2ap_wakeup_interrupt:%d\n", err);
	mif_enable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP]);

#if !IS_ENABLED(CONFIG_CP_WRESET_WA)
	if (mc->mdm_data->mif_off_during_volte) {
		register_cp_wrst_interrupt(mc);
		mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N]);
	}
#endif

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
			mif_err("failed to register LCD notifier\n");
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
	char reason[CP_CRASH_INFO_SIZE] = "Forced crash call";

	if (ld->crash_reason.type == CRASH_REASON_NONE)
		ld->crash_reason.type = CRASH_REASON_MIF_FORCED;
	crash_type = ld->crash_reason.type;

	if (strlen(ld->crash_reason.string) > 0) {
		scnprintf(reason, CP_CRASH_INFO_SIZE, "Forced crash call by %s",
				ld->crash_reason.string);
	}


	mif_err("+++\n");

	if (mc->device_reboot) {
		mif_err("skip cp crash : device is rebooting..!!!\n");
		goto exit;
	}

	print_mc_state(mc);
	exynos_pcie_rc_print_msi_register(mc->pcie_ch_num);

	if (mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_CP_ACTIVE], true) == 1) {
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_GPIO_WA)
		if (atomic_inc_return(&mc->dump_toggle_issued) > 1) {
			atomic_dec(&mc->dump_toggle_issued);
			goto exit;
		}

		if (mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI], 1, 10))
			mif_gpio_toggle_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 50);

		atomic_dec(&mc->dump_toggle_issued);
#else
		mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI], 1, 0);
#endif

	} else {
		mif_err("do not need to set dump_noti\n");
	}

	ld->link_trigger_cp_crash(mld, crash_type, reason);

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

int s5100_force_crash_exit_ext(enum crash_type type)
{
	struct link_device *ld = get_current_link(g_mc->bootd);

	ld->crash_reason.type = type;

	if (g_mc)
		g_mc->ops.trigger_cp_crash(g_mc);

	return 0;
}

static int s5100_force_crash_exit_ext_reason(const char *buf)
{
	struct link_device *ld = get_current_link(g_mc->bootd);

	if (strlen(buf) > 0)
		strncpy(ld->crash_reason.string, buf, CP_CRASH_INFO_SIZE);

	return s5100_force_crash_exit_ext(CRASH_REASON_MIF_FORCED);
}

static int s5100_force_crash_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	const char *buf = nb_data;

	return s5100_force_crash_exit_ext_reason(buf);
}

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
	struct link_device *ld = get_current_link(mc->bootd);
	struct mem_link_device *mld = to_mem_link_device(ld);
	int err = 0;

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

	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 1, 0);
	/* do not handle cp2ap_wakeup irq during dump process */
	mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP]);

	if (mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE) {
		register_pcie(ld);
		if (mc->s51xx_pdev && mc->pcie_registered)
			set_cp_rom_boot_img(mld);

		err = check_cp_status(mc, 200, true);
		if (err < 0)
			goto status_error;

		s5100_poweroff_pcie(mc, false);

		err = check_cp_status(mc, 200, false);
		if (err < 0)
			goto status_error;

		s5100_poweron_pcie(mc, false);
	} else {
		err = check_cp_status(mc, 200, false);
		if (err < 0)
			goto status_error;

		register_pcie(ld);
	}

status_error:
	if (err < 0) {
		mif_err("ERR! check_cp_status fail (err %d)\n", err);
		if (mld->attrs & LINK_ATTR_XMIT_BTDLR_PCIE)
			debug_cp_rom_boot_img(mld);
		return err;
	}

	mif_err("---\n");
	return err;
}

static int s5100_poweroff_pcie(struct modem_ctl *mc, bool force_off)
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
		mif_info("Skip pci power off: already powered off\n");
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
			mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP], true) == 1) {
			mif_info("Skip pci power off: condition not met\n");
			goto exit;
		}
	}

	set_pcie_msi_int(ld, false);

	if (mc->device_reboot) {
		mif_info("Skip pci power off: device is rebooting\n");
		goto exit;
	}

	/* recovery status is not valid after PCI link down requests from CP */
	if (mc->pcie_linkdown_retry_cnt > 0) {
		mif_info("clear linkdown_retry_cnt(%d)..!!!\n", mc->pcie_linkdown_retry_cnt);
		pcie_linkdown_history[pcie_linkdown_count++ \
			% MAX_PCIE_EVENT_HISTORY] = mc->pcie_linkdown_retry_cnt;
		mc->pcie_linkdown_retry_cnt = 0;
	}

	if (mc->pcie_cto_retry_cnt > 0) {
		mif_info("clear cto_retry_cnt(%d)..!!!\n", mc->pcie_cto_retry_cnt);
		pcie_cto_history[pcie_cto_count++ % MAX_PCIE_EVENT_HISTORY] \
			= mc->pcie_cto_retry_cnt;
		mc->pcie_cto_retry_cnt = 0;
	}

	if (exynos_pcie_rc_get_sudden_linkdown_state(mc->pcie_ch_num)) {
		exynos_pcie_rc_set_sudden_linkdown_state(mc->pcie_ch_num, false);
		in_pcie_recovery = true;
	}

	if (exynos_pcie_rc_get_cpl_timeout_state(mc->pcie_ch_num)) {
		exynos_pcie_rc_set_cpl_timeout_state(mc->pcie_ch_num, false);
		in_pcie_recovery = true;
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

	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_WAKEUP], 0, 5);
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
		s5100_force_crash_exit_ext(CRASH_REASON_PCIE_DOORBELL_FAILURE_POWEROFF);

	return 0;
}

int s5100_poweron_pcie(struct modem_ctl *mc, bool boot_on)
{
	struct link_device *ld;
	struct mem_link_device *mld;
	bool force_crash = false;
	unsigned long flags;

	if (mc == NULL) {
		mif_err("Skip pci power on: mc is NULL\n");
		return 0;
	}

	ld = get_current_link(mc->iod);
	mld = to_mem_link_device(ld);

	if (mc->phone_state == STATE_OFFLINE) {
		mif_info("Skip pci power on: phone_state is OFFLINE\n");
		return 0;
	}

	mutex_lock(&mc->pcie_onoff_lock);
	mutex_lock(&mc->pcie_check_lock);
	mif_debug("+++\n");
	if (mc->pcie_powered_on &&
			(s51xx_check_pcie_link_status(mc->pcie_ch_num) != 0)) {
		mif_info("Skip pci power on: already powered on\n");
		goto exit;
	}

	if (!boot_on &&
	    mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP], true) == 0) {
		mif_info("Skip pci power on: condition not met\n");
		goto exit;
	}

	if (mc->device_reboot) {
		mif_info("Skip pci power on: device is rebooting\n");
		goto exit;
	}

	if (!cpif_wake_lock_active(mc->ws))
		cpif_wake_lock(mc->ws);

	if (!boot_on)
		mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_WAKEUP], 1, 5);

	print_mc_state(mc);

	spin_lock_irqsave(&mc->pcie_tx_lock, flags);
	/* wait Tx done if it is running */
	spin_unlock_irqrestore(&mc->pcie_tx_lock, flags);

	if (exynos_pcie_rc_get_sudden_linkdown_state(mc->pcie_ch_num))
		exynos_pcie_set_ready_cto_recovery(mc->pcie_ch_num);

	if (exynos_pcie_rc_get_cpl_timeout_state(mc->pcie_ch_num))
		exynos_pcie_set_ready_cto_recovery(mc->pcie_ch_num);

	exynos_pcie_set_msi_ctrl_addr(mc->pcie_ch_num, shm_get_msi_base());
	if (exynos_pcie_poweron(mc->pcie_ch_num, (boot_on ? 1 : 3), (boot_on ? 1 : 2)) != 0)
		goto exit;

	mc->pcie_powered_on = true;

	if (mc->s51xx_pdev != NULL) {
		s51xx_pcie_restore_state(mc->s51xx_pdev, boot_on);

		/* DBG: check MSI sfr setting values */
		print_msi_register(mc->s51xx_pdev);
	} else {
		mif_err("DBG: MSI sfr not set up, yet(s5100_pdev is NULL)");
	}

	set_pcie_msi_int(ld, true);

	if ((mc->s51xx_pdev != NULL) && mc->pcie_registered && (mc->phone_state != STATE_CRASH_EXIT)) {
		/* DBG */
		logbuffer_log(mc->log, "DBG: doorbell: pcie_registered = %d", \
				mc->pcie_registered);
		if (s51xx_pcie_send_doorbell_int(mc->s51xx_pdev,
						 mld->intval_ap2cp_pcie_link_ack) != 0) {
			/* DBG */
			mif_err("DBG: s5100pcie_send_doorbell_int() func. is failed !!!\n");
			s5100_force_crash_exit_ext(CRASH_REASON_PCIE_DOORBELL_FAILURE_POWERON);
		}
	}

#if IS_ENABLED(CONFIG_CPIF_AP_SUSPEND_DURING_VOICE_CALL)
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
		logbuffer_log(mc->log, "DBG: doorbell: doorbell_reserved = %d", \
				mc->reserve_doorbell_int);
		mc->reserve_doorbell_int = false;
		if (s51xx_pcie_send_doorbell_int(mc->s51xx_pdev, mld->intval_ap2cp_msg) != 0)
			force_crash = true;
	}
	spin_unlock_irqrestore(&mc->pcie_tx_lock, flags);

	if (unlikely(force_crash))
		s5100_force_crash_exit_ext(CRASH_REASON_PCIE_DOORBELL_FAILURE_POWERON);

	return 0;
}

void s5100_set_pcie_irq_affinity(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct pktproc_adaptor *ppa = &mld->pktproc;
	unsigned int num_queue = 1;
	unsigned int i;

	if (ppa->use_exclusive_irq)
		num_queue = ppa->num_queue;

	for (i = 0; i < num_queue; i++) {
		if (!ppa->q[i]->irq)
			break;

		irq_set_affinity_hint(ppa->q[i]->irq, cpumask_of(mld->msi_irq_q_cpu[i]));
	}
#endif

	if (mld->msi_irq_base)
		irq_set_affinity_hint(mld->msi_irq_base, cpumask_of(mld->msi_irq_base_cpu));
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
#if IS_ENABLED(CONFIG_CPIF_AP_SUSPEND_DURING_VOICE_CALL)
		if (mc->pcie_voice_call_on)
			break;
#endif

		if (mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP], true) == 1) {
			mif_err("abort suspend\n");
			return -EBUSY;
		}
	} while (0);

#if !IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	modem_ctrl_set_kerneltime(mc);
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 0, 0);
	mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], true);
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

	s5100_set_pcie_irq_affinity(mc);

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
	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 1, 0);
	mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], true);
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
				irq_get_irq_data(mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP].num),
				(gpio_val == 1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH));
			mif_enable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP]);

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
	if ((mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_WAKEUP], false) == 0) &&
	    (mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP], false) == 0) &&
	    (s51xx_check_pcie_link_status(mc->pcie_ch_num) == 0)) {
		mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_WAKEUP], 1, 0);
		print_mc_state(mc);
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
	mc->ops.silent_reset = silent_reset_cp;

	mc->ops.start_normal_boot = start_normal_boot;
	mc->ops.complete_normal_boot = complete_normal_boot;

	mc->ops.start_dump_boot = start_dump_boot;
	mc->ops.trigger_cp_crash = trigger_cp_crash;

	mc->ops.suspend = suspend_cp;
	mc->ops.resume = resume_cp;
}

static int s5100_get_pdata(struct modem_ctl *mc, struct modem_data *pdata)
{
	struct platform_device *pdev = to_platform_device(mc->dev);
	struct device_node *np = pdev->dev.of_node;
	unsigned int i;

	/* label */
	mc->cp_gpio[CP_GPIO_AP2CP_CP_PWR].label = "AP2CP_CP_PWR";
	mc->cp_gpio[CP_GPIO_AP2CP_NRESET].label = "AP2CP_NRESET";
	mc->cp_gpio[CP_GPIO_AP2CP_WAKEUP].label = "AP2CP_WAKEUP";
	mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI].label = "AP2CP_DUMP_NOTI";
	mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE].label = "AP2CP_AP_ACTIVE";
#if !IS_ENABLED(CONFIG_CP_WRESET_WA)
	mc->cp_gpio[CP_GPIO_AP2CP_CP_WRST_N].label = "AP2CP_CP_WRST_N";
	mc->cp_gpio[CP_GPIO_CP2AP_CP_WRST_N].label = "CP2AP_CP_WRST_N";
	mc->cp_gpio[CP_GPIO_AP2CP_PM_WRST_N].label = "AP2CP_PM_WRST_N";
#endif
	mc->cp_gpio[CP_GPIO_CP2AP_PS_HOLD].label = "CP2AP_PS_HOLD";
	mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP].label = "CP2AP_WAKEUP";
	mc->cp_gpio[CP_GPIO_CP2AP_CP_ACTIVE].label = "CP2AP_CP_ACTIVE";

	/* node name */
	mc->cp_gpio[CP_GPIO_AP2CP_CP_PWR].node_name = "gpio_ap2cp_cp_pwr_on";
	mc->cp_gpio[CP_GPIO_AP2CP_NRESET].node_name = "gpio_ap2cp_nreset_n";
	mc->cp_gpio[CP_GPIO_AP2CP_WAKEUP].node_name = "gpio_ap2cp_wake_up";
	mc->cp_gpio[CP_GPIO_AP2CP_DUMP_NOTI].node_name = "gpio_ap2cp_dump_noti";
	mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE].node_name = "gpio_ap2cp_pda_active";
#if !IS_ENABLED(CONFIG_CP_WRESET_WA)
	mc->cp_gpio[CP_GPIO_AP2CP_CP_WRST_N].node_name = "gpio_ap2cp_cp_wrst_n";
	mc->cp_gpio[CP_GPIO_CP2AP_CP_WRST_N].node_name = "gpio_cp2ap_cp_wrst_n";
	mc->cp_gpio[CP_GPIO_AP2CP_PM_WRST_N].node_name = "gpio_ap2cp_pm_wrst_n";
#endif
	mc->cp_gpio[CP_GPIO_CP2AP_PS_HOLD].node_name = "gpio_cp2ap_cp_ps_hold";
	mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP].node_name = "gpio_cp2ap_wake_up";
	mc->cp_gpio[CP_GPIO_CP2AP_CP_ACTIVE].node_name = "gpio_cp2ap_phone_active";

	/* irq */
	mc->cp_gpio[CP_GPIO_CP2AP_WAKEUP].irq_type = CP_GPIO_IRQ_CP2AP_WAKEUP;
	mc->cp_gpio[CP_GPIO_CP2AP_CP_ACTIVE].irq_type = CP_GPIO_IRQ_CP2AP_CP_ACTIVE;
#if !IS_ENABLED(CONFIG_CP_WRESET_WA)
	if (pdata->mif_off_during_volte)
		mc->cp_gpio[CP_GPIO_CP2AP_CP_WRST_N].irq_type = CP_GPIO_IRQ_CP2AP_CP_WRST_N;
#endif

	/* gpio */
	for (i = 0; i < CP_GPIO_MAX; i++) {
		mc->cp_gpio[i].num =
			of_get_named_gpio(np, mc->cp_gpio[i].node_name, 0);

		if (!gpio_is_valid(mc->cp_gpio[i].num))
			continue;

		mc->cp_gpio[i].valid = true;

		gpio_request(mc->cp_gpio[i].num, mc->cp_gpio[i].label);
		if (!strncmp(mc->cp_gpio[i].label, "AP2CP", 5))
			gpio_direction_output(mc->cp_gpio[i].num, 0);
		else
			gpio_direction_input(mc->cp_gpio[i].num);

		if (mc->cp_gpio[i].irq_type != CP_GPIO_IRQ_NONE) {
			mc->cp_gpio_irq[mc->cp_gpio[i].irq_type].num =
				gpio_to_irq(mc->cp_gpio[i].num);

			if (i == CP_GPIO_CP2AP_CP_ACTIVE) {
				mc->cp_gpio_irq[mc->cp_gpio[i].irq_type].not_alive =
					pdata->cp2ap_active_not_alive;
			}
		}
	}

	/* validate */
	for (i = 0; i < CP_GPIO_MAX; i++) {
		if (!mc->cp_gpio[i].valid) {
			mif_err("Missing some of GPIOs\n");
			return -EINVAL;
		}
	}

	/* Get PCIe Channel Number */
	mif_dt_read_u32(np, "pci_ch_num", mc->pcie_ch_num);
	mif_info("PCIe Channel Number:%d\n", mc->pcie_ch_num);

	mc->sbi_crash_type_mask = pdata->sbi_crash_type_mask;
	mc->sbi_crash_type_pos = pdata->sbi_crash_type_pos;

	mc->sbi_ds_det_mask = pdata->sbi_ds_det_mask;
	mc->sbi_ds_det_pos = pdata->sbi_ds_det_pos;

	return 0;
}

static int send_panic_to_cp_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	s5100_send_panic_noti_ext();
	return NOTIFY_DONE;
}

#if IS_ENABLED(CONFIG_CPIF_AP_SUSPEND_DURING_VOICE_CALL)
static int s5100_call_state_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct modem_ctl *mc = container_of(nb, struct modem_ctl, call_state_nb);

	mif_info("call event = %lu\n", action);
	switch (action) {
	case MODEM_VOICE_CALL_OFF:
		mc->pcie_voice_call_on = false;
		if (mc->mdm_data->mif_off_during_volte) {
			mif_disable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N]);
			synchronize_irq(mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N].num);
			cpif_wake_unlock(mc->ws_wrst);
			logbuffer_log(mc->log, "released wrst wakelock after voice call");
		}
		queue_work_on(RUNTIME_PM_AFFINITY_CORE, mc->wakeup_wq,
			&mc->call_off_work);
		break;
	case MODEM_VOICE_CALL_ON:
		mc->pcie_voice_call_on = true;
		if (mc->mdm_data->mif_off_during_volte)
			mif_enable_irq(&mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N]);
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
		mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 0, 0);
		mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], true);
		break;

	case LCD_ON:
		mif_info("LCD_ON Notification\n");
		modem_ctrl_set_kerneltime(mc);
		mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], 1, 0);
		mif_gpio_get_value(&mc->cp_gpio[CP_GPIO_AP2CP_AP_ACTIVE], true);
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

	g_mc = mc;

	s5100_get_ops(mc);
	if (s5100_get_pdata(mc, pdata)) {
		mif_err("DT error: failed to parse\n");
		ret = -EINVAL;
		goto err_dt_parse;
	}
	dev_set_drvdata(mc->dev, mc);

	mc->ws = cpif_wake_lock_register(&pdev->dev, "s5100_wake_lock");
	if (mc->ws == NULL) {
		mif_err("s5100_wake_lock: wakeup_source_register fail\n");
		ret = -EINVAL;
		goto err_wake_lock_register;
	}

	if (pdata->mif_off_during_volte) {
		mc->ws_wrst = cpif_wake_lock_register(&pdev->dev, "s5100_wrst_wake_lock");
		if (mc->ws_wrst == NULL) {
			mif_err("s5100_wake_lock: wakeup_source_register fail\n");
			ret = -EINVAL;
			goto err_wake_lock_register;
		}
	}

	mutex_init(&mc->pcie_onoff_lock);
	mutex_init(&mc->pcie_check_lock);
	spin_lock_init(&mc->pcie_tx_lock);
	spin_lock_init(&mc->pcie_pm_lock);
	spin_lock_init(&mc->power_stats_lock);
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_GPIO_WA)
	atomic_set(&mc->dump_toggle_issued, 0);
#endif

	mif_gpio_set_value(&mc->cp_gpio[CP_GPIO_AP2CP_NRESET], 0, 0);

	mif_info("Register GPIO interrupts\n");
	mc->apwake_irq_chip = irq_get_chip(mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_WAKEUP].num);
	if (mc->apwake_irq_chip == NULL) {
		mif_err("Can't get irq_chip structure!!!!\n");
		ret = -EINVAL;
		goto err_irq_get_chip;
	}

	if (pdata->mif_off_during_volte) {
		mc->cp_wrst_irq_chip = irq_get_chip(
			mc->cp_gpio_irq[CP_GPIO_IRQ_CP2AP_CP_WRST_N].num);
		if (mc->cp_wrst_irq_chip == NULL) {
			mif_err("Can't get irq_chip structure!!!!\n");
			ret = -EINVAL;
			goto err_irq_get_chip;
		}
	}

	mc->wakeup_wq = create_singlethread_workqueue("cp2ap_wakeup_wq");
	if (!mc->wakeup_wq) {
		mif_err("%s: ERR! fail to create wakeup_wq\n", mc->name);
		ret = -EINVAL;
		goto err_irq_get_chip;
	}
	INIT_WORK(&mc->wakeup_work, cp2ap_wakeup_work);
	INIT_WORK(&mc->suspend_work, cp2ap_suspend_work);

	mc->crash_wq = create_singlethread_workqueue("trigger_cp_crash_wq");
	if (!mc->crash_wq) {
		mif_err("%s: ERR! fail to create crash_wq\n", mc->name);
		ret = -EINVAL;
		goto err_crash_wq;
	}
	INIT_WORK(&mc->crash_work, trigger_cp_crash_work);

	mc->reboot_nb.notifier_call = s5100_reboot_handler;
	register_reboot_notifier(&mc->reboot_nb);

	/* Register PM notifier_call */
	mc->pm_notifier.notifier_call = s5100_pm_notifier;
	ret = register_pm_notifier(&mc->pm_notifier);
	if (ret) {
		mif_err("failed to register PM notifier_call\n");
		goto err_pm_notifier;
	}

	/* Register panic notifier_call*/
	mc->send_panic_nb.notifier_call = send_panic_to_cp_notifier;
	ret = atomic_notifier_chain_register(&panic_notifier_list, &mc->send_panic_nb);
	if (ret < 0) {
		mif_err("failed to register panic notifier_call\n");
		goto err_panic_notifier;
	}

#if IS_ENABLED(CONFIG_CPIF_AP_SUSPEND_DURING_VOICE_CALL)
	INIT_WORK(&mc->call_on_work, voice_call_on_work);
	INIT_WORK(&mc->call_off_work, voice_call_off_work);

	mc->call_state_nb.notifier_call = s5100_call_state_notifier;
	ret = register_modem_voice_call_event_notifier(&mc->call_state_nb);
	if (ret < 0) {
		mif_err("failed to register modem voice call event notifier_call\n");
		goto err_modem_vce_notifier;
	}
#endif

	mc->force_crash_nb.notifier_call = s5100_force_crash_notifier;
	ret = register_modem_force_crash_handler(&mc->force_crash_nb);
	if (ret < 0) {
		mif_err("failed to register forced crash notifier: %d\n", ret);
		goto err_force_crash_notifier;
	}

	if (sysfs_create_group(&pdev->dev.kobj, &sim_group))
		mif_err("failed to create sysfs node related sim\n");

	if (sysfs_create_group(&pdev->dev.kobj, &modem_group))
		mif_err("failed to create sysfs node related modem\n");

	ret = device_create_file(&pdev->dev, &dev_attr_s5100_wake_lock);
	if (ret) {
		mif_err("%s: couldn't create s5100_wake_lock(%d)\n", __func__, ret);
		goto err_dev_create_file;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_s5100_wrst_wake_lock);
	if (ret) {
		mif_err("%s: couldn't create s5100_wrst_wake_lock(%d)\n", __func__, ret);
		goto err_dev_create_file;
	}

	return 0;

err_dev_create_file:
	sysfs_remove_group(&pdev->dev.kobj, &modem_group);
	sysfs_remove_group(&pdev->dev.kobj, &sim_group);
	unregister_modem_force_crash_handler(&mc->force_crash_nb);
err_force_crash_notifier:
#if IS_ENABLED(CONFIG_CPIF_AP_SUSPEND_DURING_VOICE_CALL)
	unregister_modem_voice_call_event_notifier(&mc->call_state_nb);
err_modem_vce_notifier:
#endif
	atomic_notifier_chain_unregister(&panic_notifier_list, &mc->send_panic_nb);
err_panic_notifier:
	unregister_pm_notifier(&mc->pm_notifier);
err_pm_notifier:
	unregister_reboot_notifier(&mc->reboot_nb);
	destroy_workqueue(mc->crash_wq);
err_crash_wq:
	destroy_workqueue(mc->wakeup_wq);
err_irq_get_chip:
	cpif_wake_lock_unregister(mc->ws);
	if (pdata->mif_off_during_volte)
		cpif_wake_lock_unregister(mc->ws_wrst);
err_wake_lock_register:
err_dt_parse:
	g_mc = NULL;
	return ret;
}

void s5100_uninit_modemctl_device(struct modem_ctl *mc, struct modem_data *pdata)
{
	struct device *dev = mc->dev;

	device_remove_file(dev, &dev_attr_s5100_wake_lock);
	sysfs_remove_group(&dev->kobj, &modem_group);
	sysfs_remove_group(&dev->kobj, &sim_group);
	unregister_modem_force_crash_handler(&mc->force_crash_nb);
#if IS_ENABLED(CONFIG_CPIF_AP_SUSPEND_DURING_VOICE_CALL)
	unregister_modem_voice_call_event_notifier(&mc->call_state_nb);
#endif
	atomic_notifier_chain_unregister(&panic_notifier_list, &mc->send_panic_nb);
	unregister_pm_notifier(&mc->pm_notifier);
	unregister_reboot_notifier(&mc->reboot_nb);
	destroy_workqueue(mc->crash_wq);
	destroy_workqueue(mc->wakeup_wq);
	mutex_destroy(&mc->pcie_check_lock);
	mutex_destroy(&mc->pcie_onoff_lock);
	cpif_wake_lock_unregister(mc->ws);
	if (pdata->mif_off_during_volte)
		cpif_wake_lock_unregister(mc->ws_wrst);
	g_mc = NULL;
}
