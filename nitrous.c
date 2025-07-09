// SPDX-License-Identifier: GPL-2.0
/*
 * Bluetooth low power control via GPIO
 *
 * Copyright 2015-2020 Google LLC.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/proc_fs.h>
#include <linux/property.h>
#include <linux/rfkill.h>
#include <linux/rtc.h>
#include <misc/logbuffer.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <soc/google/exynos-cpupm.h>

#define STATUS_IDLE	1
#define STATUS_BUSY	0

#define NITROUS_TX_AUTOSUSPEND_DELAY   	100 /* autosleep delay 100 ms after TX */
#define NITROUS_RX_AUTOSUSPEND_DELAY   	1000 /* autosleep delay 1000 ms after RX */
#define TIMESYNC_TIMESTAMP_MAX_QUEUE	16
#define TIMESYNC_NOT_SUPPORTED		0
#define TIMESYNC_SUPPORTED 		1
#define TIMESYNC_ENABLED		2
#define DBO_NOT_SUPPORTED		0
#define DBO_SUPPORTED			1
#define DBO_NOT_CONFIGURED		0
#define DBO_CONFIGURED			1

struct nitrous_lpm_proc;

struct nitrous_bt_lpm {
	struct pinctrl *pinctrls;
	struct pinctrl_state *pinctrl_default_state;
	struct gpio_desc *gpio_dev_wake;     /* Host -> Dev WAKE GPIO */
	struct gpio_desc *gpio_host_wake;    /* Dev -> Host WAKE GPIO */
	struct gpio_desc *gpio_power;        /* GPIO to control power */
	struct gpio_desc *gpio_timesync;     /* GPIO for timesync */
	struct gpio_desc *gpio_ble_dbo;      /* GPIO for dbo */
	int irq_host_wake;           /* IRQ associated with HOST_WAKE GPIO */
	int wake_polarity;           /* 0: active low; 1: active high */

	bool is_suspended;           /* driver is in suspend state */
	bool uart_tx_dev_pm_resumed;

	int irq_timesync;            /* IRQ associated with TIMESYNC GPIO*/
	int timesync_state;
	struct kfifo timestamp_queue;

	int dbo_state;
	int dbo_config;
	bool off_mode_latch;

	struct device *dev;
	struct rfkill *rfkill;
	bool rfkill_blocked;         /* blocked: OFF; not blocked: ON */
	bool lpm_enabled;
	struct nitrous_lpm_proc *proc;
	struct logbuffer *log;
	int idle_bt_tx_ip_index;
	int idle_bt_rx_ip_index;
	int wakelock_ctrl;
};

#define PROC_BTWAKE	0
#define PROC_LPM	1
#define PROC_BTWRITE	2
#define PROC_WAKELOCK	3
#define PROC_TIMESYNC	4
#define PROC_DIR	"bluetooth/sleep"
struct proc_dir_entry *bluetooth_dir, *sleep_dir;

struct nitrous_lpm_proc {
	long operation;
	struct nitrous_bt_lpm *lpm;
};

/*
 * Wake up or sleep BT device for Tx.
 */
static inline void nitrous_wake_controller(struct nitrous_bt_lpm *lpm, bool wake)
{
	int assert_level = (wake == lpm->wake_polarity);
	struct timespec64 ts;
	ktime_get_real_ts64(&ts);
	dev_dbg(lpm->dev, "DEV_WAKE: %s", (assert_level ? "Assert" : "Dessert"));
	logbuffer_log(lpm->log, "DEV_WAKE: %s  %ptTt.%03ld",
		(assert_level ? "Assert" : "Dessert"), &ts, ts.tv_nsec / NSEC_PER_MSEC);
	gpiod_set_value_cansleep(lpm->gpio_dev_wake, assert_level);
}

/*
 * Called before UART driver starts transmitting data out. UART and BT resources
 * are requested to allow a transmission.
 */
static void nitrous_prepare_uart_tx_locked(struct nitrous_bt_lpm *lpm, bool assert)
{
	int ret;

	if (lpm->rfkill_blocked) {
		dev_err(lpm->dev, "unexpected Tx when rfkill is blocked\n");
		logbuffer_log(lpm->log, "unexpected Tx when rfkill is blocked");
		return;
	}

	if (assert && !lpm->uart_tx_dev_pm_resumed) {
		ret = pm_runtime_get_sync(lpm->dev);
		lpm->uart_tx_dev_pm_resumed = true;
		/* Shall be resumed here */
		logbuffer_log(lpm->log, "uart_tx_locked");

		if (lpm->is_suspended) {
			/* This shouldn't happen. If it does, it will result in a BT crash */
			/* TODO (mullerf): Does this happen? If yes, why? */
			dev_err(lpm->dev,"Tx in device suspended. ret: %d, uc:%d\n",
				ret, atomic_read(&lpm->dev->power.usage_count));
			logbuffer_log(lpm->log,"Tx in device suspended. ret: %d, uc:%d",
				ret, atomic_read(&lpm->dev->power.usage_count));
		}
	} else if (!assert && lpm->uart_tx_dev_pm_resumed) {
		logbuffer_log(lpm->log, "uart_tx_unlocked");
		pm_runtime_mark_last_busy(lpm->dev);
		pm_runtime_put_autosuspend(lpm->dev);
		lpm->uart_tx_dev_pm_resumed = false;
	}
}

/*
 * ISR to handle host wake line from the BT chip.
 *
 * If an interrupt is received during system suspend, the handling of the
 * interrupt will be delayed until the driver is resumed.  This allows the use
 * of pm runtime framework to wake the serial driver.
 */
static irqreturn_t nitrous_host_wake_isr(int irq, void *data)
{
	struct nitrous_bt_lpm *lpm = data;
	int host_wake;
	struct timespec64 ts;

	host_wake = gpiod_get_value(lpm->gpio_host_wake);
	dev_dbg(lpm->dev, "Host wake IRQ: %u\n", host_wake);

	if (lpm->rfkill_blocked) {
		dev_err(lpm->dev, "Unexpected Host wake IRQ\n");
		logbuffer_log(lpm->log, "Unexpected Host wake IRQ");
		return IRQ_HANDLED;
	}

	ktime_get_real_ts64(&ts);
	/* Check whether host_wake is ACTIVE (== 1) */
	if (host_wake == 1) {
		logbuffer_log(lpm->log, "host_wake_isr asserted %ptTt.%03ld",
			&ts, ts.tv_nsec / NSEC_PER_MSEC);
		pm_stay_awake(lpm->dev);
		exynos_update_ip_idle_status(lpm->idle_bt_rx_ip_index, STATUS_BUSY);
	} else {
		logbuffer_log(lpm->log, "host_wake_isr de-asserted %ptTt.%03ld",
			&ts, ts.tv_nsec / NSEC_PER_MSEC);
		exynos_update_ip_idle_status(lpm->idle_bt_rx_ip_index, STATUS_IDLE);
		pm_wakeup_dev_event(lpm->dev, lpm->wakelock_ctrl, false);
	}

	return IRQ_HANDLED;
}

static irqreturn_t ntirous_timesync_isr(int irq, void *data)
{
	struct nitrous_bt_lpm *lpm = data;
	ktime_t timestamp;
	dev_dbg(lpm->dev, "Timesync IRQ: %u\n", gpiod_get_value(lpm->gpio_timesync));
	timestamp = ktime_get_boottime();
	kfifo_in(&lpm->timestamp_queue, &timestamp, sizeof(ktime_t));
	logbuffer_log(lpm->log, "Timesync: %lld", ktime_to_us(timestamp));
	return IRQ_HANDLED;
}

static int nitrous_lpm_runtime_enable(struct nitrous_bt_lpm *lpm)
{
	int rc;

	if (lpm->irq_host_wake <= 0)
		return -EOPNOTSUPP;

	if (lpm->rfkill_blocked) {
		dev_err(lpm->dev, "Unexpected LPM request\n");
		logbuffer_log(lpm->log, "Unexpected LPM request");
		return -EINVAL;
	}

	if (lpm->lpm_enabled) {
		dev_warn(lpm->dev, "Try to request LPM twice\n");
		return 0;
	}

	/* Set irq_host_wake as a trigger edge interrupt. */
	rc = devm_request_irq(lpm->dev, lpm->irq_host_wake, nitrous_host_wake_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "bt_host_wake", lpm);
	if (unlikely(rc)) {
		dev_err(lpm->dev, "Unable to request IRQ for bt_host_wake GPIO\n");
		logbuffer_log(lpm->log, "Unable to request IRQ for bt_host_wake GPIO");
		lpm->irq_host_wake = rc;
		return rc;
	}

	device_init_wakeup(lpm->dev, true);
	pm_runtime_enable(lpm->dev);
	pm_runtime_set_autosuspend_delay(lpm->dev, NITROUS_TX_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(lpm->dev);

	/* When LPM is enabled, we resume the device right away.
	It will autosuspend automatically if unused. */
	dev_dbg(lpm->dev, "DEV_WAKE: High - LPM enable");
	logbuffer_log(lpm->log, "DEV_WAKE: High - LPM enable");
	pm_runtime_get_sync(lpm->dev);
	pm_runtime_mark_last_busy(lpm->dev);
	pm_runtime_put_autosuspend(lpm->dev);

	lpm->lpm_enabled = true;

	return rc;
}

static void nitrous_lpm_runtime_disable(struct nitrous_bt_lpm *lpm)
{
	if (lpm->irq_host_wake <= 0)
		return;

	if (!lpm->lpm_enabled)
		return;

	devm_free_irq(lpm->dev, lpm->irq_host_wake, lpm);
	device_init_wakeup(lpm->dev, false);
	pm_relax(lpm->dev);
	/* Check whether usage_counter got out of sync */
	if (atomic_read(&lpm->dev->power.usage_count)) {
		dev_warn(lpm->dev, "Usage counter went out of sync: %d",
			atomic_read(&lpm->dev->power.usage_count));
		logbuffer_log(lpm->log, "Usage counter went out of sync: %d",
			atomic_read(&lpm->dev->power.usage_count));
		/* Force set it to 0 */
		atomic_set(&lpm->dev->power.usage_count, 0);
	}
	dev_dbg(lpm->dev, "DEV_WAKE: Low - LPM disable");
	logbuffer_log(lpm->log, "DEV_WAKE: Low - LPM disable");
	pm_runtime_suspend(lpm->dev);
	pm_runtime_disable(lpm->dev);
	pm_runtime_set_suspended(lpm->dev);

	lpm->uart_tx_dev_pm_resumed = false;
	lpm->lpm_enabled = false;
}

static int nitrous_proc_show(struct seq_file *m, void *v)
{
	struct nitrous_lpm_proc *data = m->private;
	struct nitrous_bt_lpm *lpm = data->lpm;
	ktime_t timestamp;
	unsigned int ret;

	switch (data->operation) {
	case PROC_BTWAKE:
		seq_printf(m, "LPM: %s\nPolarity: %s\nHOST_WAKE: %u\nDEV_WAKE: %u\n",
			   (lpm->lpm_enabled ? "Enabled" : "Disabled"),
			   (lpm->wake_polarity ? "High" : "Low"),
			   gpiod_get_value(lpm->gpio_host_wake),
			   gpiod_get_value(lpm->gpio_dev_wake));
		break;
	case PROC_LPM:
	case PROC_BTWRITE:
		seq_printf(m, "REG_ON: %s\nLPM: %s\nState: %s\n",
			   (lpm->rfkill_blocked ? "OFF" : "ON"),
			   (lpm->lpm_enabled ? "Enabled" : "Disabled"),
			   (lpm->is_suspended ? "asleep" : "awake"));
		break;
	case PROC_TIMESYNC:
		ret = kfifo_out(&lpm->timestamp_queue, &timestamp, sizeof(ktime_t));
		if (ret != sizeof(ktime_t))
			timestamp = 0;
		seq_printf(m, "%lld", ktime_to_us(timestamp));
		break;
	case PROC_WAKELOCK:
		dev_info(lpm->dev, "%s: current rx wakelock time: %d\n", __func__, lpm->wakelock_ctrl);
		seq_printf(m, "%d\n", lpm->wakelock_ctrl);
		break;
	default:
		return 0;
	}
	return 0;
}

static int nitrous_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nitrous_proc_show, pde_data(inode));
}

static ssize_t nitrous_proc_write(struct file *file, const char *buf,
	size_t count, loff_t *pos)
{
	struct nitrous_lpm_proc *data = pde_data(file_inode(file));
	struct nitrous_bt_lpm *lpm = data->lpm;
	struct timespec64 ts;
	char lbuf[6];
	int rc, value;

	if (count >= sizeof(lbuf))
		count = sizeof(lbuf) - 1;
	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;
        /* Null-terminate the string */
        lbuf[count] = '\0';

	switch (data->operation) {
	case PROC_LPM:
		if (lbuf[0] == '1') {
			dev_info(lpm->dev, "LPM enabling\n");
			logbuffer_log(lpm->log, "PROC_LPM: enable");
			rc = nitrous_lpm_runtime_enable(lpm);
			if (unlikely(rc))
				return rc;
		} else if (lbuf[0] == '0') {
			dev_info(lpm->dev, "LPM disabling\n");
			logbuffer_log(lpm->log, "PROC_LPM: disable");
			nitrous_lpm_runtime_disable(lpm);
		} else {
			dev_warn(lpm->dev, "Unknown LPM operation\n");
			logbuffer_log(lpm->log, "PROC_LPM: unknown");
			return -EFAULT;
		}
		break;
	case PROC_BTWRITE:
		if (!lpm->lpm_enabled) {
			dev_info(lpm->dev, "LPM not enabled\n");
			logbuffer_log(lpm->log, "PROC_BTWRITE: not enabled");
			return count;
		}
		if (lbuf[0] == '1') {
			dev_dbg(lpm->dev, "LPM waking up for Tx\n");
			ktime_get_real_ts64(&ts);
			logbuffer_log(lpm->log, "PROC_BTWRITE: waking up %ptTt.%03ld",
				&ts, ts.tv_nsec / NSEC_PER_MSEC);
			nitrous_prepare_uart_tx_locked(lpm, true);
		} else if (lbuf[0] == '0') {
			dev_dbg(lpm->dev, "LPM Tx done\n");
			ktime_get_real_ts64(&ts);
			logbuffer_log(lpm->log, "PROC_BTWRITE: Tx done %ptTt.%03ld",
				&ts, ts.tv_nsec / NSEC_PER_MSEC);
			nitrous_prepare_uart_tx_locked(lpm, false);
		}
		break;
	case PROC_WAKELOCK:
		rc = kstrtoint(lbuf, 10, &value); // Convert string to integer
		if (rc) {
			dev_err(lpm->dev, "Invalid integer input\n");
			return -EINVAL;
		}
		lpm->wakelock_ctrl = value;
		dev_info(lpm->dev, "%s: set wakelock_ctrl: %d\n", __func__, value);
		break;
	default:
		return 0;
	}
	return count;
}

static const struct proc_ops nitrous_proc_read_fops = {
	.proc_open	= nitrous_proc_open,
	.proc_read	= seq_read,
	.proc_release	= single_release,
};

static const struct proc_ops nitrous_proc_readwrite_fops = {
	.proc_open	= nitrous_proc_open,
	.proc_read	= seq_read,
	.proc_write	= nitrous_proc_write,
	.proc_release	= single_release,
};

static void nitrous_lpm_remove_proc_entries(struct nitrous_bt_lpm *lpm)
{
	if (bluetooth_dir == NULL)
		return;
	if (sleep_dir) {
		remove_proc_entry("btwrite", sleep_dir);
		remove_proc_entry("lpm", sleep_dir);
		remove_proc_entry("btwake", sleep_dir);
		remove_proc_entry("sleep", bluetooth_dir);
	}
	if (lpm->wakelock_ctrl) {
		remove_proc_entry("wakelock_ctrl", bluetooth_dir);
	}
	if (lpm->timesync_state) {
		remove_proc_entry("timesync", bluetooth_dir);
	}
	remove_proc_entry("bluetooth", 0);
	if (lpm->proc) {
		devm_kfree(lpm->dev, lpm->proc);
		lpm->proc = NULL;
	}
}

static void toggle_timesync(struct nitrous_bt_lpm *lpm) {
	int rc;

	if (lpm->timesync_state == TIMESYNC_NOT_SUPPORTED)
		return;
	rc = devm_request_irq(lpm->dev, lpm->irq_timesync, ntirous_timesync_isr,
			IRQF_TRIGGER_RISING, "bt_timesync", lpm);
	if (unlikely(rc)) {
		lpm->timesync_state = TIMESYNC_SUPPORTED;
		dev_logbuffer_logk(lpm->dev, lpm->log, LOGLEVEL_ERR, "Unable to request IRQ for bt_timesync GPIO");
	} else {
		lpm->timesync_state = TIMESYNC_ENABLED;
		logbuffer_log(lpm->log, "Rquest IRQ for bt_timesync GPIO successful");
	}
}


static int nitrous_lpm_init(struct nitrous_bt_lpm *lpm)
{
	int rc, proc_size = 4;
	unsigned long fifo_size = 0;
	struct proc_dir_entry *entry;
	struct nitrous_lpm_proc *data;

	lpm->irq_host_wake = gpiod_to_irq(lpm->gpio_host_wake);
	dev_info(lpm->dev, "IRQ: %d active: %s\n", lpm->irq_host_wake,
		(lpm->wake_polarity ? "High" : "Low"));
	logbuffer_log(lpm->log, "init: IRQ: %d active: %s", lpm->irq_host_wake,
		(lpm->wake_polarity ? "High" : "Low"));

	lpm->is_suspended = true;

	if (lpm->timesync_state) {
		lpm->irq_timesync = gpiod_to_irq(lpm->gpio_timesync);

		fifo_size = TIMESYNC_TIMESTAMP_MAX_QUEUE * sizeof(ktime_t);
		fifo_size = roundup_pow_of_two(fifo_size);
		if (kfifo_alloc(&lpm->timestamp_queue, fifo_size, GFP_KERNEL)) {
			dev_err(lpm->dev, "Failed to alloc queue for Timesync");
			logbuffer_log(lpm->log, "Failed to alloc queue for Timesync");
			return -ENOMEM;
		}

		proc_size += 1;
	}

	data = devm_kzalloc(lpm->dev, sizeof(struct nitrous_lpm_proc) * proc_size, GFP_KERNEL);
	if (data == NULL) {
		dev_err(lpm->dev, "Unable to alloc memory");
		logbuffer_log(lpm->log, "Unable to alloc memory");
		return -ENOMEM;
	}
	lpm->proc = data;

	bluetooth_dir = proc_mkdir("bluetooth", NULL);
	if (bluetooth_dir == NULL) {
		dev_err(lpm->dev, "Unable to create /proc/bluetooth directory");
		logbuffer_log(lpm->log, "Unable to create /proc/bluetooth directory");
		rc = -ENOMEM;
		goto fail;
	}
	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (sleep_dir == NULL) {
		dev_err(lpm->dev, "Unable to create /proc/%s directory", PROC_DIR);
		logbuffer_log(lpm->log, "Unable to create /proc/%s directory", PROC_DIR);
		rc = -ENOMEM;
		goto fail;
	}
	/* Creating read only proc entries "btwake" showing GPIOs state */
	data[0].operation = PROC_BTWAKE;
	data[0].lpm = lpm;
	entry = proc_create_data("btwake", (S_IRUSR | S_IRGRP), sleep_dir,
				 &nitrous_proc_read_fops, data);
	if (entry == NULL) {
		dev_err(lpm->dev, "Unable to create /proc/%s/btwake entry", PROC_DIR);
		logbuffer_log(lpm->log, "Unable to create /proc/%s/btwake entry", PROC_DIR);
		rc = -ENOMEM;
		goto fail;
	}
	/* read/write proc entries "lpm" */
	data[1].operation = PROC_LPM;
	data[1].lpm = lpm;
	entry = proc_create_data("lpm", (S_IRUSR | S_IRGRP | S_IWUSR),
			sleep_dir, &nitrous_proc_readwrite_fops, data + 1);
	if (entry == NULL) {
		dev_err(lpm->dev, "Unable to create /proc/%s/lpm entry", PROC_DIR);
		logbuffer_log(lpm->log, "Unable to create /proc/%s/lpm entry", PROC_DIR);
		rc = -ENOMEM;
		goto fail;
	}
	/* read/write proc entries "btwrite" */
	data[2].operation = PROC_BTWRITE;
	data[2].lpm = lpm;
	entry = proc_create_data("btwrite", (S_IRUSR | S_IRGRP | S_IWUSR),
			sleep_dir, &nitrous_proc_readwrite_fops, data + 2);
	if (entry == NULL) {
		dev_err(lpm->dev, "Unable to create /proc/%s/btwrite entry", PROC_DIR);
		logbuffer_log(lpm->log, "Unable to create /proc/%s/btwrite entry", PROC_DIR);
		rc = -ENOMEM;
		goto fail;
	}

	if (lpm->wakelock_ctrl) {
		/* read/write proc entries "wakelock_ctrl" */
		data[3].operation = PROC_WAKELOCK;
		data[3].lpm = lpm;
		entry = proc_create_data("wakelock_ctrl", (S_IRUSR | S_IRGRP | S_IWUSR),
								 sleep_dir, &nitrous_proc_readwrite_fops, data + 3);
		if (entry == NULL) {
			dev_err(lpm->dev, "Unable to create /proc/%s/wakelock_ctrl entry", PROC_DIR);
			logbuffer_log(lpm->log, "Unable to create /proc/%s/wakelock_ctrl entry", PROC_DIR);
			rc = -ENOMEM;
			goto fail;
		}
	}

	if (lpm->timesync_state) {
		/* read/write proc entries "timesync" */
		data[4].operation = PROC_TIMESYNC;
		data[4].lpm = lpm;
		entry = proc_create_data("timesync", (S_IRUSR | S_IRGRP),
				bluetooth_dir, &nitrous_proc_read_fops, data + 4);
		if (entry == NULL) {
			dev_err(lpm->dev, "Unable to create /proc/bluetooth/timesync entry");
			logbuffer_log(lpm->log, "Unable to create /proc/bluetooth/timesync entry");
			rc = -ENOMEM;
			goto fail;
		}
		toggle_timesync(lpm);
	}

	return 0;

fail:
	nitrous_lpm_remove_proc_entries(lpm);
	return rc;
}

static void nitrous_lpm_cleanup(struct nitrous_bt_lpm *lpm)
{
	nitrous_lpm_runtime_disable(lpm);
	lpm->irq_host_wake = 0;
	if (lpm->timesync_state) {
		lpm->irq_timesync = 0;
		kfifo_free(&lpm->timestamp_queue);
	}

	nitrous_lpm_remove_proc_entries(lpm);
}

/*
 * Toggle ble_dbo GPIO for Flip-Flop design
 */
static void toggle_dbo_ff(struct nitrous_bt_lpm *lpm)
{
	if (!lpm->off_mode_latch) {
		if (lpm->dbo_state) {
			gpiod_set_value_cansleep(lpm->gpio_ble_dbo, false);
			udelay(1);
			gpiod_set_value_cansleep(lpm->gpio_ble_dbo, true);
			udelay(1);
			gpiod_set_value_cansleep(lpm->gpio_ble_dbo, false);
		}
	}
}

/*
 * Set BT power on/off (blocked is true: OFF; blocked is false: ON)
 */
static int nitrous_rfkill_set_power(void *data, bool blocked)
{
	struct nitrous_bt_lpm *lpm = data;
	struct timespec64 ts;
	int ret;

	if (!lpm) {
		return -EINVAL;
	}

	dev_info(lpm->dev, "rfkill: %s (blocked=%d)\n", blocked ? "off" : "on",
		blocked);
	logbuffer_log(lpm->log, "rfkill: blocked=%s", blocked ? "off" : "on");

	if (blocked == lpm->rfkill_blocked) {
		dev_info(lpm->dev, "rfkill: already in requested state: %s\n",
			blocked ? "off" : "on");
		logbuffer_log(lpm->log, "rfkill: already in requested state: %s",
			blocked ? "off" : "on");
		return 0;
	}

	/* Reset to make sure LPM is disabled */
	nitrous_lpm_runtime_disable(lpm);
	ktime_get_real_ts64(&ts);

	if (!lpm->dbo_config) {
		ret = gpiod_direction_output(lpm->gpio_ble_dbo, 1);
		if (ret) {
			dev_info(lpm->dev, "DBO: ret = %d", ret);
		} else {
			lpm->dbo_config = DBO_CONFIGURED;
		}
	}

	if (!blocked) {
		/* Power up the BT chip. delay between consecutive toggles. */
		logbuffer_log(lpm->log, "Power up BT chip %ptTt", &ts);
		dev_dbg(lpm->dev, "REG_ON: Low");
		gpiod_set_value_cansleep(lpm->gpio_power, false);
		toggle_dbo_ff(lpm);
		msleep(30);
		exynos_update_ip_idle_status(lpm->idle_bt_tx_ip_index, STATUS_BUSY);
		exynos_update_ip_idle_status(lpm->idle_bt_rx_ip_index, STATUS_BUSY);
		dev_dbg(lpm->dev, "REG_ON: High");
		gpiod_set_value_cansleep(lpm->gpio_power, true);
		toggle_dbo_ff(lpm);
		/* Set DEV_WAKE to High as part of the power sequence */
		dev_dbg(lpm->dev, "DEV_WAKE: High - Power sequence");
		gpiod_set_value_cansleep(lpm->gpio_dev_wake, true);
	} else {
		/* Set DEV_WAKE to Low as part of the power sequence */
		dev_dbg(lpm->dev, "DEV_WAKE: Low - Power sequence");
		gpiod_set_value_cansleep(lpm->gpio_dev_wake, false);

		/* Power down the BT chip */
		logbuffer_log(lpm->log, "Power down BT chip %ptTt", &ts);
		dev_dbg(lpm->dev, "REG_ON: Low");
		gpiod_set_value_cansleep(lpm->gpio_power, false);
		toggle_dbo_ff(lpm);
		exynos_update_ip_idle_status(lpm->idle_bt_tx_ip_index, STATUS_IDLE);
		exynos_update_ip_idle_status(lpm->idle_bt_rx_ip_index, STATUS_IDLE);
	}
	lpm->rfkill_blocked = blocked;

	/* wait for device to power cycle and come out of reset */
	usleep_range(10000, 20000);

	return 0;
}

static const struct rfkill_ops nitrous_rfkill_ops = {
	.set_block = nitrous_rfkill_set_power,
};

static int nitrous_rfkill_init(struct nitrous_bt_lpm *lpm)
{
	int rc;

	lpm->gpio_power = devm_gpiod_get_optional(lpm->dev, "shutdown", GPIOD_OUT_LOW);
	if (IS_ERR(lpm->gpio_power))
		return PTR_ERR(lpm->gpio_power);

	lpm->rfkill = rfkill_alloc(
		"nitrous_bluetooth",
		lpm->dev,
		RFKILL_TYPE_BLUETOOTH,
		&nitrous_rfkill_ops,
		lpm
	);
	if (unlikely(!lpm->rfkill))
		return -ENOMEM;

	/* Make sure rfkill core is initialized to be blocked initially. */
	rfkill_init_sw_state(lpm->rfkill, true);
	rc = rfkill_register(lpm->rfkill);
	if (unlikely(rc))
		goto err_rfkill_register;

	/* Power off chip at startup. */
	nitrous_rfkill_set_power(lpm, true);
	return 0;

err_rfkill_register:
	rfkill_destroy(lpm->rfkill);
	lpm->rfkill = NULL;
	return rc;
}

static void nitrous_rfkill_cleanup(struct nitrous_bt_lpm *lpm)
{
	nitrous_rfkill_set_power(lpm, true);
	rfkill_unregister(lpm->rfkill);
	rfkill_destroy(lpm->rfkill);
	lpm->rfkill = NULL;
}

static int nitrous_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nitrous_bt_lpm *lpm;
	int rc, wakelock_time = 0;

	lpm = devm_kzalloc(dev, sizeof(struct nitrous_bt_lpm), GFP_KERNEL);
	if (!lpm)
		return -ENOMEM;

	lpm->dev = dev;
	dev_dbg(lpm->dev, "probe:\n");

	if (device_property_read_u32(dev, "wake-polarity", &lpm->wake_polarity)) {
		dev_warn(lpm->dev, "Wake polarity not in dev tree\n");
		lpm->wake_polarity = 1;
	}

	lpm->pinctrls = devm_pinctrl_get(lpm->dev);
	if (IS_ERR(lpm->pinctrls)) {
		dev_warn(lpm->dev, "Can't get pinctrl\n");
	} else {
		lpm->pinctrl_default_state =
			pinctrl_lookup_state(lpm->pinctrls, "default");
		if (IS_ERR(lpm->pinctrl_default_state))
			dev_warn(lpm->dev, "Can't get default pinctrl state\n");
	}

	lpm->gpio_dev_wake = devm_gpiod_get_optional(dev, "device-wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(lpm->gpio_dev_wake))
		return PTR_ERR(lpm->gpio_dev_wake);

	lpm->gpio_host_wake = devm_gpiod_get_optional(dev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(lpm->gpio_host_wake))
		return PTR_ERR(lpm->gpio_host_wake);

	lpm->gpio_timesync = devm_gpiod_get_optional(dev, "timesync", GPIOD_IN);
	lpm->timesync_state = TIMESYNC_NOT_SUPPORTED;
	if (IS_ERR(lpm->gpio_timesync)) {
		dev_warn(lpm->dev, "Can't get Timesync GPIO descriptor\n");
	} else if (lpm->gpio_timesync) {
		lpm->timesync_state = TIMESYNC_SUPPORTED;
	}
	dev_dbg(lpm->dev, "Timesync support: %x", lpm->timesync_state);

	lpm->gpio_ble_dbo = devm_gpiod_get_optional(dev, "bt-ble-dbo-le", GPIOD_IN);
	lpm->dbo_state = DBO_NOT_SUPPORTED;
	if (IS_ERR(lpm->gpio_ble_dbo)) {
		dev_warn(lpm->dev, "Can't get dbo GPIO descriptor\n");
	} else if (lpm->gpio_ble_dbo) {
		lpm->dbo_state = DBO_SUPPORTED;
	}
	dev_dbg(lpm->dev, "DBO support: %x", lpm->dbo_state);

	lpm->off_mode_latch = device_property_read_bool(lpm->dev, "off-mode-latch");

	if (!of_property_read_u32(pdev->dev.of_node, "goog,wakelock-ctrl",
				&wakelock_time)) {
		dev_info(lpm->dev, "Using Wakelock Control time (%d)", wakelock_time);
		lpm->wakelock_ctrl = wakelock_time;
	} else {
		dev_info(lpm->dev, "Using default Wakelock time");
		lpm->wakelock_ctrl = NITROUS_RX_AUTOSUSPEND_DELAY;
	}

	lpm->log = logbuffer_register("btlpm");
	if (IS_ERR_OR_NULL(lpm->log)) {
		dev_info(lpm->dev, "logbuffer get failed\n");
		lpm->log = NULL;
	}

	rc = nitrous_lpm_init(lpm);
	if (unlikely(rc))
		goto err_lpm_init;

	rc = nitrous_rfkill_init(lpm);
	if (unlikely(rc))
		goto err_rfkill_init;

	if (!IS_ERR_OR_NULL(lpm->pinctrl_default_state)) {
		rc = pinctrl_select_state(lpm->pinctrls,
					  lpm->pinctrl_default_state);
		if (unlikely(rc))
			dev_warn(lpm->dev, "Can't set default pinctrl state\n");
	}

	platform_set_drvdata(pdev, lpm);

	lpm->idle_bt_tx_ip_index = exynos_get_idle_ip_index("bluetooth-tx");
	exynos_update_ip_idle_status(lpm->idle_bt_tx_ip_index, STATUS_IDLE);

	lpm->idle_bt_rx_ip_index = exynos_get_idle_ip_index("bluetooth-rx");
	exynos_update_ip_idle_status(lpm->idle_bt_rx_ip_index, STATUS_IDLE);

	lpm->dbo_config = DBO_NOT_CONFIGURED;

	logbuffer_log(lpm->log, "probe: successful");

	return rc;

err_rfkill_init:
	nitrous_rfkill_cleanup(lpm);
err_lpm_init:
	nitrous_lpm_cleanup(lpm);
	devm_kfree(dev, lpm);
	return rc;
}

static int nitrous_remove(struct platform_device *pdev)
{
	struct nitrous_bt_lpm *lpm = platform_get_drvdata(pdev);

	if (!lpm) {
		return -EINVAL;
	}

	logbuffer_log(lpm->log, "removing");
	nitrous_rfkill_cleanup(lpm);
	nitrous_lpm_cleanup(lpm);
	if (!IS_ERR_OR_NULL(lpm->log))
		logbuffer_unregister(lpm->log);

	devm_kfree(&pdev->dev, lpm);

	return 0;
}

static int nitrous_suspend_device(struct device *dev)
{
	struct nitrous_bt_lpm *lpm = dev_get_drvdata(dev);

	dev_dbg(lpm->dev, "suspend_device from %s\n",
		(lpm->is_suspended ? "asleep" : "awake"));
	logbuffer_log(lpm->log, "suspend_device from %s",
		(lpm->is_suspended ? "asleep" : "awake"));

	nitrous_wake_controller(lpm, false);
	exynos_update_ip_idle_status(lpm->idle_bt_tx_ip_index, STATUS_IDLE);
	lpm->is_suspended = true;

	return 0;
}

static int nitrous_resume_device(struct device *dev)
{
	struct nitrous_bt_lpm *lpm = dev_get_drvdata(dev);

	dev_dbg(lpm->dev, "resume_device from %s\n",
		(lpm->is_suspended ? "asleep" : "awake"));
	logbuffer_log(lpm->log, "resume_device from %s",
		(lpm->is_suspended ? "asleep" : "awake"));

	exynos_update_ip_idle_status(lpm->idle_bt_tx_ip_index, STATUS_BUSY);
	nitrous_wake_controller(lpm, true);
	lpm->is_suspended = false;

	return 0;
}

static int nitrous_suspend(struct device *dev)
{
	struct nitrous_bt_lpm *lpm = dev_get_drvdata(dev);
	struct timespec64 ts;

	if (lpm->rfkill_blocked)
		return 0;

	ktime_get_real_ts64(&ts);
	dev_dbg(lpm->dev, "nitrous_suspend %ptTt\n", &ts);
	logbuffer_log(lpm->log, "nitrous_suspend %ptTt", &ts);

	if (device_may_wakeup(dev) && lpm->lpm_enabled) {
		pm_runtime_force_suspend(lpm->dev);
		logbuffer_log(lpm->log, "pm_runtime_force_suspend");
		enable_irq_wake(lpm->irq_host_wake);
		dev_dbg(lpm->dev, "Host wake IRQ enabled\n");
		logbuffer_log(lpm->log, "Host wake IRQ enabled");
	}

	return 0;
}

static int nitrous_resume(struct device *dev)
{
	struct nitrous_bt_lpm *lpm = dev_get_drvdata(dev);
	struct timespec64 ts;

	if (lpm->rfkill_blocked)
		return 0;

	ktime_get_real_ts64(&ts);
	dev_dbg(lpm->dev, "nitrous_resume %ptTt\n", &ts);
	logbuffer_log(lpm->log, "nitrous_resume %ptTt", &ts);

	if (device_may_wakeup(dev) && lpm->lpm_enabled) {
		disable_irq_wake(lpm->irq_host_wake);
		dev_dbg(lpm->dev, "Host wake IRQ disabled\n");
		logbuffer_log(lpm->log, "Host wake IRQ disabled");
		pm_runtime_force_resume(lpm->dev);
		logbuffer_log(lpm->log, "pm_runtime_force_resume");
	}

	return 0;
}

static struct of_device_id nitrous_match_table[] = {
	{ .compatible = "goog,nitrous" },
	{}
};
MODULE_DEVICE_TABLE(of, nitrous_match_table);

static const struct dev_pm_ops nitrous_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nitrous_suspend, nitrous_resume)
	SET_RUNTIME_PM_OPS(nitrous_suspend_device, nitrous_resume_device, NULL)
};

static struct platform_driver nitrous_platform_driver = {
	.probe = nitrous_probe,
	.remove =  nitrous_remove,
	.driver = {
		.name = "nitrous_bluetooth",
		.owner = THIS_MODULE,
		.of_match_table = nitrous_match_table,
		.pm = &nitrous_pm_ops,
	},
};

static int __init nitrous_init(void)
{
	return platform_driver_register(&nitrous_platform_driver);
}

static void __exit nitrous_exit(void)
{
	platform_driver_unregister(&nitrous_platform_driver);
}

module_init(nitrous_init);
module_exit(nitrous_exit);
MODULE_DESCRIPTION("Nitrous Oxide Driver for Bluetooth");
MODULE_AUTHOR("Google");
MODULE_LICENSE("GPL");
