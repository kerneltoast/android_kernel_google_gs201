// SPDX-License-Identifier: <GPL-2.0>
/*
 * Copyright (C) 2016 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <net/nfc/nci.h>
#include <linux/clk.h>
#include <soc/google/exynos-pmu-if.h>
#include "st21nfc.h"

#define MAX_BUFFER_SIZE 260
#define HEADER_LENGTH 3
#define IDLE_CHARACTER 0x7e
#define ST21NFC_POWER_STATE_MAX 3
#define WAKEUP_SRC_TIMEOUT		(2000)
#define EXYNOS_CLK_MASK		0x01

#define DRIVER_VERSION "2.0.19"

#define PROP_PWR_MON_RW_ON_NTF nci_opcode_pack(NCI_GID_PROPRIETARY, 5)
#define PROP_PWR_MON_RW_OFF_NTF nci_opcode_pack(NCI_GID_PROPRIETARY, 6)

/*The enum is used to index a pw_states array, the values matter here*/
enum st21nfc_power_state {
	ST21NFC_IDLE = 0,
	ST21NFC_ACTIVE = 1,
	ST21NFC_ACTIVE_RW = 2
};

static const char *const st21nfc_power_state_name[] = {
	"IDLE", "ACTIVE", "ACTIVE_RW"
};

enum st21nfc_read_state {
	ST21NFC_HEADER,
	ST21NFC_PAYLOAD
};

struct nfc_sub_power_stats {
	uint64_t count;
	uint64_t duration;
	uint64_t last_entry;
	uint64_t last_exit;
};

struct nfc_sub_power_stats_error {
	/* error transition header --> payload state machine */
	uint64_t header_payload;
	/* error transition from an active state when not in idle state */
	uint64_t active_not_idle;
	/* error transition from idle state to idle state */
	uint64_t idle_to_idle;
	/* warning transition from active_rw state to idle state */
	uint64_t active_rw_to_idle;
	/* error transition from active state to active state */
	uint64_t active_to_active;
	/* error transition from idle state to active state with notification */
	uint64_t idle_to_active_ntf;
	/* error transition from active_rw state to active_rw state */
	uint64_t act_rw_to_act_rw;
	/* error transition from idle state to */
	/* active_rw state with notification   */
	uint64_t idle_to_active_rw_ntf;
};

/*
 * The member 'polarity_mode' defines
 * how the wakeup pin is configured and handled.
 * it can take the following values :
 * IRQF_TRIGGER_RISING
 * IRQF_TRIGGER_HIGH
 */
struct st21nfc_device {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct mutex pidle_mutex;
	struct i2c_client *client;
	struct miscdevice st21nfc_device;
	uint8_t buffer[MAX_BUFFER_SIZE];
	bool irq_enabled;
	bool irq_wake_up;
	bool irq_is_attached;
	bool device_open; /* Is device open? */
	spinlock_t irq_enabled_lock;
	enum st21nfc_power_state pw_current;
	enum st21nfc_read_state r_state_current;
	int irq_pw_stats_idle;
	struct nfc_sub_power_stats pw_states[ST21NFC_POWER_STATE_MAX];
	struct nfc_sub_power_stats_error pw_states_err;
	struct workqueue_struct *st_p_wq;
	struct work_struct st_p_work;
	/*Power state shadow copies for reading*/
	enum st21nfc_power_state c_pw_current;
	struct nfc_sub_power_stats c_pw_states[ST21NFC_POWER_STATE_MAX];
	struct nfc_sub_power_stats_error c_pw_states_err;

	/* CLK control */
	bool clk_run;
	struct clk *s_clk;
	uint8_t pinctrl_en;
	bool pidle_active_low;
	int irq_clkreq;
	unsigned int clk_pad;

	/* GPIO for NFCC IRQ pin (input) */
	struct gpio_desc *gpiod_irq;
	/* GPIO for NFCC Reset pin (output) */
	struct gpio_desc *gpiod_reset;
	/* GPIO for NFCC CLK_REQ pin (input) */
	struct gpio_desc *gpiod_clkreq;
	/* GPIO for NFCC CLF_MONITOR_PWR (input) */
	struct gpio_desc *gpiod_pidle;
	/* irq_gpio polarity to be used */
	unsigned int polarity_mode;
};

/*
 * Routine to enable clock.
 * this routine can be extended to select from multiple
 * sources based on clk_src_name.
 */
static int st21nfc_clock_select(struct st21nfc_device *st21nfc_dev)
{
	int ret = 0;

	st21nfc_dev->s_clk = clk_get(&st21nfc_dev->client->dev, "nfc_ref_clk");

	/* if NULL we assume external crystal and dont fail */
	if ((st21nfc_dev->s_clk == NULL) || IS_ERR(st21nfc_dev->s_clk))
		return 0;

	if (st21nfc_dev->clk_run == false) {
		ret = clk_prepare_enable(st21nfc_dev->s_clk);

		if (ret)
			goto err_clk;

		st21nfc_dev->clk_run = true;
	}
	return ret;

err_clk:
	return -EINVAL;
}

/*
 * Routine to disable clocks
 */
static int st21nfc_clock_deselect(struct st21nfc_device *st21nfc_dev)
{
	/* if NULL we assume external crystal and dont fail */
	if ((st21nfc_dev->s_clk == NULL) || IS_ERR(st21nfc_dev->s_clk))
		return 0;

	if (st21nfc_dev->clk_run == true) {
		clk_disable_unprepare(st21nfc_dev->s_clk);
		st21nfc_dev->clk_run = false;
	}
	return 0;
}

static void st21nfc_exynos_clk_control(struct st21nfc_device *st21nfc_dev,
				       bool enable)
{
	if (st21nfc_dev->clk_pad) {
		exynos_pmu_update(st21nfc_dev->clk_pad, EXYNOS_CLK_MASK, enable ? 1 : 0);
	}
}

static irqreturn_t st21nfc_clkreq_irq_handler(int irq, void *dev_id)
{
	struct st21nfc_device *st21nfc_dev = dev_id;
	int value = gpiod_get_value(st21nfc_dev->gpiod_clkreq);

	if (st21nfc_dev->pinctrl_en) {
		st21nfc_exynos_clk_control(st21nfc_dev, value ? true : false);
	}
	return IRQ_HANDLED;
}

static void st21nfc_disable_irq(struct st21nfc_device *st21nfc_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&st21nfc_dev->irq_enabled_lock, flags);
	if (st21nfc_dev->irq_enabled) {
		disable_irq_nosync(st21nfc_dev->client->irq);
		st21nfc_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&st21nfc_dev->irq_enabled_lock, flags);
}

static void st21nfc_enable_irq(struct st21nfc_device *st21nfc_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&st21nfc_dev->irq_enabled_lock, flags);
	if (!st21nfc_dev->irq_enabled) {
		st21nfc_dev->irq_enabled = true;
		enable_irq(st21nfc_dev->client->irq);

	}
	spin_unlock_irqrestore(&st21nfc_dev->irq_enabled_lock, flags);
}

static irqreturn_t st21nfc_dev_irq_handler(int irq, void *dev_id)
{
	struct st21nfc_device *st21nfc_dev = dev_id;

	if (device_may_wakeup(&st21nfc_dev->client->dev))
		pm_wakeup_event(&st21nfc_dev->client->dev,
			WAKEUP_SRC_TIMEOUT);
	st21nfc_disable_irq(st21nfc_dev);

	/* Wake up waiting readers */
	wake_up(&st21nfc_dev->read_wq);

	return IRQ_HANDLED;
}

static int st21nfc_loc_set_polaritymode(struct st21nfc_device *st21nfc_dev,
					int mode)
{
	struct i2c_client *client = st21nfc_dev->client;
	struct device *dev = &client->dev;
	unsigned int irq_type;
	int ret;

	st21nfc_dev->polarity_mode = mode;
	/* setup irq_flags */
	switch (mode) {
	case IRQF_TRIGGER_RISING:
		irq_type = IRQ_TYPE_EDGE_RISING;
		break;
	case IRQF_TRIGGER_HIGH:
		irq_type = IRQ_TYPE_LEVEL_HIGH;
		break;
	default:
		irq_type = IRQ_TYPE_EDGE_RISING;
		break;
	}
	if (st21nfc_dev->irq_is_attached) {
		devm_free_irq(dev, client->irq, st21nfc_dev);
		st21nfc_dev->irq_is_attached = false;
	}
	ret = irq_set_irq_type(client->irq, irq_type);
	if (ret) {
		dev_err(dev, "%s : set_irq_type failed\n", __func__);
		return -ENODEV;
	}
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	dev_dbg(dev, "%s : requesting IRQ %d\n", __func__, client->irq);
	st21nfc_dev->irq_enabled = true;

	ret = devm_request_irq(dev, client->irq, st21nfc_dev_irq_handler,
				st21nfc_dev->polarity_mode,
				client->name, st21nfc_dev);
	if (ret) {
		dev_err(dev, "%s : devm_request_irq failed\n", __func__);
		return -ENODEV;
	}
	st21nfc_dev->irq_is_attached = true;
	st21nfc_disable_irq(st21nfc_dev);

	return ret;
}


static void st21nfc_power_stats_switch(
	struct st21nfc_device *st21nfc_dev, uint64_t current_time_ms,
	enum st21nfc_power_state old_state, enum st21nfc_power_state new_state,
	bool is_ntf)
{
	mutex_lock(&st21nfc_dev->pidle_mutex);

	if (new_state == old_state) {
		if ((st21nfc_dev->pw_states[ST21NFC_IDLE].last_entry != 0) ||
		    (old_state != ST21NFC_IDLE)) {
			dev_err(&st21nfc_dev->client->dev,
				"%s Error: Switched from %s to %s!: %llx, ntf=%d\n",
				__func__, st21nfc_power_state_name[old_state],
				st21nfc_power_state_name[new_state],
				current_time_ms, is_ntf);
			if (new_state == ST21NFC_IDLE)
				st21nfc_dev->pw_states_err.idle_to_idle++;
			else if (new_state == ST21NFC_ACTIVE)
				st21nfc_dev->pw_states_err.active_to_active++;
			else if (new_state == ST21NFC_ACTIVE_RW)
				st21nfc_dev->pw_states_err.act_rw_to_act_rw++;

			mutex_unlock(&st21nfc_dev->pidle_mutex);
			return;
		}
	} else if (!is_ntf &&
		new_state == ST21NFC_ACTIVE &&
		old_state != ST21NFC_IDLE) {
		st21nfc_dev->pw_states_err.active_not_idle++;
	} else if (!is_ntf &&
		new_state == ST21NFC_IDLE &&
		old_state == ST21NFC_ACTIVE_RW) {
		st21nfc_dev->pw_states_err.active_rw_to_idle++;
	} else if (is_ntf &&
		new_state == ST21NFC_ACTIVE &&
		old_state == ST21NFC_IDLE) {
		st21nfc_dev->pw_states_err.idle_to_active_ntf++;
	} else if (is_ntf &&
		new_state == ST21NFC_ACTIVE_RW &&
		old_state == ST21NFC_IDLE) {
		st21nfc_dev->pw_states_err.idle_to_active_rw_ntf++;
	}

	dev_dbg(&st21nfc_dev->client->dev,
		"%s Switching from %s to %s: %llx, ntf=%d\n", __func__,
		st21nfc_power_state_name[old_state],
		st21nfc_power_state_name[new_state], current_time_ms, is_ntf);
	st21nfc_dev->pw_states[old_state].last_exit = current_time_ms;
	st21nfc_dev->pw_states[old_state].duration +=
		st21nfc_dev->pw_states[old_state].last_exit -
		st21nfc_dev->pw_states[old_state].last_entry;
	st21nfc_dev->pw_states[new_state].count++;
	st21nfc_dev->pw_current = new_state;
	st21nfc_dev->pw_states[new_state].last_entry = current_time_ms;

	mutex_unlock(&st21nfc_dev->pidle_mutex);
}

static void st21nfc_power_stats_idle_signal(struct st21nfc_device *st21nfc_dev)
{
	uint64_t current_time_ms = ktime_to_ms(ktime_get_boottime());
	bool is_active = (bool) gpiod_get_value(st21nfc_dev->gpiod_pidle);
	is_active = st21nfc_dev->pidle_active_low ? !is_active : is_active;

	st21nfc_power_stats_switch(st21nfc_dev, current_time_ms,
		st21nfc_dev->pw_current, is_active ? ST21NFC_ACTIVE : ST21NFC_IDLE,
		false);
}

static void st21nfc_pstate_wq(struct work_struct *work)
{
	struct st21nfc_device *st21nfc_dev = container_of(work,
							struct st21nfc_device,
							st_p_work);

	st21nfc_power_stats_idle_signal(st21nfc_dev);
}

static irqreturn_t st21nfc_dev_power_stats_handler(int irq, void *dev_id)
{
	struct st21nfc_device *st21nfc_dev = dev_id;

	queue_work(st21nfc_dev->st_p_wq, &(st21nfc_dev->st_p_work));

	return IRQ_HANDLED;
}

static void st21nfc_power_stats_filter(
	struct st21nfc_device *st21nfc_dev, char *buf, size_t count)
{
	uint64_t current_time_ms = ktime_to_ms(ktime_get_boottime());
	__u16 ntf_opcode = nci_opcode(buf);

	if (IS_ERR(st21nfc_dev->gpiod_pidle))
		return;

	/* In order to avoid counting active state on PAYLOAD where it would
	 * match a possible header, power states are filtered only on NCI
	 * headers.
	 */
	if (st21nfc_dev->r_state_current != ST21NFC_HEADER)
		return;

	if (count != HEADER_LENGTH) {
		dev_err(&st21nfc_dev->client->dev,
			"Warning: expect previous one was idle data\n");
		st21nfc_dev->pw_states_err.header_payload++;
		return;
	}

	if (nci_mt(buf) != NCI_MT_NTF_PKT
		&& nci_opcode_gid(ntf_opcode) != NCI_GID_PROPRIETARY)
		return;

	switch (ntf_opcode) {
	case PROP_PWR_MON_RW_OFF_NTF:
		st21nfc_power_stats_switch(st21nfc_dev, current_time_ms,
			st21nfc_dev->pw_current, ST21NFC_ACTIVE, true);
		break;
	case PROP_PWR_MON_RW_ON_NTF:
		st21nfc_power_stats_switch(st21nfc_dev, current_time_ms,
			st21nfc_dev->pw_current, ST21NFC_ACTIVE_RW, true);
		break;
	default:
		return;
	}
	return;
}

static ssize_t st21nfc_dev_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offset)
{
	struct st21nfc_device *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_device,
						       st21nfc_device);
	int ret, idle = 0;

	if (count == 0)
		return 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	dev_dbg(&st21nfc_dev->client->dev, "%s : reading %zu bytes.\n",
		__func__, count);

	mutex_lock(&st21nfc_dev->read_mutex);

	/* Read data */
	ret = i2c_master_recv(st21nfc_dev->client, st21nfc_dev->buffer, count);
	if (ret < 0) {
		dev_err(&st21nfc_dev->client->dev,
			"%s: i2c_master_recv returned %d\n", __func__, ret);
		mutex_unlock(&st21nfc_dev->read_mutex);
		return ret;
	}
	if (st21nfc_dev->r_state_current == ST21NFC_HEADER) {
		/* Counting idle index */
		for (idle = 0;
		     idle < ret && st21nfc_dev->buffer[idle] == IDLE_CHARACTER;
		     idle++)
			;

		if (idle > 0 && idle < HEADER_LENGTH) {
			memmove(st21nfc_dev->buffer,
				st21nfc_dev->buffer + idle, ret - idle);
			ret = i2c_master_recv(st21nfc_dev->client,
					      st21nfc_dev->buffer + ret - idle,
					      idle);
			if (ret < 0) {
				dev_err(&st21nfc_dev->client->dev,
					"%s: i2c_master_recv returned %d\n",
					__func__, ret);
				mutex_unlock(&st21nfc_dev->read_mutex);
				return ret;
			}
			ret = count;
		}
	}
	mutex_unlock(&st21nfc_dev->read_mutex);

	if (ret < 0) {
		dev_err(&st21nfc_dev->client->dev,
			"%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		dev_err(&st21nfc_dev->client->dev,
			"%s: received too many bytes from i2c (%d)\n", __func__,
			ret);
		return -EIO;
	}

	if (idle < HEADER_LENGTH) {
		st21nfc_power_stats_filter(st21nfc_dev, st21nfc_dev->buffer,
					   ret);
		/* change state only if a payload is detected, i.e. size > 0*/
		if ((st21nfc_dev->r_state_current == ST21NFC_HEADER) &&
			(st21nfc_dev->buffer[2] > 0)) {
			st21nfc_dev->r_state_current = ST21NFC_PAYLOAD;
			dev_dbg(&st21nfc_dev->client->dev,
				"%s : new state = ST21NFC_PAYLOAD\n", __func__);
		} else {
			st21nfc_dev->r_state_current = ST21NFC_HEADER;
			dev_dbg(&st21nfc_dev->client->dev,
				"%s : new state = ST21NFC_HEADER\n", __func__);
		}
	}

	if (copy_to_user(buf, st21nfc_dev->buffer, ret)) {
		dev_warn(&st21nfc_dev->client->dev,
			 "%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	return ret;
}

static ssize_t st21nfc_dev_write(struct file *filp, const char __user *buf,
				 size_t count, loff_t *offset)
{
	struct st21nfc_device *st21nfc_dev = container_of(filp->private_data,
				   struct st21nfc_device, st21nfc_device);
	char *tmp = NULL;
	int ret = count;

	dev_dbg(&st21nfc_dev->client->dev, "%s: st21nfc_dev ptr %p\n", __func__,
		st21nfc_dev);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp)) {
		dev_err(&st21nfc_dev->client->dev, "%s : memdup_user failed\n",
			__func__);
		return -EFAULT;
	}

	dev_dbg(&st21nfc_dev->client->dev, "%s : writing %zu bytes.\n",
		__func__, count);
	/* Write data */
	ret = i2c_master_send(st21nfc_dev->client, tmp, count);
	if (ret != count) {
		dev_err(&st21nfc_dev->client->dev,
			"%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	kfree(tmp);

	return ret;
}

static int st21nfc_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct st21nfc_device *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_device,
						       st21nfc_device);

	if (st21nfc_dev->device_open) {
		ret = -EBUSY;
	} else {
		st21nfc_dev->device_open = true;
		if (st21nfc_dev->clk_pad)
			st21nfc_exynos_clk_control(st21nfc_dev, true);
	}
	return ret;
}


static int st21nfc_release(struct inode *inode, struct file *file)
{
	struct st21nfc_device *st21nfc_dev = container_of(file->private_data,
						       struct st21nfc_device,
						       st21nfc_device);

	st21nfc_dev->device_open = false;
	if (st21nfc_dev->clk_pad) {
		st21nfc_exynos_clk_control(st21nfc_dev, false);
	}
	return 0;
}

static long st21nfc_dev_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	struct st21nfc_device *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_device,
						       st21nfc_device);

	struct i2c_client *client = st21nfc_dev->client;
	struct device *dev = &client->dev;
	int ret = 0;

	switch (cmd) {

	case ST21NFC_SET_POLARITY_RISING:
		dev_info(dev, " ### ST21NFC_SET_POLARITY_RISING ###");
		st21nfc_loc_set_polaritymode(st21nfc_dev, IRQF_TRIGGER_RISING);
		break;

	case ST21NFC_SET_POLARITY_HIGH:
		dev_info(dev, " ### ST21NFC_SET_POLARITY_HIGH ###");
		st21nfc_loc_set_polaritymode(st21nfc_dev, IRQF_TRIGGER_HIGH);
		break;

	case ST21NFC_PULSE_RESET:
		/* Double pulse is done to exit Quick boot mode.*/
		if (!IS_ERR(st21nfc_dev->gpiod_reset)) {
			/* pulse low for 20 millisecs */
			gpiod_set_value(st21nfc_dev->gpiod_reset, 0);
			msleep(20);
			gpiod_set_value(st21nfc_dev->gpiod_reset, 1);
			usleep_range(10000, 11000);
			/* pulse low for 20 millisecs */
			gpiod_set_value(st21nfc_dev->gpiod_reset, 0);
			msleep(20);
			gpiod_set_value(st21nfc_dev->gpiod_reset, 1);
		}
		st21nfc_dev->r_state_current = ST21NFC_HEADER;
		break;

	case ST21NFC_GET_WAKEUP:
		/* deliver state of Wake_up_pin as return value of ioctl */
		ret = gpiod_get_value(st21nfc_dev->gpiod_irq);
		/*
		 * Warning: depending on gpiod_get_value implementation,
		 * it can returns a value different than 1 in case of high level
		 */
		if (ret != 0)
			ret = 1;

		dev_dbg(&st21nfc_dev->client->dev, "%s get gpio result %d\n",
			__func__, ret);
		break;
	case ST21NFC_GET_POLARITY:
		ret = st21nfc_dev->polarity_mode;
		dev_dbg(&st21nfc_dev->client->dev, "%s get polarity %d\n",
			__func__, ret);
		break;
	case ST21NFC_RECOVERY:
		/* For ST21NFCD usage only */
		dev_info(dev, "%s Recovery Request\n", __func__);
		if (!IS_ERR(st21nfc_dev->gpiod_reset)) {
			/* pulse low for 20 millisecs */
			gpiod_set_value(st21nfc_dev->gpiod_reset, 0);
			usleep_range(10000, 11000);
			if (st21nfc_dev->irq_is_attached) {
				devm_free_irq(dev, client->irq, st21nfc_dev);
				st21nfc_dev->irq_is_attached = false;
			}
			/* During the reset, force IRQ OUT as */
			/* DH output instead of input in normal usage */
			ret = gpiod_direction_output(st21nfc_dev->gpiod_irq, 1);
			if (ret) {
				dev_err(&st21nfc_dev->client->dev,
					"%s : gpiod_direction_output failed\n",
					__func__);
				ret = -ENODEV;
				break;
			}

			gpiod_set_value(st21nfc_dev->gpiod_irq, 1);
			usleep_range(10000, 11000);
			gpiod_set_value(st21nfc_dev->gpiod_reset, 1);

			dev_info(dev, "%s done Pulse Request\n", __func__);
		}
		msleep(20);
		gpiod_set_value(st21nfc_dev->gpiod_irq, 0);
		msleep(20);
		gpiod_set_value(st21nfc_dev->gpiod_irq, 1);
		msleep(20);
		gpiod_set_value(st21nfc_dev->gpiod_irq, 0);
		msleep(20);
		dev_info(dev, "%s Recovery procedure finished\n", __func__);
		ret = gpiod_direction_input(st21nfc_dev->gpiod_irq);
		if (ret) {
			dev_err(&st21nfc_dev->client->dev,
				"%s : gpiod_direction_input failed\n",
				__func__);
			ret = -ENODEV;
		}
		break;
	case ST21NFC_CLK_ENABLE:
		st21nfc_exynos_clk_control(st21nfc_dev, true);
		break;
	case ST21NFC_CLK_DISABLE:
		st21nfc_exynos_clk_control(st21nfc_dev, false);
		break;
	case ST21NFC_CLK_STATE:
		if (st21nfc_dev->clk_pad == 0 ||
			exynos_pmu_read(st21nfc_dev->clk_pad, &ret) < 0) {
			ret = -ENODEV;
		} else {
			ret &= EXYNOS_CLK_MASK;
		}
		break;
	default:
		dev_err(&st21nfc_dev->client->dev, "%s bad ioctl %u\n",
			__func__, cmd);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static unsigned int st21nfc_poll(struct file *file, poll_table *wait)
{
	struct st21nfc_device *st21nfc_dev = container_of(file->private_data,
						       struct st21nfc_device,
						       st21nfc_device);
	unsigned int mask = 0;
	int pinlev = 0;

	/* wait for Wake_up_pin == high  */
	poll_wait(file, &st21nfc_dev->read_wq, wait);

	pinlev = gpiod_get_value(st21nfc_dev->gpiod_irq);

	if (pinlev != 0) {
		dev_dbg(&st21nfc_dev->client->dev, "%s return ready\n",
			__func__);
		mask = POLLIN | POLLRDNORM;	/* signal data avail */
		st21nfc_disable_irq(st21nfc_dev);
	} else {
		/* Wake_up_pin is low. Activate ISR  */
		if (!st21nfc_dev->irq_enabled) {
			dev_dbg(&st21nfc_dev->client->dev, "%s enable irq\n",
				__func__);
			st21nfc_enable_irq(st21nfc_dev);
		} else {
			dev_dbg(&st21nfc_dev->client->dev,
				"%s irq already enabled\n", __func__);
		}
	}
	return mask;
}

static const struct file_operations st21nfc_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = st21nfc_dev_read,
	.write = st21nfc_dev_write,
	.open = st21nfc_dev_open,
	.poll = st21nfc_poll,
	.release = st21nfc_release,

	.unlocked_ioctl = st21nfc_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = st21nfc_dev_ioctl
#endif
};

static ssize_t i2c_addr_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client != NULL)
		return scnprintf(buf, PAGE_SIZE, "0x%.2x\n", client->addr);
	return -ENODEV;
}

static ssize_t i2c_addr_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct st21nfc_device *data = dev_get_drvdata(dev);
	long new_addr = 0;

	if (data != NULL && data->client != NULL) {
		if (!kstrtol(buf, 10, &new_addr)) {
			mutex_lock(&data->read_mutex);
			data->client->addr = new_addr;
			mutex_unlock(&data->read_mutex);
			return count;
		}
		return -EINVAL;
	}
	return 0;
}

static ssize_t version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", DRIVER_VERSION);
}

static uint64_t st21nfc_power_duration(struct st21nfc_device *data,
				       enum st21nfc_power_state pstate,
				       uint64_t current_time_ms)
{

	return data->c_pw_current != pstate ?
		data->c_pw_states[pstate].duration :
		data->c_pw_states[pstate].duration +
		(current_time_ms - data->c_pw_states[pstate].last_entry);
}

static ssize_t power_stats_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct st21nfc_device *data = dev_get_drvdata(dev);
	uint64_t current_time_ms;
	uint64_t idle_duration;
	uint64_t active_ce_duration;
	uint64_t active_rw_duration;

	mutex_lock(&data->pidle_mutex);

	data->c_pw_current = data->pw_current;
	data->c_pw_states_err = data->pw_states_err;
	memcpy(data->c_pw_states, data->pw_states,
	       ST21NFC_POWER_STATE_MAX * sizeof(struct nfc_sub_power_stats));

	mutex_unlock(&data->pidle_mutex);

	current_time_ms = ktime_to_ms(ktime_get_boottime());
	idle_duration = st21nfc_power_duration(data, ST21NFC_IDLE,
					       current_time_ms);
	active_ce_duration = st21nfc_power_duration(data, ST21NFC_ACTIVE,
						    current_time_ms);
	active_rw_duration = st21nfc_power_duration(data, ST21NFC_ACTIVE_RW,
						    current_time_ms);

	return scnprintf(buf, PAGE_SIZE,
		"NFC subsystem\n"
		"Idle mode:\n"
		"\tCumulative count: 0x%llx\n"
		"\tCumulative duration msec: 0x%llx\n"
		"\tLast entry timestamp msec: 0x%llx\n"
		"\tLast exit timestamp msec: 0x%llx\n"
		"Active mode:\n"
		"\tCumulative count: 0x%llx\n"
		"\tCumulative duration msec: 0x%llx\n"
		"\tLast entry timestamp msec: 0x%llx\n"
		"\tLast exit timestamp msec: 0x%llx\n"
		"Active Reader/Writer mode:\n"
		"\tCumulative count: 0x%llx\n"
		"\tCumulative duration msec: 0x%llx\n"
		"\tLast entry timestamp msec: 0x%llx\n"
		"\tLast exit timestamp msec: 0x%llx\n"
		"\nError transition header --> payload state machine: 0x%llx\n"
		"Error transition from an Active state when not in Idle state: 0x%llx\n"
		"Error transition from Idle state to Idle state: 0x%llx\n"
		"Warning transition from Active Reader/Writer state to Idle state: 0x%llx\n"
		"Error transition from Active state to Active state: 0x%llx\n"
		"Error transition from Idle state to Active state with notification: 0x%llx\n"
		"Error transition from Active Reader/Writer state to Active Reader/Writer state: 0x%llx\n"
		"Error transition from Idle state to Active Reader/Writer state with notification: 0x%llx\n"
		"\nTotal uptime: 0x%llx Cumulative modes time: 0x%llx\n",
		data->c_pw_states[ST21NFC_IDLE].count,
		idle_duration,
		data->c_pw_states[ST21NFC_IDLE].last_entry,
		data->c_pw_states[ST21NFC_IDLE].last_exit,
		data->c_pw_states[ST21NFC_ACTIVE].count,
		active_ce_duration,
		data->c_pw_states[ST21NFC_ACTIVE].last_entry,
		data->c_pw_states[ST21NFC_ACTIVE].last_exit,
		data->c_pw_states[ST21NFC_ACTIVE_RW].count,
		active_rw_duration,
		data->c_pw_states[ST21NFC_ACTIVE_RW].last_entry,
		data->c_pw_states[ST21NFC_ACTIVE_RW].last_exit,
		data->c_pw_states_err.header_payload,
		data->c_pw_states_err.active_not_idle,
		data->c_pw_states_err.idle_to_idle,
		data->c_pw_states_err.active_rw_to_idle,
		data->c_pw_states_err.active_to_active,
		data->c_pw_states_err.idle_to_active_ntf,
		data->c_pw_states_err.act_rw_to_act_rw,
		data->c_pw_states_err.idle_to_active_rw_ntf,
		current_time_ms,
		idle_duration + active_ce_duration + active_rw_duration);
}

static DEVICE_ATTR_RW(i2c_addr);

static DEVICE_ATTR_RO(version);

static DEVICE_ATTR_RO(power_stats);

static struct attribute *st21nfc_attrs[] = {
	&dev_attr_i2c_addr.attr,
	&dev_attr_version.attr,
	NULL,
};

static struct attribute_group st21nfc_attr_grp = {
	.attrs = st21nfc_attrs,
};

static const struct acpi_gpio_params irq_gpios = {0, 0, false };
static const struct acpi_gpio_params reset_gpios = {1, 0, false };
static const struct acpi_gpio_params pidle_gpios = {2, 0, false};
static const struct acpi_gpio_params clkreq_gpios = {3, 0, false};

static const struct acpi_gpio_mapping acpi_st21nfc_gpios[] = {
	{ "irq-gpios", &irq_gpios, 1},
	{ "reset-gpios", &reset_gpios, 1},
	{ "pidle-gpios", &pidle_gpios, 1},
	{ "clkreq-gpios", &clkreq_gpios, 1},
};

static int st21nfc_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct st21nfc_device *st21nfc_dev;
	struct device *dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "%s : need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	st21nfc_dev = devm_kzalloc(dev, sizeof(*st21nfc_dev), GFP_KERNEL);
	if (st21nfc_dev == NULL)
		return -ENOMEM;

	/* store for later use */
	st21nfc_dev->client = client;
	st21nfc_dev->r_state_current = ST21NFC_HEADER;
	client->adapter->retries = 1;

	ret = acpi_dev_add_driver_gpios(ACPI_COMPANION(dev),
					acpi_st21nfc_gpios);
	if (ret)
		dev_dbg(dev, "Unable to add GPIO mapping table\n");

	st21nfc_dev->gpiod_irq = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(st21nfc_dev->gpiod_irq)) {
		dev_err(dev, "%s : Unable to request irq-gpios\n", __func__);
		return -ENODEV;
	}

	st21nfc_dev->gpiod_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(st21nfc_dev->gpiod_reset)) {
		dev_warn(dev, "%s : Unable to request reset-gpios\n", __func__);
		return -ENODEV;
	}

	st21nfc_dev->gpiod_pidle = devm_gpiod_get(dev, "pidle", GPIOD_IN);
	if (IS_ERR(st21nfc_dev->gpiod_pidle)) {
		ret = 0;
	} else {
		if (!device_property_read_bool(dev, "st,pidle_active_low")) {
			dev_dbg(dev, "[dsc]%s:[OPTIONAL] pidle_active_low not set\n", __func__);
			st21nfc_dev->pidle_active_low = false;
		} else {
			dev_dbg(dev, "[dsc]%s:[OPTIONAL] pidle_active_low set\n", __func__);
			st21nfc_dev->pidle_active_low = true;
		}
		/* Prepare a workqueue for st21nfc_dev_power_stats_handler */
		st21nfc_dev->st_p_wq = create_workqueue("st_pstate_work");
		if(!st21nfc_dev->st_p_wq)
			return -ENODEV;
		mutex_init(&st21nfc_dev->pidle_mutex);
		INIT_WORK(&(st21nfc_dev->st_p_work), st21nfc_pstate_wq);
		/* Start the power stat in power mode idle */
		st21nfc_dev->irq_pw_stats_idle =
					gpiod_to_irq(st21nfc_dev->gpiod_pidle);

		ret = irq_set_irq_type(st21nfc_dev->irq_pw_stats_idle,
				       IRQ_TYPE_EDGE_BOTH);
		if (ret) {
			dev_err(dev, "%s : set_irq_type failed\n", __func__);
			goto err_pidle_workqueue;
		}

		/* This next call requests an interrupt line */
		ret = devm_request_irq(dev, st21nfc_dev->irq_pw_stats_idle,
				(irq_handler_t)st21nfc_dev_power_stats_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				/* Interrupt on both edges */
				"st21nfc_pw_stats_idle_handle",
				st21nfc_dev);
		if (ret) {
			dev_err(dev,
				"%s : devm_request_irq for power stats idle failed\n",
				__func__);
			goto err_pidle_workqueue;
		}

		ret = sysfs_create_file(&dev->kobj,
					&dev_attr_power_stats.attr);
		if (ret) {
			dev_err(dev,
				"%s : sysfs_create_file for power stats failed\n",
				__func__);
			goto err_pidle_workqueue;
		}
	}

	st21nfc_dev->gpiod_clkreq = devm_gpiod_get(dev, "clkreq", GPIOD_IN);
	if (IS_ERR(st21nfc_dev->gpiod_clkreq)) {
		ret = 0;
	} else {
		if (!device_property_read_bool(dev, "st,clk_pinctrl")) {
			dev_dbg(dev, "[dsc]%s:[OPTIONAL] clk_pinctrl not set\n",
				__func__);
			st21nfc_dev->pinctrl_en = 0;
		} else {
			dev_dbg(dev, "[dsc]%s:[OPTIONAL] clk_pinctrl set\n",
				__func__);
			st21nfc_dev->pinctrl_en = 1;

			/* handle clk_req irq */
			st21nfc_dev->irq_clkreq =
					gpiod_to_irq(st21nfc_dev->gpiod_clkreq);

			ret = irq_set_irq_type(st21nfc_dev->irq_clkreq,
				       IRQ_TYPE_EDGE_BOTH);
			if (ret) {
				dev_err(dev, "%s : set_irq_type failed\n",
					__func__);
				st21nfc_dev->pinctrl_en = 0;
			} else {
				ret = devm_request_irq(dev,
						st21nfc_dev->irq_clkreq,
						st21nfc_clkreq_irq_handler,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						"st21nfc_clkreq_handle",
						st21nfc_dev);
				if (ret) {
					dev_err(dev,
						"%s : devm_request_irq for clkreq irq failed\n",
						__func__);
					st21nfc_dev->pinctrl_en = 0;
				}
			}
		}

		/* Get clk_pad value*/
		if (device_property_read_u32(dev, "pmu_clk_pad", &st21nfc_dev->clk_pad)) {
			dev_err(dev, "%s : PMU_CLKOUT_PAD offset is unset\n", __func__);
			st21nfc_dev->clk_pad = 0;
			st21nfc_dev->pinctrl_en = 0;
		}

		ret = st21nfc_clock_select(st21nfc_dev);
		if (ret < 0) {
			dev_err(dev, "%s : st21nfc_clock_select failed\n", __func__);
			goto err_sysfs_power_stats;
		}
	}

	client->irq = gpiod_to_irq(st21nfc_dev->gpiod_irq);

	/* init mutex and queues */
	init_waitqueue_head(&st21nfc_dev->read_wq);
	mutex_init(&st21nfc_dev->read_mutex);
	spin_lock_init(&st21nfc_dev->irq_enabled_lock);
	dev_dbg(dev, "%s : debug irq_gpio = %d, client-irq =  %d\n", __func__,
		desc_to_gpio(st21nfc_dev->gpiod_irq), client->irq);
	if (!IS_ERR(st21nfc_dev->gpiod_pidle)) {
		dev_dbg(dev, "%s : pidle_gpio = %d\n", __func__,
			desc_to_gpio(st21nfc_dev->gpiod_pidle));
	}
	if (!IS_ERR(st21nfc_dev->gpiod_clkreq)) {
		dev_dbg(dev, "%s : clkreq_gpio = %d\n", __func__,
			desc_to_gpio(st21nfc_dev->gpiod_clkreq));
	}
	st21nfc_dev->st21nfc_device.minor = MISC_DYNAMIC_MINOR;
	st21nfc_dev->st21nfc_device.name = "st21nfc";
	st21nfc_dev->st21nfc_device.fops = &st21nfc_dev_fops;
	st21nfc_dev->st21nfc_device.parent = dev;

	i2c_set_clientdata(client, st21nfc_dev);
	ret = misc_register(&st21nfc_dev->st21nfc_device);
	if (ret) {
		dev_err(dev, "%s : misc_register failed\n", __func__);
		goto err_misc_register;
	}

	ret = sysfs_create_group(&dev->kobj, &st21nfc_attr_grp);
	if (ret) {
		dev_err(dev, "%s : sysfs_create_group failed\n", __func__);
		goto err_sysfs_create_group_failed;
	}
	device_init_wakeup(&client->dev, true);
	device_set_wakeup_capable(&client->dev, true);
	st21nfc_dev->irq_wake_up = false;

	return 0;

err_sysfs_create_group_failed:
	misc_deregister(&st21nfc_dev->st21nfc_device);
err_misc_register:
	mutex_destroy(&st21nfc_dev->read_mutex);
err_sysfs_power_stats:
	if (!IS_ERR(st21nfc_dev->gpiod_pidle)) {
		sysfs_remove_file(&client->dev.kobj,
				  &dev_attr_power_stats.attr);
	}
err_pidle_workqueue:
	if (!IS_ERR(st21nfc_dev->gpiod_pidle)) {
		mutex_destroy(&st21nfc_dev->pidle_mutex);
		destroy_workqueue(st21nfc_dev->st_p_wq);
	}
	return ret;
}

static int st21nfc_remove(struct i2c_client *client)
{
	struct st21nfc_device *st21nfc_dev = i2c_get_clientdata(client);

	st21nfc_clock_deselect(st21nfc_dev);
	misc_deregister(&st21nfc_dev->st21nfc_device);
	if (!IS_ERR(st21nfc_dev->gpiod_pidle)) {
		sysfs_remove_file(&client->dev.kobj,
				  &dev_attr_power_stats.attr);
		mutex_destroy(&st21nfc_dev->pidle_mutex);
	}
	sysfs_remove_group(&client->dev.kobj, &st21nfc_attr_grp);
	mutex_destroy(&st21nfc_dev->read_mutex);
	acpi_dev_remove_driver_gpios(ACPI_COMPANION(&client->dev));

	return 0;
}

static int st21nfc_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct st21nfc_device *st21nfc_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) && st21nfc_dev->irq_enabled) {
		if (!enable_irq_wake(client->irq))
			st21nfc_dev->irq_wake_up = true;
	}

	return 0;
}

static int st21nfc_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct st21nfc_device *st21nfc_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) && st21nfc_dev->irq_wake_up) {
		if (!disable_irq_wake(client->irq))
			st21nfc_dev->irq_wake_up = false;
	}

	if (!IS_ERR(st21nfc_dev->gpiod_pidle)) {
		bool is_active = (bool) gpiod_get_value(st21nfc_dev->gpiod_pidle);
		is_active = st21nfc_dev->pidle_active_low ? !is_active : is_active;
		if((st21nfc_dev->pw_current == ST21NFC_IDLE && is_active) ||
		   (st21nfc_dev->pw_current == ST21NFC_ACTIVE && !is_active)) {
			queue_work(st21nfc_dev->st_p_wq,
				   &(st21nfc_dev->st_p_work));
		}
	}
	return 0;
}


static const struct i2c_device_id st21nfc_id[] = {
	{"st21nfc", 0},
	{}
};

static const struct of_device_id st21nfc_of_match[] = {
	{ .compatible = "st,st21nfc", },
	{}
};
MODULE_DEVICE_TABLE(of, st21nfc_of_match);

static const struct dev_pm_ops st21nfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st21nfc_suspend, st21nfc_resume)
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id st21nfc_acpi_match[] = {
	{"SMO2104"},
	{}
};
MODULE_DEVICE_TABLE(acpi, st21nfc_acpi_match);
#endif

static struct i2c_driver st21nfc_driver = {
	.id_table = st21nfc_id,
	.driver = {
		.name	= "st21nfc",
		.owner	= THIS_MODULE,
		.of_match_table	= st21nfc_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm = &st21nfc_pm_ops,
		.acpi_match_table = ACPI_PTR(st21nfc_acpi_match),
	},
	.probe		= st21nfc_probe,
	.remove		= st21nfc_remove,
};

#ifdef GKI_MODULE
module_i2c_driver(st21nfc_driver);
#else
/*
 * module load/unload record keeping
 */

static int __init st21nfc_dev_init(void)
{
	pr_info("%s: Loading st21nfc driver (version %s)\n",
		__func__, DRIVER_VERSION);
	return i2c_add_driver(&st21nfc_driver);
}

module_init(st21nfc_dev_init);

static void __exit st21nfc_dev_exit(void)
{
	pr_debug("Unloading st21nfc driver\n");
	i2c_del_driver(&st21nfc_driver);
}

module_exit(st21nfc_dev_exit);
#endif

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("NFC ST21NFC driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
