// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (C) 2024 Google, Inc.
 */

//#define DEBUG
#define pr_fmt(fmt) "fth:%s: " fmt, __func__
#include <linux/input.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/of_gpio.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/input.h>
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
#include <goog_touch_interface.h>
#include <linux/notifier.h>
#endif
#include "fps_touch_handler.h"

#define FTH_DEV "fth"
#define MINOR_NUM_FD 0
#define FTH_INPUT_DEV_NAME "fth_key_input"
#define FTH_INPUT_DEV_VERSION 0x0100

struct touch_event {
	int X;
	int Y;
	int major;
	int minor;
	int orientation;
	/**
	 * id = -1 when finger is lifted, otherwise
	 *  id = x, where x is the number of previous
	 *  finger ups.
	 */
	int id;
	ktime_t ktime_mono;
	bool updated;
};

struct finger_detect_touch {
	struct fth_touch_config_v6 config;
	struct fth_touch_config_v6 up_config;
	struct work_struct work;
	struct touch_event current_events[FTH_MAX_FINGERS];
	struct touch_event last_events[FTH_MAX_FINGERS];
	int delta_X[FTH_MAX_FINGERS];
	int delta_Y[FTH_MAX_FINGERS];
	bool is_finger_in[FTH_MAX_FINGERS];
	int current_slot;
};

struct fth_drvdata {
	struct class	*fth_class;
	struct cdev	fth_fd_cdev;
	struct input_dev	*in_dev;
	struct input_dev	*input_touch_dev;
	struct device	*dev;
	char		*fth_node;
	atomic_t	fd_available;
	atomic_t	ipc_available;
	struct mutex	mutex;
	struct mutex	fd_events_mutex;
	struct finger_detect_touch fd_touch;
	uint32_t current_slot_state[FTH_MAX_FINGERS];
	DECLARE_KFIFO(fd_events, struct fth_touch_event_v6, FTH_MAX_FD_EVENTS);
	wait_queue_head_t read_wait_queue_fd;
	struct fth_fd_buf scrath_buf;
	atomic_t wakelock_acquired;
	bool lptw_event_report_enabled;
};

static void fth_fd_report_event(struct fth_drvdata *drvdata,
		struct fth_touch_event_v6 *event)
{
	if (!drvdata || !event) {
		pr_err("NULL ptr passed\n");
		return;
	}
	mutex_lock(&drvdata->fd_events_mutex);
	if (!kfifo_put(&drvdata->fd_events, *event)) {
		pr_err("FD events fifo: error adding item\n");
	} else {
		pr_debug("FD event %d at slot %d queued at time %lu uS\n",
				event->state, event->slot,
				(unsigned long)ktime_to_us(ktime_get()));
		pr_debug("FD event: x:%d, y:%d, major:%d, minor:%d, "
				"orientation:%d, time_us:%lld\n",
				event->X[event->slot], event->Y[event->slot],
				event->major, event->minor,
				event->orientation, event->time_us);
	}
	mutex_unlock(&drvdata->fd_events_mutex);
	wake_up_interruptible(&drvdata->read_wait_queue_fd);
}

static int fth_touch_connect(struct input_handler *handler,
	struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	struct fth_drvdata *drvdata;
	int ret;

	/* Only coonect with the built-in touchscreen device. */
	if (!(dev->uniq && strncmp(dev->uniq, "google_touchscreen", 18) == 0)) {
		pr_info("Skip connecting device(name:'%s', uniq:'%s')\n",
			dev->name ? dev->name : "",
			dev->uniq ? dev->uniq : "");
		return 0;
	}

	drvdata = handler->private;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "fth_touch";
	ret = input_register_handle(handle);
	if (ret) {
		pr_err("Failed to register to input handle: %d\n", ret);
		kfree(handle);
		return ret;
	}
	ret = input_open_device(handle);
	if (ret) {
		pr_err("Failed to open to input handle: %d\n", ret);
		input_unregister_handle(handle);
		kfree(handle);
		return ret;
	}

	drvdata->input_touch_dev = handle->dev;

	pr_info("Connected device: %s\n", dev_name(&dev->dev));
	return ret;
}

static void fth_touch_disconnect(struct input_handle *handle)
{
	struct fth_drvdata *drvdata = handle->handler->private;
	pr_info("Disconnected device: %s\n", dev_name(&handle->dev->dev));
	input_close_device(handle);
	input_unregister_handle(handle);
	if (handle->dev->uniq && strncmp(handle->dev->uniq, "google_touchscreen", 18) == 0) {
		if (drvdata->input_touch_dev)
			drvdata->input_touch_dev = NULL;
	}
	kfree(handle);
}

static void fth_touch_report_event(struct input_handle *handle,
	unsigned int type, unsigned int code, int value)
{
	struct fth_drvdata *drvdata = handle->handler->private;
	struct finger_detect_touch *fd_touch = &drvdata->fd_touch;
	struct touch_event *event = NULL;
	static bool report_event = true;

	if (!fd_touch->config.touch_fd_enable)
		return;

	if (type != EV_SYN && type != EV_ABS)
		return;

	if (fd_touch->current_slot >= FTH_MAX_FINGERS) {
		pr_warn("Touch event current slot: %d received out of bound\n",
			fd_touch->current_slot);
		return;
	}

	if (!drvdata->input_touch_dev) {
		pr_warn("input_touch_dev is NULL\n");
		return;
	}

	event = &fd_touch->current_events[fd_touch->current_slot];
	switch (code) {
	case ABS_MT_SLOT:
		fd_touch->current_slot = value;
		if (!report_event)
			event->updated = true;
		report_event = false;
		break;
	case ABS_MT_TRACKING_ID:
		event->id = value;
		report_event = false;
		break;
	case ABS_MT_POSITION_X:
		event->X = abs(value);
		report_event = false;
		break;
	case ABS_MT_POSITION_Y:
		event->Y = abs(value);
		report_event = false;
		break;
	case ABS_MT_TOUCH_MAJOR:
		event->major = value;
		report_event = false;
		break;
	case ABS_MT_TOUCH_MINOR:
		event->minor = value;
		report_event = false;
		break;
	case ABS_MT_ORIENTATION:
		event->orientation = value;
		report_event = false;
		break;
	case ABS_MT_PRESSURE:
		report_event = false;
		break;
	case SYN_REPORT:
		event->updated = true;
		report_event = true;
		break;
	default:
		break;
	}
	if (report_event) {
		if (!fd_touch->config.touch_fd_enable) {
			memcpy(fd_touch->last_events,
					fd_touch->current_events,
					FTH_MAX_FINGERS * sizeof(
					struct touch_event));
		} else {
			struct input_dev *dev = handle->dev;
			event->ktime_mono = dev->timestamp[INPUT_CLK_MONO];
			pm_stay_awake(drvdata->dev);
			schedule_work(&drvdata->fd_touch.work);
		}
	}
}

static const struct input_device_id fth_touch_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = {BIT_MASK(EV_ABS)},
	},
	{},
};
MODULE_DEVICE_TABLE(input, fth_touch_ids);

static struct input_handler fth_touch_handler = {
	.event = fth_touch_report_event,
	.connect = fth_touch_connect,
	.disconnect = fth_touch_disconnect,
	.name =	"fth_touch",
	.id_table = fth_touch_ids
};

static bool fth_touch_filter_aoi_region(struct touch_event *event,
		struct fth_touch_config_v6 *config)
{
	if ((event == NULL) || (config == NULL)) {
		return false;
	}
	if (event->X < config->left ||
			event->X > config->right ||
			event->Y < config->top ||
			event->Y > config->bottom)
		return false;
	else
		return true;
}

static bool fth_touch_filter_by_radius(
		struct fth_drvdata *drvdata,
		struct touch_event *current_event,
		struct touch_event *last_event,
		int slot)
{
	unsigned int del_X = 0, del_Y = 0;
	struct fth_touch_config_v6 *config = &drvdata->fd_touch.config;
	drvdata->fd_touch.delta_X[slot] +=
			current_event->X - last_event->X;
	drvdata->fd_touch.delta_Y[slot] +=
			current_event->Y - last_event->Y;
	del_X = abs(drvdata->fd_touch.delta_X[slot]);
	del_Y = abs(drvdata->fd_touch.delta_Y[slot]);
	if (!config->rad_filter_enable ||
			del_X > config->rad_x ||
			del_Y > config->rad_y) {
		drvdata->fd_touch.delta_X[slot] = 0;
		drvdata->fd_touch.delta_Y[slot] = 0;
		return true;
	} else
		return false;
}

static void fth_touch_work_func(struct work_struct *work)
{
	struct fth_drvdata *drvdata = NULL;
	struct fth_touch_config_v6 *config = NULL;
	struct fth_touch_config_v6 *large_config = NULL;
	struct finger_detect_touch *fd_touch = NULL;
	struct touch_event current_event, last_event;
	struct fth_touch_event_v6 finger_event = {0};
	bool in_small_aoi = false;
	bool in_large_aoi = false;
	int slot = 0;
	if (!work) {
		pr_err("NULL pointer passed\n");
		return;
	}
	drvdata = container_of(work, struct fth_drvdata, fd_touch.work);
	fd_touch = &drvdata->fd_touch;
	config = &fd_touch->config;
	large_config = &fd_touch->up_config;
	finger_event.touch_valid = true;
	for (slot = 0; slot < FTH_MAX_FINGERS; slot++) {
		if (fd_touch->current_events[slot].id >= 0) {
			finger_event.X[slot] = fd_touch->current_events[slot].X;
			finger_event.Y[slot] = fd_touch->current_events[slot].Y;
			finger_event.updated[slot] = true;
			finger_event.num_fingers++;
		}
	}
	for (slot = 0; slot < FTH_MAX_FINGERS; slot++) {
		bool *is_finger_in = &fd_touch->is_finger_in[slot];
		memcpy(&current_event, &fd_touch->current_events[slot],
				sizeof(current_event));
		fd_touch->current_events[slot].updated = false;
		if (!current_event.updated) {
			continue;
                }
		memcpy(&last_event, &fd_touch->last_events[slot],
				sizeof(last_event));
		memcpy(&fd_touch->last_events[slot], &current_event,
				sizeof(current_event));
		if (current_event.id < 0) {
			// -1 corresponds to finger being lifted.
			finger_event.state = FTH_EVENT_FINGER_UP;
		} else if ((last_event.id < 0) && (current_event.id >= 0)) {
			// The finger was previously up, and is now down.
			finger_event.state = FTH_EVENT_FINGER_DOWN;
		} else if (last_event.id == current_event.id){
			finger_event.state = FTH_EVENT_FINGER_MOVE;
		} else {
			// Somehow got incrementing ids with no -1 between.
			pr_warn("finger up got missed, reporting finger down\n");
			finger_event.state = FTH_EVENT_FINGER_DOWN;
		}
		// Do filtering to update state and only report events of interest.
		in_small_aoi = fth_touch_filter_aoi_region(&current_event, config);
		in_large_aoi = fth_touch_filter_aoi_region(&current_event, large_config);
		if (!(*is_finger_in)) {
			if (in_small_aoi && !(current_event.id < 0)) {
					finger_event.state = FTH_EVENT_FINGER_DOWN;
					*is_finger_in = true;
			} else {
				if (drvdata->lptw_event_report_enabled) {
					finger_event.state = FTH_EVENT_FINGER_MOVE;
				} else {
					// Don't report.
					continue;
				}
			}
		} else {
			// Need to update state if finger has left large AoI.
			if (current_event.id < 0) {
				*is_finger_in = false;
			} else if (!in_large_aoi) {
					finger_event.state = FTH_EVENT_FINGER_UP;
					*is_finger_in = false;
			}
			// Report event.
		}

		// Radius filtering on moves to limit report frequency.
		if (finger_event.state == FTH_EVENT_FINGER_MOVE &&
				!fth_touch_filter_by_radius(drvdata,
				&current_event,	&last_event, slot)) {
			// Don't report if not enough movement since last move.
			continue;
		}
		// Report touch event to HAL and update interrupts.
		finger_event.slot = slot;
		finger_event.major = current_event.major;
		finger_event.minor = current_event.minor;
		finger_event.orientation = current_event.orientation;
		finger_event.time_us = ktime_to_us(current_event.ktime_mono);
		if (finger_event.state != FTH_EVENT_FINGER_MOVE) {
			drvdata->current_slot_state[slot] = finger_event.state;
		}
		if (config->touch_fd_enable) {
			fth_fd_report_event(drvdata, &finger_event);
		}
	}
	pm_relax(drvdata->dev);
}

/**
 * fth_open() - Function called when user space opens device.
 * Successful if driver not currently open.
 * @inode:	ptr to inode object
 * @file:	ptr to file object
 *
 * Return: 0 on success. Error code on failure.
 */
static int fth_open(struct inode *inode, struct file *file)
{
	struct fth_drvdata *drvdata = NULL;
	int rc = 0;
	int minor_no = -1;
	if (!inode || !inode->i_cdev || !file) {
		pr_err("NULL pointer passed\n");
		return -EINVAL;
	}
	minor_no = iminor(inode);
	if (minor_no == MINOR_NUM_FD) {
		drvdata = container_of(inode->i_cdev,
				struct fth_drvdata, fth_fd_cdev);
	} else {
		pr_err("Invalid minor number\n");
		return -EINVAL;
	}
	file->private_data = drvdata;
	pr_debug("entry minor_no=%d fd_available=%d\n",
			minor_no, atomic_read(&drvdata->fd_available));
	/* disallowing concurrent opens */
	if (minor_no == MINOR_NUM_FD &&
			!atomic_dec_and_test(&drvdata->fd_available)) {
		atomic_inc(&drvdata->fd_available);
		rc = -EBUSY;
	}
	pr_debug("exit : %d  fd_available=%d\n",
			rc, atomic_read(&drvdata->fd_available));
	return rc;
}

/**
 * fth_release() - Function called when user space closes device.
 * @inode:	ptr to inode object
 * @file:	ptr to file object
 *
 * Return: 0 on success. Error code on failure.
 */
static int fth_release(struct inode *inode, struct file *file)
{
	struct fth_drvdata *drvdata;
	int minor_no = -1;
	if (!file || !file->private_data || !inode) {
		pr_err("NULL pointer passed\n");
		return -EINVAL;
	}
	drvdata = file->private_data;
	minor_no = iminor(inode);
	pr_debug("entry minor_no=%d fd_available=%d\n",
			minor_no, atomic_read(&drvdata->fd_available));
	if (minor_no == MINOR_NUM_FD) {
		atomic_inc(&drvdata->fd_available);
	} else {
		pr_err("Invalid minor number\n");
		return -EINVAL;
	}
	if (atomic_read(&drvdata->wakelock_acquired) != 0) {
		pr_debug("Releasing wakelock\n");
		pm_relax(drvdata->dev);
		atomic_set(&drvdata->wakelock_acquired, 0);
	}
	pr_debug("exit : fd_available=%d\n", atomic_read(&drvdata->fd_available));
	return 0;
}

/**
 * fth_ioctl() - Function called when user space calls ioctl.
 * @file:	struct file - not used
 * @cmd:	cmd identifier
 * @arg:	ptr to relevant structe: either fth_app or
 *		fth_send_tz_cmd depending on which cmd is passed
 *
 * Return: 0 on success. Error code on failure.
 */
static long fth_ioctl(
		struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	void __user *priv_arg = (void __user *)arg;
	struct fth_drvdata *drvdata;
	if (!file || !file->private_data) {
		pr_err("NULL pointer passed\n");
		return -EINVAL;
	}
	drvdata = file->private_data;
	if (IS_ERR(priv_arg)) {
		dev_err(drvdata->dev, "%s: invalid user space pointer %lu\n",
			__func__, arg);
		return -EINVAL;
	}
	mutex_lock(&drvdata->mutex);
	pr_debug("cmd received %d\n", cmd);
	switch (cmd) {
	case FTH_IOCTL_SEND_KEY_EVENT:
	{
		struct fth_key_event key_event;
		if (copy_from_user(&key_event, priv_arg,
			sizeof(key_event))
				!= 0) {
			rc = -EFAULT;
			pr_err("failed copy from user space %d\n", rc);
			goto end;
		}
		input_event(drvdata->in_dev, EV_KEY,
				key_event.key, key_event.value);
		input_sync(drvdata->in_dev);
		break;
	}
	case FTH_IOCTL_ENABLE_LPTW_EVENT_REPORT:
	{
		drvdata->lptw_event_report_enabled = true;
		pr_debug("lptw_event_report_enabled:%d\n",
				drvdata->lptw_event_report_enabled);
		break;
	}
	case FTH_IOCTL_DISABLE_LPTW_EVENT_REPORT:
	{
		drvdata->lptw_event_report_enabled = false;
		pr_debug("lptw_event_report_enabled:%d\n",
				drvdata->lptw_event_report_enabled);
		break;
	}
	case FTH_IOCTL_ACQUIRE_WAKELOCK:
	{
		if (atomic_read(&drvdata->wakelock_acquired) == 0) {
			pr_debug("Acquiring wakelock\n");
			pm_stay_awake(drvdata->dev);
		}
		atomic_inc(&drvdata->wakelock_acquired);
		break;
	}
	case FTH_IOCTL_RELEASE_WAKELOCK:
	{
		if (atomic_read(&drvdata->wakelock_acquired) == 0)
			break;
		if (atomic_dec_and_test(&drvdata->wakelock_acquired)) {
			pr_debug("Releasing wakelock\n");
			pm_relax(drvdata->dev);
		}
		break;
	}
	case FTH_IOCTL_GET_TOUCH_FD_VERSION:
	{
		struct fth_touch_fd_version version;
		version.version = FTH_TOUCH_FD_VERSION_6;
		rc = copy_to_user((void __user *)priv_arg,
				&version, sizeof(version));
		if (rc != 0) {
			pr_err("Failed to copy touch FD version: %d\n", rc);
			rc = -EFAULT;
			goto end;
		}
		break;
	}
	case FTH_IOCTL_CONFIGURE_TOUCH_FD_V6:
	{
		__s32 version;
		if (copy_from_user(&drvdata->fd_touch.config.version,
				priv_arg,
				sizeof(drvdata->fd_touch.config.version))
				!= 0) {
			rc = -EFAULT;
			pr_err("failed copy from user space %d\n", rc);
			goto end;
		}
		version = drvdata->fd_touch.config.version.version;
		if (version != FTH_TOUCH_FD_VERSION_6) {
			rc = -EINVAL;
			pr_err("unsupported version %d\n",
					drvdata->fd_touch.config.version.version);
			goto end;
		}
		if (copy_from_user(&drvdata->fd_touch.config,
				priv_arg,
				sizeof(drvdata->fd_touch.config)) != 0) {
			rc = -EFAULT;
			pr_err("failed copy from user space %d\n", rc);
			goto end;
		} else {
			// Succeeded in copying, double side length for up AoI.
			struct fth_touch_config_v6 *config = &drvdata->fd_touch.config;
			int width = config->right - config->left;
			int height = config->bottom - config->top;
			memcpy(&drvdata->fd_touch.up_config,
						 &drvdata->fd_touch.config,
						 sizeof(drvdata->fd_touch.config));
			drvdata->fd_touch.up_config.right += width/2;
			drvdata->fd_touch.up_config.left -= width/2;
			drvdata->fd_touch.up_config.top -= height/2;
			drvdata->fd_touch.up_config.bottom += height/2;
		}
		pr_info("Touch FD enable: %d\n",
			drvdata->fd_touch.config.touch_fd_enable);
		pr_info("left: %d right: %d top: %d bottom: %d\n",
			drvdata->fd_touch.config.left,
			drvdata->fd_touch.config.right,
			drvdata->fd_touch.config.top,
			drvdata->fd_touch.config.bottom);
		pr_info("Radius Filter enable: %d\n",
			drvdata->fd_touch.config.rad_filter_enable);
		pr_info("rad_x: %d rad_y: %d\n",
			drvdata->fd_touch.config.rad_x,
			drvdata->fd_touch.config.rad_y);
		pr_info("up_config: left: %d right: %d top: %d bottom: %d\n",
			drvdata->fd_touch.up_config.left,
			drvdata->fd_touch.up_config.right,
			drvdata->fd_touch.up_config.top,
			drvdata->fd_touch.up_config.bottom);
		break;
	}
	default:
		pr_err("invalid cmd %d\n", cmd);
		rc = -ENOIOCTLCMD;
		goto end;
	}
end:
	mutex_unlock(&drvdata->mutex);
	return rc;
}

static int get_events_fifo_len_locked(
		struct fth_drvdata *drvdata, int minor_no)
{
	int len = 0;
	if (minor_no == MINOR_NUM_FD) {
		mutex_lock(&drvdata->fd_events_mutex);
		len = kfifo_len(&drvdata->fd_events);
		mutex_unlock(&drvdata->fd_events_mutex);
	}
	return len;
}

static ssize_t fth_read(struct file *filp, char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	struct fth_touch_event_v6 *fd_evt;
	struct fth_drvdata *drvdata;
	struct fth_fd_buf *scratch_buf;
	wait_queue_head_t *read_wait_queue = NULL;
	int i = 0;
	int minor_no = -1;
	int fifo_len = 0;
	ssize_t num_bytes = 0;
	pr_debug("entry with numBytes = %zd, minor_no = %d\n", cnt, minor_no);
	if (!filp || !filp->private_data) {
		pr_err("NULL pointer passed\n");
		return -EINVAL;
	}
	drvdata = filp->private_data;
	minor_no = iminor(filp->f_path.dentry->d_inode);
	scratch_buf = &drvdata->scrath_buf;
	memset(scratch_buf, 0, sizeof(*scratch_buf));
	if (minor_no == MINOR_NUM_FD) {
		if (cnt < sizeof(*scratch_buf)) {
			pr_err("Num bytes to read is too small\n");
			return -EINVAL;
		}
		read_wait_queue = &drvdata->read_wait_queue_fd;
	} else {
		pr_err("Invalid minor number\n");
		return -EINVAL;
	}
	fifo_len = get_events_fifo_len_locked(drvdata, minor_no);
	while (fifo_len == 0) {
		if (filp->f_flags & O_NONBLOCK) {
			pr_debug("fw_events fifo: empty, returning\n");
			return -EAGAIN;
		}
		pr_debug("fw_events fifo: empty, waiting\n");
		if (wait_event_interruptible(*read_wait_queue,
				(get_events_fifo_len_locked(
				drvdata, minor_no) > 0)))
			return -ERESTARTSYS;
		fifo_len = get_events_fifo_len_locked(drvdata, minor_no);
	}
	if (minor_no == MINOR_NUM_FD) {
		mutex_lock(&drvdata->fd_events_mutex);
		scratch_buf->num_events = kfifo_len(&drvdata->fd_events);
		for (i = 0; i < scratch_buf->num_events; i++) {
			fd_evt = &scratch_buf->fd_events[i];
			if (!kfifo_get(&drvdata->fd_events, fd_evt)) {
				pr_err("FD event fifo: err popping item\n");
				scratch_buf->num_events = i;
				break;
			}
			pr_debug("Reading event slot:%d state:%d time:%lldus\n",
					fd_evt->slot, fd_evt->state,
					fd_evt->time_us);
		}
		pr_debug("%d FD events read at time %lu uS\n",
				scratch_buf->num_events,
				(unsigned long)ktime_to_us(ktime_get()));
		num_bytes = copy_to_user(ubuf, scratch_buf,
				sizeof(*scratch_buf));
		mutex_unlock(&drvdata->fd_events_mutex);
	} else {
		pr_err("Invalid minor number\n");
	}
	if (num_bytes != 0)
		pr_warn("Could not copy %ld bytes\n", num_bytes);
	return num_bytes;
}

static __poll_t fth_poll(struct file *filp,
	struct poll_table_struct *wait)
{
	struct fth_drvdata *drvdata;
	__poll_t mask = 0;
	int minor_no = -1;
	if (!filp || !filp->private_data) {
		pr_err("NULL pointer passed\n");
		return -EINVAL;
	}
	drvdata = filp->private_data;
	minor_no = iminor(filp->f_path.dentry->d_inode);
	if (minor_no == MINOR_NUM_FD) {
		poll_wait(filp, &drvdata->read_wait_queue_fd, wait);
		if (kfifo_len(&drvdata->fd_events) > 0)
			mask |= (POLLIN | POLLRDNORM);
	} else {
		pr_err("Invalid minor number\n");
		return -EINVAL;
	}
	return mask;
}

static const struct file_operations fth_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fth_ioctl,
	.open = fth_open,
	.release = fth_release,
	.read = fth_read,
	.poll = fth_poll
};

static int fth_dev_register(struct fth_drvdata *drvdata)
{
	dev_t dev_no, major_no;
	int ret = 0;
	size_t node_size;
	char *node_name = FTH_DEV;
	struct device *dev = drvdata->dev;
	struct device *device;
	node_size = strlen(node_name) + 1;
	drvdata->fth_node = devm_kzalloc(dev, node_size, GFP_KERNEL);
	if (!drvdata->fth_node) {
		ret = -ENOMEM;
		goto err_alloc;
	}
	strscpy(drvdata->fth_node, node_name, node_size);
	ret = alloc_chrdev_region(&dev_no, 0, 2, drvdata->fth_node);
	if (ret) {
		pr_err("alloc_chrdev_region failed %d\n", ret);
		goto err_alloc;
	}
	major_no = MAJOR(dev_no);
	cdev_init(&drvdata->fth_fd_cdev, &fth_fops);
	drvdata->fth_fd_cdev.owner = THIS_MODULE;
	ret = cdev_add(&drvdata->fth_fd_cdev,
			MKDEV(major_no, MINOR_NUM_FD), 1);
	if (ret) {
		pr_err("cdev_add failed for fd %d\n", ret);
		goto err_cdev_add;
	}
	drvdata->fth_class = class_create(THIS_MODULE, drvdata->fth_node);
	if (IS_ERR(drvdata->fth_class)) {
		ret = PTR_ERR(drvdata->fth_class);
		pr_err("class_create failed %d\n", ret);
		goto err_class_create;
	}
	device = device_create(drvdata->fth_class, NULL,
			drvdata->fth_fd_cdev.dev, drvdata,
			"%s_fd", drvdata->fth_node);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		pr_err("fd device_create failed %d\n", ret);
		goto err_dev_create;
	}
	return 0;
err_dev_create:
	class_destroy(drvdata->fth_class);
err_class_create:
	cdev_del(&drvdata->fth_fd_cdev);
err_cdev_add:
	unregister_chrdev_region(drvdata->fth_fd_cdev.dev, 1);
err_alloc:
	return ret;
}

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
/**
 * @brief Called to report a lptw gesture as a touch.
 *
 * @param state 0 = up, 1 = down, 2 = move
 * @param x x coordinate of the lptw gesture centroid.
 * @param y y coordinate of the lptw gesture centroid.
 * @param major major of the lptw gesture centroid.
 * @param minor minor of the lptw gesture centroid.
 * @param orientation orientation of the lptw gesture centroid.
 */
static void fth_lptw_report_event(int state, int x, int y, int major, int minor,
	int orientation) {
	struct fth_touch_event_v6 event;
	struct fth_drvdata *drvdata = fth_touch_handler.private;

	memset(&event, 0, sizeof(event));

	event.state = state;
	event.X[0] = abs(x);
	event.Y[0] = abs(y);
	event.major = major;
	event.minor = minor;
	event.orientation = orientation;
	// Use a unique finger slot so LPTW touches can be recognized.
	event.slot = FTH_LPTW_FINGER_SLOT;
	event.touch_valid = true;
	event.time_us = ktime_to_us(ktime_get());
	pr_info("lptw touch: state=%d params=(%d, %d, %d, %d, %d) time_us=%lld",
		state, event.X[0], event.Y[0], event.major, event.minor,
		event.orientation, event.time_us);

	fth_fd_report_event(drvdata, &event);
}

static int fth_lptw_notifier_callback(struct notifier_block *nb,
		unsigned long action, void *data)
{
	int *lptw_param = data;
	fth_lptw_report_event(action, lptw_param[0], lptw_param[1],
		lptw_param[2], lptw_param[3], lptw_param[4]);
	return NOTIFY_OK;
}

static struct notifier_block fth_notifier_block = {
	.notifier_call = fth_lptw_notifier_callback,
};
#endif

/**
 * fth_probe() - Function loads hardware config from device tree
 * @pdev:	ptr to platform device object
 *
 * Return: 0 on success. Error code on failure.
 */
static int fth_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fth_drvdata *drvdata;
	int rc = 0;
	int slot = 0;
	pr_debug("entry\n");
	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);
	atomic_set(&drvdata->fd_available, 1);
	atomic_set(&drvdata->wakelock_acquired, 0);
	mutex_init(&drvdata->mutex);
	mutex_init(&drvdata->fd_events_mutex);
	rc = fth_dev_register(drvdata);
	if (rc < 0)
		goto end;
	INIT_KFIFO(drvdata->fd_events);
	init_waitqueue_head(&drvdata->read_wait_queue_fd);
	rc = device_init_wakeup(&pdev->dev, 1);
	if (rc < 0)
		goto end;
	fth_touch_handler.private = drvdata;
	INIT_WORK(&drvdata->fd_touch.work, fth_touch_work_func);
	for (slot = 0; slot < FTH_MAX_FINGERS; slot++) {
		drvdata->fd_touch.current_events[slot].id = -1;
		drvdata->fd_touch.last_events[slot].id = -1;
	}
	rc = input_register_handler(&fth_touch_handler);
	if (rc < 0)
		pr_err("Failed to register input handler: %d\n", rc);

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	goog_lptw_notifier_register(&fth_notifier_block, true);
#endif
end:
	pr_debug("exit : %d\n", rc);
	return rc;
}

static int fth_remove(struct platform_device *pdev)
{
	struct fth_drvdata *drvdata = platform_get_drvdata(pdev);
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	goog_lptw_notifier_register(&fth_notifier_block, false);
#endif
	mutex_destroy(&drvdata->mutex);
	mutex_destroy(&drvdata->fd_events_mutex);
	device_destroy(drvdata->fth_class, drvdata->fth_fd_cdev.dev);
	class_destroy(drvdata->fth_class);
	cdev_del(&drvdata->fth_fd_cdev);
	unregister_chrdev_region(drvdata->fth_fd_cdev.dev, 1);
	device_init_wakeup(&pdev->dev, 0);
	input_unregister_handler(&fth_touch_handler);
	return 0;
}

static const struct of_device_id fth_match[] = {
	{ .compatible = "google,fps-touch-handler" },
	{}
};

static struct platform_driver fth_plat_driver = {
	.probe = fth_probe,
	.remove = fth_remove,
	.driver = {
		.name = "fps_touch_handler",
		.of_match_table = fth_match,
	},
};

static int __init fps_touch_handler_init(void)
{
	int ret;
	pr_debug("entry\n");
	ret = platform_driver_register(&fth_plat_driver);
	return ret;
}

static void __exit fps_touch_handler_exit(void)
{
	pr_debug("entry\n");
	platform_driver_unregister(&fth_plat_driver);
}

module_init(fps_touch_handler_init);
module_exit(fps_touch_handler_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FPS TOUCH HANDLER");

