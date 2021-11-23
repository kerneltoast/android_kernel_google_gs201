// SPDX-License-Identifier: GPL-2.0
/* Copyright 2014 Broadcom Corporation
 *
 * The BBD (Broadcom Bridge Driver)
 *
 */

/* TODO: Use dev_*() calls instead */
#define pr_fmt(fmt) "GPSBBD: " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include "bbd.h"

#if IS_ENABLED(CONFIG_SENSORS_SSP)
#include <linux/spi/spi.h> /* Needs because SSP is tightly coupled with SPI */
extern struct spi_driver *pssp_driver;

static const struct spi_device dummy_spi = {
	.dev = {
		.init_name = "mock",
	},
};
#endif

/* Character device names of BBD */
static const char *bbd_dev_name[BBD_DEVICE_INDEX] = {
	"bbd_shmd",
	"bbd_sensor",
	"bbd_control",
	"bbd_patch",
#ifdef BBD_PWR_STATUS
	"bbd_pwrstat",
#endif /* BBD_PWR_STATUS */
};

/* Embedded patch file provided as /dev/bbd_patch */
static const unsigned char bbd_patch[] = {
};

#ifdef CONFIG_SENSORS_BBD_LEGACY_PATCH
static const unsigned char legacy_bbd_patch[] = {
#include "legacy_bbd_patch_file.h"
};
#else
static const unsigned char legacy_bbd_patch[] = {
	"mock",
};
#endif

/* Function to push read data into any bbd device's read buf */
ssize_t bbd_on_read(struct bbd_device *bbd, unsigned int minor,
		const unsigned char *buf, size_t size);

#ifdef DEBUG_1HZ_STAT

static const char *bbd_stat_name[STAT_MAX] = {
	"tx@lhd",
	"tx@ssp",
	"tx@rpc",
	"tx@tl",
	"tx@ssi",
	"rx@ssi",
	"rx@tl",
	"rx@rpc",
	"rx@ssp",
	"rx@lhd"
};

/*
 * BBD 1hz Statistics Functions
 */

static void bbd_init_stat(struct bbd_device *bbd)
{
	struct bbd_stat *stat1hz = &bbd->stat1hz;

	memset(stat1hz, 0, sizeof(*stat1hz));

	stat1hz->bbd = bbd;
	stat1hz->min_rx_lat = (u64)-1;
	stat1hz->min_rx_dur = (u64)-1;
	stat1hz->workq = create_singlethread_workqueue("BBD_1HZ_TICK");
}

static void bbd_exit_stat(struct bbd_device *bbd)
{
	struct bbd_stat *stat1hz = &bbd->stat1hz;

	bbd_disable_stat(bbd);
	if (stat1hz->workq) {
		flush_workqueue(stat1hz->workq);
		destroy_workqueue(stat1hz->workq);
		stat1hz->workq = 0;
	}
}

static void bbd_report_stat(struct work_struct *work)
{
	const int MAX_SIZE = 512;
	char *buf;
	int i;
	int count = 0;
	struct bbd_stat *stat1hz = container_of(work, struct bbd_stat, work);

	buf = kvmalloc_array(MAX_SIZE, sizeof(char), GFP_KERNEL);
	if (!buf)
		return;

	count += scnprintf(buf + count, MAX_SIZE - count, "BBD:");
	for (i = 0; i < STAT_MAX; i++) {
		count += scnprintf(buf + count, MAX_SIZE - count, " %s=%llu",
				bbd_stat_name[i], stat1hz->stat[i]);
	}
	count += scnprintf(buf + count, MAX_SIZE - count,
			" rxlat_min=%llu rxlat_max=%llu",
			stat1hz->min_rx_lat, stat1hz->max_rx_lat);
	count += scnprintf(buf + count, MAX_SIZE - count,
			" rxdur_min=%llu rxdur_max=%llu",
			stat1hz->min_rx_dur, stat1hz->max_rx_dur);

	/* report only in case we had SSI traffic */
	if (stat1hz->stat[STAT_TX_SSI] || stat1hz->stat[STAT_RX_SSI])
		bbd_on_read(stat1hz->bbd, BBD_MINOR_CONTROL, buf, count);

	for (i = 0; i < STAT_MAX; i++)
		stat1hz->stat[i] = 0;

	stat1hz->min_rx_lat = (u64)-1;
	stat1hz->min_rx_dur = (u64)-1;
	stat1hz->max_rx_lat = 0;
	stat1hz->max_rx_dur = 0;

	kvfree(buf);
}

static void bbd_stat_timer_func(struct timer_list *t)
{
	struct bbd_stat *stat1hz = container_of(t, struct bbd_stat, timer);
	if (stat1hz->workq)
		queue_work(stat1hz->workq, &stat1hz->work);
	mod_timer(&stat1hz->timer, jiffies + HZ);
}

void bbd_update_stat(struct bbd_device *bbd,
		int idx, unsigned int count)
{
	struct bbd_stat *stat1hz = &bbd->stat1hz;
	stat1hz->stat[idx] += count;
}
EXPORT_SYMBOL_GPL(bbd_update_stat);

void bbd_enable_stat(struct bbd_device *bbd)
{
	struct bbd_stat *stat1hz = &bbd->stat1hz;
	if (stat1hz->enabled) {
		dev_dbg(bbd->dev, "1HZ stat already enable. skipping.\n");
		return;
	}

	INIT_WORK(&stat1hz->work, bbd_report_stat);
	timer_setup(&stat1hz->timer, bbd_stat_timer_func, 0);
	mod_timer(&stat1hz->timer, jiffies + HZ);
	stat1hz->enabled = true;
}
EXPORT_SYMBOL_GPL(bbd_enable_stat);

void bbd_disable_stat(struct bbd_device *bbd)
{
	struct bbd_stat *stat1hz = &bbd->stat1hz;
	if (!stat1hz->enabled) {
		dev_dbg(bbd->dev, "1HZ stat already disabled. skipping.\n");
		return;
	}
	del_timer_sync(&stat1hz->timer);
	cancel_work_sync(&stat1hz->work);
	stat1hz->enabled = false;
}
EXPORT_SYMBOL_GPL(bbd_disable_stat);
#endif /* DEBUG_1HZ_STAT */


static void bbd_log_hex(bool log_enabled, const char *prefix_str,
		const unsigned char *buf, size_t len)
{
	if (likely(!log_enabled))
		return;

	if (!prefix_str)
		prefix_str = "...unknown...";

	print_hex_dump(KERN_INFO, prefix_str, DUMP_PREFIX_NONE, 32, 1,
			buf, len, false);
}

/**
 * bbd_control - Handles command string from lhd
 */
static ssize_t bbd_control(struct bbd_device *bbd, const char *buf, ssize_t len)
{
#ifdef DEBUG_1HZ_STAT
	pr_info("%s\n", buf);
#endif

	if (strcmp(buf, ESW_CTRL_READY)) {
		if (bbd->ssp_cb && bbd->ssp_cb->on_mcu_ready)
			bbd->ssp_cb->on_mcu_ready(bbd->ssp_priv, true);
	} else if (strcmp(buf, ESW_CTRL_NOTREADY)) {
		struct circ_buf *circ = &bbd->priv[BBD_MINOR_SENSOR].read_buf;

		circ->head = circ->tail = 0;
		if (bbd->ssp_cb && bbd->ssp_cb->on_mcu_ready)
			bbd->ssp_cb->on_mcu_ready(bbd->ssp_priv, false);
	} else if (strcmp(buf, ESW_CTRL_CRASHED)) {
		struct circ_buf *circ = &bbd->priv[BBD_MINOR_SENSOR].read_buf;

		circ->head = circ->tail = 0;

		if (bbd->ssp_cb && bbd->ssp_cb->on_mcu_ready)
			bbd->ssp_cb->on_mcu_ready(bbd->ssp_priv, false);

		if (bbd->ssp_cb && bbd->ssp_cb->on_control)
			bbd->ssp_cb->on_control(bbd->ssp_priv, buf);
	} else if (strcmp(buf, BBD_CTRL_DEBUG_OFF)) {
		bbd->db = false;
#if IS_ENABLED(CONFIG_SENSORS_SSP)
	} else if (!strcmp(buf, SSP_DEBUG_ON)) {
		bbd->ssp_dbg = true;
		bbd->ssp_pkt_dbg = true;
	} else if (!strstr(buf, SSP_DEBUG_OFF)) {
		bbd->ssp_dbg = false;
		bbd->ssp_pkt_dbg = false;
#endif
	} else if (strcmp(buf, SSI_DEBUG_ON)) {
		bcm_ssi_debug(bbd->dev, 0, true);
	} else if (strcmp(buf, SSI_DEBUG_OFF)) {
		bcm_ssi_debug(bbd->dev, 0, false);
	} else if (strcmp(buf, PZC_DEBUG_ON)) {
		bcm_ssi_debug(bbd->dev, 1, true);
	} else if (strcmp(buf, PZC_DEBUG_OFF)) {
		bcm_ssi_debug(bbd->dev, 1, false);
	} else if (strcmp(buf, RNG_DEBUG_ON)) {
		bcm_ssi_debug(bbd->dev, 2, true);
	} else if (strcmp(buf, RNG_DEBUG_OFF)) {
		bcm_ssi_debug(bbd->dev, 2, false);
#ifdef BBD_PWR_STATUS
	} else if (!strcmp(buf, GPSD_CORE_ON)) {
		u64 now = ktime_to_us(ktime_get_boottime());
		struct gnss_pwrstats *pwrstats =
			&bbd->priv[BBD_MINOR_PWRSTAT].pwrstats;

		mutex_lock(&bbd->priv[BBD_MINOR_PWRSTAT].lock);

		pwrstats->gps_stat = STAT_GPS_ON;
		pwrstats->gps_on_cnt++;
		pwrstats->gps_on_entry = now;
		pwrstats->gps_off_exit = now;
		pwrstats->gps_off_duration +=
		pwrstats->gps_off_exit - pwrstats->gps_off_entry;

		mutex_unlock(&bbd->priv[BBD_MINOR_PWRSTAT].lock);
	} else if (!strcmp(buf, GPSD_CORE_OFF)) {
		u64 now = ktime_to_us(ktime_get_boottime());
		struct gnss_pwrstats *pwrstats =
			&bbd->priv[BBD_MINOR_PWRSTAT].pwrstats;

		mutex_lock(&bbd->priv[BBD_MINOR_PWRSTAT].lock);

		pwrstats->gps_stat = STAT_GPS_OFF;
		pwrstats->gps_off_cnt++;
		pwrstats->gps_off_entry = now;
		pwrstats->gps_on_exit = now;
		pwrstats->gps_on_duration +=
		pwrstats->gps_on_exit - pwrstats->gps_on_entry;

		mutex_unlock(&bbd->priv[BBD_MINOR_PWRSTAT].lock);
#endif /* BBD_PWR_STATUS */
	} else if (bbd->ssp_cb && bbd->ssp_cb->on_control) {
		/* Tell SHMD about the unknown control string */
		bbd->ssp_cb->on_control(bbd->ssp_priv, buf);
	}

	return len;
}

/*
 * BBD Common File Functions
 */

/**
 * bbd_common_open - Common open function for BBD devices
 */
static int bbd_common_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct bbd_cdev_priv *bbd_cdev =
		container_of(cdev, struct bbd_cdev_priv, cdev);

	unsigned int minor = iminor(inode);
	struct circ_buf *circ = &bbd_cdev->read_buf;

	if (minor >= BBD_DEVICE_INDEX)
		return -ENODEV;

	if (bbd_cdev->busy && minor != BBD_MINOR_CONTROL)
		return -EBUSY;

	bbd_cdev->busy = true;

	/* Reset circ buffer */
	circ->head = circ->tail = 0;

	filp->private_data = bbd_cdev->bbd;

	return 0;
}

/**
 * bbd_common_release - Common release function for BBD devices
 */
static int bbd_common_release(struct inode *inode, struct file *filp)
{
	struct bbd_device *bbd = filp->private_data;
	unsigned int minor = iminor(inode);

	if (minor >= BBD_DEVICE_INDEX) {
		WARN_ON(minor >= BBD_DEVICE_INDEX);
		return 0;
	}
#ifdef DEBUG_1HZ_STAT
	pr_info("%s", bbd_dev_name[minor]);
#endif

	bbd->priv[minor].busy = false;

	return 0;
}

/**
 * bbd_common_read - Common read function for BBD devices
 *
 * lhd reads from BBD devices via this function
 *
 */
static ssize_t bbd_common_read(
		struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct bbd_device *bbd = filp->private_data;
	unsigned int minor = iminor(filp->f_path.dentry->d_inode);
	struct circ_buf *circ = &bbd->priv[minor].read_buf;
	size_t rd_size = 0;

	if (minor >= BBD_DEVICE_INDEX) {
		WARN_ON(minor >= BBD_DEVICE_INDEX);
		goto out;
	}

	mutex_lock(&bbd->priv[minor].lock);

	/*
	 * Copy from circ buffer to lhd
	 * Because lhd's buffer is linear,
	 * we may require 2 copies from [tail..end] and [end..head]
	 */
	do {
		size_t cnt_to_end = CIRC_CNT_TO_END(circ->head,
				circ->tail, BBD_BUFF_SIZE);
		size_t copied = min(cnt_to_end, size);

		if (copy_to_user(buf + rd_size,
				(void *) circ->buf + circ->tail, copied)) {
			mutex_unlock(&bbd->priv[minor].lock);
			rd_size = -EFAULT;
			goto out;
		}

		size -= copied;
		rd_size += copied;
		circ->tail = (circ->tail + copied) & (BBD_BUFF_SIZE - 1);

	} while (size > 0 && CIRC_CNT(circ->head, circ->tail, BBD_BUFF_SIZE));

	mutex_unlock(&bbd->priv[minor].lock);

	bbd_log_hex(bbd->db, bbd_dev_name[minor], buf, rd_size);

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(bbd, STAT_RX_LHD, rd_size);
#endif
out:
	return rd_size;
}

/**
 * bbd_common_write - Common write function for BBD devices
 * lhd writes to BBD devices via this function
 */
static ssize_t bbd_common_write(
	struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct bbd_device *bbd = filp->private_data;
	unsigned int minor = iminor(filp->f_path.dentry->d_inode);

	if (size >= BBD_BUFF_SIZE)
		return -EFAULT;

	if (copy_from_user(bbd->priv[minor].write_buf, buf, size)) {
		pr_err("failed to copy from user.\n");
		return -EFAULT;
	}

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(bbd, STAT_TX_LHD, size);
#endif
	return size;
}

/**
 * bbd_common_poll - Common poll function for BBD devices
 */
static unsigned int bbd_common_poll(struct file *filp, poll_table *wait)
{
	struct bbd_device *bbd = filp->private_data;
	unsigned int minor = iminor(filp->f_path.dentry->d_inode);
	struct circ_buf *circ = &bbd->priv[minor].read_buf;
	unsigned int mask = 0;

	if (minor >= BBD_DEVICE_INDEX)
		return POLLNVAL;

	poll_wait(filp, &bbd->priv[minor].poll_wait, wait);

	if (CIRC_CNT(circ->head, circ->tail, BBD_BUFF_SIZE))
		mask |= POLLIN;

	return mask;
}

/*
 * BBD Device Specific File Functions
 */


/**
 * bbd_control_write - Write function for BBD control (/dev/bbd_control)
 *
 * Receives control string from lhd and handles it
 *
 */
ssize_t bbd_control_write(
	struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct bbd_device *bbd = filp->private_data;
	unsigned int minor = iminor(filp->f_path.dentry->d_inode);

	/* get command string first */
	ssize_t len = bbd_common_write(filp, buf, size, ppos);

	if (len <= 0)
		return len;

	/* Process received command string */
	return bbd_control(bbd, bbd->priv[minor].write_buf, len);
}

ssize_t bbd_patch_read(
	struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	ssize_t rd_size = size;
	size_t  offset = filp->f_pos;
	struct bbd_device *bbd = filp->private_data;
	const unsigned char *curr_bbd_patch;
	size_t bbd_patch_sz;

	if (bbd->legacy_patch) {
		curr_bbd_patch = legacy_bbd_patch;
		bbd_patch_sz = sizeof(legacy_bbd_patch);
	} else {
		curr_bbd_patch = bbd_patch;
		bbd_patch_sz = sizeof(bbd_patch);
	}

	if (offset >= bbd_patch_sz) {       /* signal EOF */
		*ppos = 0;
		return 0;
	}
	if (offset+size > bbd_patch_sz)
		rd_size = bbd_patch_sz - offset;
	if (copy_to_user(buf, curr_bbd_patch + offset, rd_size))
		rd_size = -EFAULT;
	else
		*ppos = filp->f_pos + rd_size;

	return rd_size;
}

#ifdef BBD_PWR_STATUS
#define BBD_MAX_PWRSTAT_SIZE 512

ssize_t bbd_pwrstat_read(
	struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	const int MAX_SIZE = BBD_MAX_PWRSTAT_SIZE;
	char buf2[BBD_MAX_PWRSTAT_SIZE];
	int ret = 0;
	u64 now, gps_on_dur, gps_off_dur;
	struct bbd_device *bbd = filp->private_data;
	struct gnss_pwrstats *pwrstats = &bbd->priv[BBD_MINOR_PWRSTAT].pwrstats;

	mutex_lock(&bbd->priv[BBD_MINOR_PWRSTAT].lock);

	now = ktime_to_us(ktime_get_boottime());
	if (pwrstats->gps_stat == STAT_GPS_ON) {
		gps_on_dur = pwrstats->gps_on_duration +
			(now - pwrstats->gps_on_entry);
		gps_off_dur = pwrstats->gps_off_duration;
	} else {
		gps_on_dur = pwrstats->gps_on_duration;
		gps_off_dur = pwrstats->gps_off_duration +
			(now - pwrstats->gps_off_entry);
	}

	ret += scnprintf(buf2 + ret, MAX_SIZE - ret, "GPS_ON:\n");
	ret += scnprintf(buf2 + ret, MAX_SIZE - ret,
			"count: 0x%0llx\n", pwrstats->gps_on_cnt);
	ret += scnprintf(buf2 + ret, MAX_SIZE - ret,
			"duration_usec: 0x%0llx\n", gps_on_dur);
	ret += scnprintf(buf2 + ret, MAX_SIZE - ret,
			"last_entry_timestamp_usec: 0x%0llx\n",
			pwrstats->gps_on_entry);
	ret += scnprintf(buf2 + ret, MAX_SIZE - ret,
			"last_exit_timestamp_usec: %0lld\n",
			pwrstats->gps_on_exit);

	ret += scnprintf(buf2 + ret, MAX_SIZE - ret, "GPS_OFF:\n");
	ret += scnprintf(buf2 + ret, MAX_SIZE - ret,
			"count: 0x%0llx\n", pwrstats->gps_off_cnt);
	ret += scnprintf(buf2 + ret, MAX_SIZE - ret,
			"duration_usec: 0x%0llx\n", gps_off_dur);
	ret += scnprintf(buf2 + ret, MAX_SIZE - ret,
			"last_entry_timestamp_usec: 0x%0llx\n",
			pwrstats->gps_off_entry);
	ret += scnprintf(buf2 + ret, MAX_SIZE - ret,
			"last_exit_timestamp_usec: 0x%0llx\n",
			pwrstats->gps_off_exit);

	mutex_unlock(&bbd->priv[BBD_MINOR_PWRSTAT].lock);

	return simple_read_from_buffer(buf, size, ppos, buf2, ret);
}
#endif /* BBD_PWR_STATUS */
/**
 *
 * bbd_on_read - Push data into read buffer of specified char device.
 *   if minor is bbd_sensor
 *
 * @buf: linear buffer
 */
ssize_t bbd_on_read(struct bbd_device *bbd, unsigned int minor,
		const unsigned char *buf, size_t size)
{
	struct circ_buf *circ = &bbd->priv[minor].read_buf;
	size_t wr_size = 0;

	bbd_log_hex(bbd->db, bbd_dev_name[minor], buf, size);

	mutex_lock(&bbd->priv[minor].lock);

	/* If there's not enough speace, drop it but try waking up reader */
	if (CIRC_SPACE(circ->head, circ->tail, BBD_BUFF_SIZE) < size) {
		pr_err("%s read buffer full. Dropping %zd bytes\n",
				bbd_dev_name[minor], size);
		goto skip;
	}

	/*
	 * Copy into circ buffer from linear buffer
	 * We may require 2 copies from [head..end] and [start..head]
	 */
	do {
		size_t space_to_end = CIRC_SPACE_TO_END(
				circ->head, circ->tail, BBD_BUFF_SIZE);
		size_t copied = min(space_to_end, size);

		memcpy(circ->buf + circ->head, buf + wr_size, copied);
		size -= copied;
		wr_size += copied;
		circ->head = (circ->head + copied) & (BBD_BUFF_SIZE - 1);

	} while (size > 0 && CIRC_SPACE(circ->head, circ->tail, BBD_BUFF_SIZE));
skip:
	mutex_unlock(&bbd->priv[minor].lock);

	/* Wake up reader */
	wake_up(&bbd->priv[minor].poll_wait);

	return wr_size;
}

/*
 * PM Operation Functions
 */

static int bbd_suspend(struct bbd_device *bbd, pm_message_t state)
{

#ifdef DEBUG_1HZ_STAT
	bbd_disable_stat(bbd);
#endif
#if IS_ENABLED(CONFIG_SENSORS_SSP)
	/* Call SSP suspend */
	if (pssp_driver->driver.pm && pssp_driver->driver.pm->suspend)
		pssp_driver->driver.pm->suspend(&dummy_spi.dev);
	/* Per (b/203008378#comment3), the following delay is specified to the
	 * chips sensor hub system. Moving the delay to within the define removes
	 * it when the features is not used. Updating to a non-blocking sleep
	 * instead of a busy wait.
	 */
	msleep(20);
#endif

	return 0;
}

static int bbd_resume(struct bbd_device *bbd)
{
#if IS_ENABLED(CONFIG_SENSORS_SSP)
	/* Call SSP resume */
	if (pssp_driver->driver.pm && pssp_driver->driver.pm->suspend)
		pssp_driver->driver.pm->resume(&dummy_spi.dev);
#endif
#ifdef DEBUG_1HZ_STAT
	bbd_enable_stat(bbd);
#endif
	return 0;
}

static int bbd_notifier(
		struct notifier_block *nb, unsigned long event, void *data)
{
	struct bbd_device *bbd = container_of(nb, struct bbd_device, notifier);
	pm_message_t state = {0};

	switch (event) {
	case PM_SUSPEND_PREPARE:
		state.event = event;
		bbd_suspend(bbd, state);
		break;
	case PM_POST_SUSPEND:
		bbd_resume(bbd);
		break;
	}
	return NOTIFY_OK;
}

/*
 * BBD Device Init and Exit Functions
 */

static const struct file_operations bbd_fops[BBD_DEVICE_INDEX] = {
	/* bbd shmd file operations */
	{
		.owner          =  THIS_MODULE,
	},
	/* bbd sensor file operations */
	{
		.owner          =  THIS_MODULE,
		.open           =  bbd_common_open,
		.release        =  bbd_common_release,
		.read           =  bbd_common_read,
		.write          =  NULL,
		.poll           =  bbd_common_poll,
	},
	/* bbd control file operations */
	{
		.owner		=  THIS_MODULE,
		.open		=  bbd_common_open,
		.release	=  bbd_common_release,
		.read		=  bbd_common_read,
		.write		=  bbd_control_write,
		.poll		=  bbd_common_poll,
	},
	/* bbd patch file operations */
	{
		.owner		=  THIS_MODULE,
		.open		=  bbd_common_open,
		.release	=  bbd_common_release,
		.read		=  bbd_patch_read,
		.write		=  NULL, /* /dev/bbd_patch is read-only */
		.poll		=  NULL,
	},
#ifdef BBD_PWR_STATUS
	/* bbd power file operations */
	{
		.owner          =  THIS_MODULE,
		.open           =  bbd_common_open,
		.release        =  bbd_common_release,
		.read           =  bbd_pwrstat_read,
		.write          =  NULL,
		.poll		=  NULL,
	},
#endif /* BBD_PWR_STATUS */
};


struct bbd_device *bbd_init(struct device *dev, bool legacy_patch)
{
	int minor, ret = -ENOMEM;
	struct timespec64 ts1;
	unsigned long start, elapsed;
	struct bbd_device *bbd;

	ts1 = ktime_to_timespec64(ktime_get_boottime());
	start = ts1.tv_sec * 1000000000ULL + ts1.tv_nsec;

	/* Initialize BBD device */
	bbd = kvzalloc(sizeof(struct bbd_device), GFP_KERNEL);
	if (!bbd) {
		ret = -ENOMEM;
		goto exit;
	}

	bbd->dev = dev;
	bbd->legacy_patch = legacy_patch;

	/*
	 * Allocate device major number for this BBD device
	 * Starts minor number from 1 to ignore BBD SHMD device
	 */
	ret = alloc_chrdev_region(&bbd->dev_num, 1, BBD_DEVICE_INDEX, "bbd");
	if (ret) {
		pr_err("failed to alloc_chrdev_region(), ret=%d", ret);
		goto exit;
	}

	/* Create class which is required for device_create() */
	bbd->class = class_create(THIS_MODULE, "bbd");
	if (IS_ERR(bbd->class)) {
		pr_err("failed to create class bbd\n");
		goto exit;
	}

	/* Create BBD char devices */
	for (minor = 0; minor < BBD_DEVICE_INDEX; minor++) {
		struct bbd_cdev_priv *bbd_cdev = &bbd->priv[minor];

		/* Init buf, waitqueue, mutex, etc. */
		bbd_cdev->bbd = bbd;
		bbd_cdev->devno = MKDEV(MAJOR(bbd->dev_num), minor);
		bbd_cdev->read_buf.buf = bbd_cdev->_read_buf;

		init_waitqueue_head(&bbd_cdev->poll_wait);
		mutex_init(&bbd_cdev->lock);

#ifdef BBD_PWR_STATUS
		/* Initial power stats */
		memset(&bbd->priv[minor].pwrstats, 0, sizeof(struct gnss_pwrstats));
		bbd->priv[minor].pwrstats.gps_off_cnt = 1;
#endif /* BBD_PWR_STATUS */

		/* Don't register /dev/bbd_shmd */
		if (minor == BBD_MINOR_SHMD)
			continue;

		/*
		 * Register cdev which relates above
		 * device number with this BBD device
		 */
		cdev_init(&bbd_cdev->cdev, &bbd_fops[minor]);
		bbd_cdev->cdev.owner = THIS_MODULE;
		bbd_cdev->cdev.ops = &bbd_fops[minor];
		ret = cdev_add(&bbd_cdev->cdev, bbd_cdev->devno, 1);
		if (ret) {
			pr_err("failed to cdev_add() \"%s\", ret=%d",
					 bbd_dev_name[minor], ret);
			unregister_chrdev_region(bbd_cdev->devno, 1);
			goto free_class;
		}

		/* Let it show in FS */
		bbd_cdev->dev = device_create(bbd->class, NULL,
				bbd_cdev->devno, bbd, "%s", bbd_dev_name[minor]);
		if (IS_ERR_OR_NULL(dev)) {
			pr_err("failed to device_create() \"%s\", ret = %d",
					bbd_dev_name[minor], ret);
			unregister_chrdev_region(bbd_cdev->devno, 1);
			cdev_del(&bbd_cdev->cdev);
			goto free_class;
		}

		/* Done. Put success log and init BBD specific fields */
		pr_info("(%d,%d) registered /dev/%s\n",
				MAJOR(bbd->dev_num), minor, bbd_dev_name[minor]);
	}

	bbd->notifier.notifier_call = bbd_notifier;

	/* Register PM */
	ret = register_pm_notifier(&bbd->notifier);
	if (ret)
		goto free_class;

#if IS_ENABLED(CONFIG_SENSORS_SSP)
	/* Now, we can initialize SSP */
	if (device_register(&dummy_spi.dev))
		goto free_class;

	struct spi_device *spi = to_spi_device(dev);
	void *org_priv, *new_priv;

	org_priv = spi_get_drvdata(spi);
	pssp_driver->probe(spi);
	new_priv = spi_get_drvdata(spi);
	spi_set_drvdata(spi, org_priv);
	spi_set_drvdata(&dummy_spi, new_priv);
#endif
	ts1 = ktime_to_timespec64(ktime_get_boottime());
	elapsed = (ts1.tv_sec * 1000000000ULL + ts1.tv_nsec) - start;
	pr_info("%lu nsec elapsed\n", elapsed);

#ifdef DEBUG_1HZ_STAT
	bbd_init_stat(bbd);
#endif
	return bbd;

free_class:
	while (--minor > BBD_MINOR_SHMD) {
		dev_t devno = MKDEV(MAJOR(bbd->dev_num), minor);
		struct cdev *cdev = &bbd->priv[minor].cdev;

		device_destroy(bbd->class, devno);
		cdev_del(cdev);
		unregister_chrdev_region(devno, 1);
	}
	class_destroy(bbd->class);
exit:
	kvfree(bbd);
	return NULL;
}

EXPORT_SYMBOL_GPL(bbd_init);

void bbd_exit(struct device *dev)
{
	struct bbd_device *bbd = dev_get_drvdata(dev);
	int minor;

#if IS_ENABLED(CONFIG_SENSORS_SSP)
	/* Shutdown SSP first*/
	pssp_driver->shutdown(&dummy_spi);
#endif

	/* Remove BBD char devices */
	for (minor = BBD_MINOR_SENSOR; minor < BBD_DEVICE_INDEX; minor++) {
		struct bbd_cdev_priv *bbd_cdev = &bbd->priv[minor];

		device_destroy(bbd->class, bbd_cdev->devno);
		cdev_del(&bbd_cdev->cdev);
		unregister_chrdev_region(bbd_cdev->devno, 1);

		pr_info("(%d,%d) unregistered /dev/%s\n",
				MAJOR(bbd->dev_num), minor, bbd_dev_name[minor]);
	}

#ifdef DEBUG_1HZ_STAT
	bbd_exit_stat(bbd);
#endif
	/* Remove class */
	class_destroy(bbd->class);
	kvfree(bbd);
}

EXPORT_SYMBOL_GPL(bbd_exit);

MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
