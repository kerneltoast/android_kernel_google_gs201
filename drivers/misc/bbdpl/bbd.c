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
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/fs.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include "bbd.h"

#ifdef CONFIG_SENSORS_SSP
#include <linux/spi/spi.h> /* Needs because SSP is tightly coupled with SPI */

extern struct spi_driver *pssp_driver;
extern bool ssp_dbg;
extern bool ssp_pkt_dbg;

static const struct spi_device dummy_spi = {
	.dev = {
		.init_name = "mock",
	},
};
#endif

#if IS_ENABLED(CONFIG_BCM_GPS_SPI_DRIVER)
bool ssi_dbg;
EXPORT_SYMBOL_GPL(ssi_dbg);
bool ssi_dbg_pzc = true; /* SHOULD BE TRUE */
EXPORT_SYMBOL_GPL(ssi_dbg_pzc);
bool ssi_dbg_rng;
EXPORT_SYMBOL_GPL(ssi_dbg_rng);
#endif /* IS_ENABLED(CONFIG_BCM_GPS_SPI_DRIVER) */


#ifdef BBD_PWR_STATUS
struct gnss_pwrstats {
	bool    gps_stat;
	u64	gps_on_cnt;
	u64	gps_on_duration;
	u64	gps_on_entry;
	u64	gps_on_exit;
	u64	gps_off_cnt;
	u64	gps_off_duration;
	u64	gps_off_entry;
	u64	gps_off_exit;
};
#endif /* BBD_PWR_STATUS */

#define BBD_BUFF_SIZE (PAGE_SIZE*2)
struct bbd_cdev_priv {
	const char *name;
	struct cdev dev;			/* char device */
	bool busy;
	struct circ_buf read_buf;		/* LHD reads from BBD */
	struct mutex lock;			/* Lock for read_buf */
	char _read_buf[BBD_BUFF_SIZE];		/* LHD reads from BBD */
	char write_buf[BBD_BUFF_SIZE];		/* LHD writes into BBD */
	wait_queue_head_t poll_wait;		/* for poll */
#ifdef BBD_PWR_STATUS
	struct gnss_pwrstats pwrstats;		/* GNSS power state */
#endif /* BBD_PWR_STATUS */
};

struct bbd_device {
	struct class *class;			/* for device_create */

	struct bbd_cdev_priv priv[BBD_DEVICE_INDEX];/* individual structures */
	bool db;				/* debug flag */

	void *ssp_priv;				/* private data pointer */
	struct bbd_callbacks *ssp_cb;		/* callbacks for SSP */

	bool legacy_patch;		/* check for using legacy_bbd_patch */
	dev_t dev_num;			/* device number */
};

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

/*
 * The global BBD device which has all necessary information.
 * It's not beautiful but useful when we debug by Trace32.
 */
static struct bbd_device bbd; /* TODO(b/170369951): remove global variable */

/* Embedded patch file provided as /dev/bbd_patch */
static unsigned char bbd_patch[] = {
};

#ifdef CONFIG_SENSORS_BBD_LEGACY_PATCH
static unsigned char legacy_bbd_patch[] = {
#include "legacy_bbd_patch_file.h"
};
#else
static unsigned char legacy_bbd_patch[] = {
	"mock",
};
#endif

/* Function to push read data into any bbd device's read buf */
ssize_t bbd_on_read(unsigned int minor, const unsigned char *buf, size_t size);

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

struct bbd_stat stat1hz; /* TODO remove global variable */

/*
 * BBD 1hz Statistics Functions
 */

static void bbd_init_stat(void)
{
	memset(&stat1hz, 0, sizeof(stat1hz));

	stat1hz.min_rx_lat = (u64)-1;
	stat1hz.min_rx_dur = (u64)-1;
	stat1hz.workq = create_singlethread_workqueue("BBD_1HZ_TICK");
}

static void bbd_exit_stat(void)
{
	bbd_disable_stat();
	if (stat1hz.workq) {
		flush_workqueue(stat1hz.workq);
		destroy_workqueue(stat1hz.workq);
		stat1hz.workq = 0;
	}
}

static void bbd_report_stat(struct work_struct *work)
{
	const int MAX_SIZE = 512;
	char *buf;
	int i;
	int count = 0;

	buf = kvmalloc_array(MAX_SIZE, sizeof(char), GFP_KERNEL);
	if (!buf)
		return;

	count += scnprintf(buf + count, MAX_SIZE - count, "BBD:");
	for (i = 0; i < STAT_MAX; i++) {
		count += scnprintf(buf + count, MAX_SIZE - count, " %s=%llu",
				bbd_stat_name[i], stat1hz.stat[i]);
	}
	count += scnprintf(buf + count, MAX_SIZE - count,
			" rxlat_min=%llu rxlat_max=%llu",
			stat1hz.min_rx_lat, stat1hz.max_rx_lat);
	count += scnprintf(buf + count, MAX_SIZE - count,
			" rxdur_min=%llu rxdur_max=%llu",
			stat1hz.min_rx_dur, stat1hz.max_rx_dur);

	/* report only in case we had SSI traffic */
	if (stat1hz.stat[STAT_TX_SSI] || stat1hz.stat[STAT_RX_SSI])
		bbd_on_read(BBD_MINOR_CONTROL, buf, strlen(buf) + 1);

	for (i = 0; i < STAT_MAX; i++)
		stat1hz.stat[i] = 0;

	stat1hz.min_rx_lat = (u64)-1;
	stat1hz.min_rx_dur = (u64)-1;
	stat1hz.max_rx_lat = 0;
	stat1hz.max_rx_dur = 0;

	kvfree(buf);
}

static void bbd_stat_timer_func(unsigned long p)
{
	if (stat1hz.workq)
		queue_work(stat1hz.workq, &stat1hz.work);
	mod_timer(&stat1hz.timer, jiffies + HZ);
}

void bbd_update_stat(int idx, unsigned int count)
{
	stat1hz.stat[idx] += count;
}

void bbd_enable_stat(void)
{
	if (stat1hz.enabled) {
		pr_info("1HZ stat already enable. skipping.\n");
		return;
	}

	INIT_WORK(&stat1hz.work, bbd_report_stat);
	setup_timer(&stat1hz.timer, bbd_stat_timer_func, 0);
	mod_timer(&stat1hz.timer, jiffies + HZ);
	stat1hz.enabled = true;
}

void bbd_disable_stat(void)
{
	if (!stat1hz.enabled) {
		pr_info("1HZ stat already disabled. skipping.\n");
		return;
	}
	del_timer_sync(&stat1hz.timer);
	cancel_work_sync(&stat1hz.work);
	stat1hz.enabled = false;
}
#endif /* DEBUG_1HZ_STAT */


static void bbd_log_hex(const char *prefix_str,
		const unsigned char *buf,
		size_t len)
{
	if (likely(!bbd.db))
		return;

	if (!prefix_str)
		prefix_str = "...unknown...";

	print_hex_dump(KERN_INFO, prefix_str, DUMP_PREFIX_NONE, 32, 1,
			buf, len, false);
}

/**
 * bbd_control - Handles command string from lhd
 */
ssize_t bbd_control(const char *buf, ssize_t len)
{
#ifdef DEBUG_1HZ_STAT
	pr_info("%s\n", buf);
#endif

	if (!strcmp(buf, ESW_CTRL_READY)) {
		if (bbd.ssp_cb && bbd.ssp_cb->on_mcu_ready)
			bbd.ssp_cb->on_mcu_ready(bbd.ssp_priv, true);
	} else if (!strcmp(buf, ESW_CTRL_NOTREADY)) {
		struct circ_buf *circ = &bbd.priv[BBD_MINOR_SENSOR].read_buf;

		circ->head = circ->tail = 0;
		if (bbd.ssp_cb && bbd.ssp_cb->on_mcu_ready)
			bbd.ssp_cb->on_mcu_ready(bbd.ssp_priv, false);
	} else if (!strcmp(buf, ESW_CTRL_CRASHED)) {
		struct circ_buf *circ = &bbd.priv[BBD_MINOR_SENSOR].read_buf;

		circ->head = circ->tail = 0;

		if (bbd.ssp_cb && bbd.ssp_cb->on_mcu_ready)
			bbd.ssp_cb->on_mcu_ready(bbd.ssp_priv, false);

		if (bbd.ssp_cb && bbd.ssp_cb->on_control)
			bbd.ssp_cb->on_control(bbd.ssp_priv, buf);
	} else if (!strcmp(buf, BBD_CTRL_DEBUG_OFF)) {
		bbd.db = false;
#ifdef CONFIG_SENSORS_SSP
	} else if (!strcmp(buf, SSP_DEBUG_ON)) {
		ssp_dbg = true;
		ssp_pkt_dbg = true;
	} else if (!strstr(buf, SSP_DEBUG_OFF)) {
		ssp_dbg = false;
		ssp_pkt_dbg = false;
#endif
#if IS_ENABLED(CONFIG_BCM_GPS_SPI_DRIVER)
	} else if (!strcmp(buf, SSI_DEBUG_ON)) {
		ssi_dbg = true;
	} else if (!strcmp(buf, SSI_DEBUG_OFF)) {
		ssi_dbg = false;
	} else if (!strcmp(buf, PZC_DEBUG_ON)) {
		ssi_dbg_pzc = true;
	} else if (!strcmp(buf, PZC_DEBUG_OFF)) {
		ssi_dbg_pzc = false;
	} else if (!strcmp(buf, RNG_DEBUG_ON)) {
		ssi_dbg_rng = true;
	} else if (!strcmp(buf, RNG_DEBUG_OFF)) {
		ssi_dbg_rng = false;
#endif /* IS_ENABLED(CONFIG_BCM_GPS_SPI_DRIVER) */
#ifdef BBD_PWR_STATUS
	} else if (!strcmp(buf, GPSD_CORE_ON)) {
		u64 now = ktime_to_us(ktime_get_boottime());
		struct gnss_pwrstats *pwrstats =
			&bbd.priv[BBD_MINOR_PWRSTAT].pwrstats;

		mutex_lock(&bbd.priv[BBD_MINOR_PWRSTAT].lock);

		pwrstats->gps_stat = STAT_GPS_ON;
		pwrstats->gps_on_cnt++;
		pwrstats->gps_on_entry = now;
		pwrstats->gps_off_exit = now;
		pwrstats->gps_off_duration +=
			pwrstats->gps_off_exit - pwrstats->gps_off_entry;

		mutex_unlock(&bbd.priv[BBD_MINOR_PWRSTAT].lock);
	} else if (!strcmp(buf, GPSD_CORE_OFF)) {
		u64 now = ktime_to_us(ktime_get_boottime());
		struct gnss_pwrstats *pwrstats =
			&bbd.priv[BBD_MINOR_PWRSTAT].pwrstats;

		mutex_lock(&bbd.priv[BBD_MINOR_PWRSTAT].lock);

		pwrstats->gps_stat = STAT_GPS_OFF;
		pwrstats->gps_off_cnt++;
		pwrstats->gps_off_entry = now;
		pwrstats->gps_on_exit = now;
		pwrstats->gps_on_duration +=
			pwrstats->gps_on_exit - pwrstats->gps_on_entry;

		mutex_unlock(&bbd.priv[BBD_MINOR_PWRSTAT].lock);
#endif /* BBD_PWR_STATUS */
	} else if (bbd.ssp_cb && bbd.ssp_cb->on_control) {
		/* Tell SHMD about the unknown control string */
		bbd.ssp_cb->on_control(bbd.ssp_priv, buf);
	}

	return len;
}

/*
 * BBD Common File Functions
 */

/**
 * bbd_common_open - Common open function for BBD devices
 */
int bbd_common_open(struct inode *inode, struct file *filp)
{
	unsigned int minor = iminor(inode);
	struct circ_buf *circ = &bbd.priv[minor].read_buf;

	if (minor >= BBD_DEVICE_INDEX)
		return -ENODEV;

	if (bbd.priv[minor].busy && minor != BBD_MINOR_CONTROL)
		return -EBUSY;

	bbd.priv[minor].busy = true;

	/* Reset circ buffer */
	circ->head = circ->tail = 0;

	filp->private_data = &bbd;

	return 0;
}

/**
 * bbd_common_release - Common release function for BBD devices
 */
static int bbd_common_release(struct inode *inode, struct file *filp)
{
	unsigned int minor = iminor(inode);

	if (minor >= BBD_DEVICE_INDEX) {
		WARN_ON(minor >= BBD_DEVICE_INDEX);
		return 0;
	}
#ifdef DEBUG_1HZ_STAT
	pr_info("%s", bbd.priv[minor].name);
#endif

	bbd.priv[minor].busy = false;

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
	unsigned int minor = iminor(filp->f_path.dentry->d_inode);
	struct circ_buf *circ = &bbd.priv[minor].read_buf;
	size_t rd_size = 0;

	if (minor >= BBD_DEVICE_INDEX) {
		WARN_ON(minor >= BBD_DEVICE_INDEX);
		goto out;
	}

	mutex_lock(&bbd.priv[minor].lock);

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
			mutex_unlock(&bbd.priv[minor].lock);
			rd_size = -EFAULT;
			goto out;
		}

		size -= copied;
		rd_size += copied;
		circ->tail = (circ->tail + copied) & (BBD_BUFF_SIZE - 1);

	} while (size > 0 && CIRC_CNT(circ->head, circ->tail, BBD_BUFF_SIZE));

	mutex_unlock(&bbd.priv[minor].lock);

	bbd_log_hex(bbd_dev_name[minor], buf, rd_size);

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(STAT_RX_LHD, rd_size);
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
	unsigned int minor = iminor(filp->f_path.dentry->d_inode);

	if (size >= BBD_BUFF_SIZE)
		return -EFAULT;

	if (copy_from_user(bbd.priv[minor].write_buf, buf, size)) {
		pr_err("failed to copy from user.\n");
		return -EFAULT;
	}

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(STAT_TX_LHD, size);
#endif
	return size;
}

/**
 * bbd_common_poll - Common poll function for BBD devices
 */
static unsigned int bbd_common_poll(struct file *filp, poll_table *wait)
{
	unsigned int minor = iminor(filp->f_path.dentry->d_inode);
	struct circ_buf *circ = &bbd.priv[minor].read_buf;
	unsigned int mask = 0;

	if (minor >= BBD_DEVICE_INDEX)
		return POLLNVAL;

	poll_wait(filp, &bbd.priv[minor].poll_wait, wait);

	if (CIRC_CNT(circ->head, circ->tail, BBD_BUFF_SIZE))
		mask |= POLLIN;

	return mask;
}

/*
 * BBD Device Specific File Functions
 */

/**
 * bbd_sensor_write - BBD's RPC calls this function to send sensor packet
 *
 * @buf: contains sensor packet coming from gpsd/lhd
 *
 */
ssize_t bbd_sensor_write(const char *buf, size_t size)
{
	/*
	 * Copies into /dev/bbd_shmd. If SHMD was sleeping in poll_wait,
	 * bbd_on_read() wakes it up also
	 */
	bbd_on_read(BBD_MINOR_SHMD, buf, size);

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(STAT_RX_SSP, size);
#endif
	/* OK. Now call pre-registered SHMD callbacks */
	if (bbd.ssp_cb->on_packet)
		bbd.ssp_cb->on_packet(bbd.ssp_priv,
				bbd.priv[BBD_MINOR_SHMD].write_buf, size);
	else if (bbd.ssp_cb->on_packet_alarm)
		bbd.ssp_cb->on_packet_alarm(bbd.ssp_priv);
	else
		pr_err("no SSP on_packet callback registered. Dropped %zd bytes\n",
			size);

	/* TODO why return size without modified */
	return size;
}

/**
 * bbd_control_write - Write function for BBD control (/dev/bbd_control)
 *
 * Receives control string from lhd and handles it
 *
 */
ssize_t bbd_control_write(
	struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int minor = iminor(filp->f_path.dentry->d_inode);

	/* get command string first */
	ssize_t len = bbd_common_write(filp, buf, size, ppos);

	if (len <= 0)
		return len;

	/* Process received command string */
	return bbd_control(bbd.priv[minor].write_buf, len);
}

ssize_t bbd_patch_read(
	struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	ssize_t rd_size = size;
	size_t  offset = filp->f_pos;
	struct bbd_device *bbd = filp->private_data;
	unsigned char *curr_bbd_patch;
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
	struct gnss_pwrstats *pwrstats = &bbd.priv[BBD_MINOR_PWRSTAT].pwrstats;

	mutex_lock(&bbd.priv[BBD_MINOR_PWRSTAT].lock);

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

	mutex_unlock(&bbd.priv[BBD_MINOR_PWRSTAT].lock);

	return simple_read_from_buffer(buf, size, ppos, buf2, ret);
}
#endif /* BBD_PWR_STATUS */

static ssize_t bbd_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t len)
{
	bbd_control(buf, strlen(buf) + 1);
	return len;
}

static ssize_t pl_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{

	return 0;
}

static DEVICE_ATTR_WO(bbd);
static DEVICE_ATTR_RO(pl);

static struct attribute *bbd_attributes[] = {
	&dev_attr_bbd.attr,
	&dev_attr_pl.attr,
	NULL
};

static const struct attribute_group bbd_group = {
	.attrs = bbd_attributes,
};

/**
 *
 * bbd_on_read - Push data into read buffer of specified char device.
 *   if minor is bbd_sensor
 *
 * @buf: linear buffer
 */
ssize_t bbd_on_read(unsigned int minor, const unsigned char *buf, size_t size)
{
	struct circ_buf *circ = &bbd.priv[minor].read_buf;
	size_t wr_size = 0;

	bbd_log_hex(bbd_dev_name[minor], buf, size);

	mutex_lock(&bbd.priv[minor].lock);

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
	mutex_unlock(&bbd.priv[minor].lock);

	/* Wake up reader */
	wake_up(&bbd.priv[minor].poll_wait);


	return wr_size;
}

/*
 * PM Operation Functions
 */

static int bbd_suspend(pm_message_t state)
{

#ifdef DEBUG_1HZ_STAT
	bbd_disable_stat();
#endif
#ifdef CONFIG_SENSORS_SSP
	/* Call SSP suspend */
	if (pssp_driver->driver.pm && pssp_driver->driver.pm->suspend)
		pssp_driver->driver.pm->suspend(&dummy_spi.dev);
#endif
	mdelay(20);

	return 0;
}

static int bbd_resume(void)
{
#ifdef CONFIG_SENSORS_SSP
	/* Call SSP resume */
	if (pssp_driver->driver.pm && pssp_driver->driver.pm->suspend)
		pssp_driver->driver.pm->resume(&dummy_spi.dev);
#endif
#ifdef DEBUG_1HZ_STAT
	bbd_enable_stat();
#endif
	return 0;
}

static int bbd_notifier(
		struct notifier_block *nb, unsigned long event, void *data)
{
	pm_message_t state = {0};

	switch (event) {
	case PM_SUSPEND_PREPARE:
		state.event = event;
		bbd_suspend(state);
		break;
	case PM_POST_SUSPEND:
		bbd_resume();
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block bbd_notifier_block = {
	.notifier_call = bbd_notifier,
};

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


int bbd_init(struct device *dev, bool legacy_patch)
{
	int minor, ret = -ENOMEM;
	struct timespec64 ts1;
	unsigned long start, elapsed;


	ts1 = ktime_to_timespec64(ktime_get_boottime());
	start = ts1.tv_sec * 1000000000ULL + ts1.tv_nsec;

	/* Initialize BBD device */
	memset(&bbd, 0, sizeof(bbd)); /* TODO don't use global */

	bbd.legacy_patch = legacy_patch;

	/*
	 * Allocate device major number for this BBD device
	 * Starts minor number from 1 to ignore BBD SHMD device
	 */
	ret = alloc_chrdev_region(&bbd.dev_num, 1, BBD_DEVICE_INDEX, "bbd");
	if (ret) {
		pr_err("failed to alloc_chrdev_region(), ret=%d", ret);
		goto exit;
	}

	/* Create class which is required for device_create() */
	bbd.class = class_create(THIS_MODULE, "bbd");
	if (IS_ERR(bbd.class)) {
		pr_err("failed to create class bbd\n");
		goto exit;
	}

	/* Create BBD char devices */
	for (minor = 0; minor < BBD_DEVICE_INDEX; minor++) {
		dev_t devno = MKDEV(MAJOR(bbd.dev_num), minor);
		struct cdev *cdev = &bbd.priv[minor].dev;
		const char *name = bbd_dev_name[minor];
		struct device *dev;

		/* Init buf, waitqueue, mutex, etc. */
		bbd.priv[minor].name = bbd_dev_name[minor];
		bbd.priv[minor].read_buf.buf = bbd.priv[minor]._read_buf;

		init_waitqueue_head(&bbd.priv[minor].poll_wait);
		mutex_init(&bbd.priv[minor].lock);

#ifdef BBD_PWR_STATUS
		/* Initial power stats */
		memset(&bbd.priv[minor].pwrstats, 0, sizeof(struct gnss_pwrstats));
		bbd.priv[minor].pwrstats.gps_off_cnt = 1;
#endif /* BBD_PWR_STATUS */

		/* Don't register /dev/bbd_shmd */
		if (minor == BBD_MINOR_SHMD)
			continue;

		/*
		 * Register cdev which relates above
		 * device number with this BBD device
		 */
		cdev_init(cdev, &bbd_fops[minor]);
		cdev->owner = THIS_MODULE;
		cdev->ops = &bbd_fops[minor];
		ret = cdev_add(cdev, devno, 1);
		if (ret) {
			pr_err("failed to cdev_add() \"%s\", ret=%d",
					 name, ret);
			unregister_chrdev_region(devno, 1);
			goto free_class;
		}

		/* Let it show in FS */
		dev = device_create(bbd.class, NULL, devno, NULL, "%s", name);
		if (IS_ERR_OR_NULL(dev)) {
			pr_err("failed to device_create() \"%s\", ret = %d",
					 name, ret);
			unregister_chrdev_region(devno, 1);
			cdev_del(&bbd.priv[minor].dev);
			goto free_class;
		}

		/* Done. Put success log and init BBD specific fields */
		pr_info("(%d,%d) registered /dev/%s\n",
				MAJOR(bbd.dev_num), minor, name);

	}


	/* Register PM */
	ret = register_pm_notifier(&bbd_notifier_block);
	if (ret)
		goto free_class;

#ifdef CONFIG_SENSORS_SSP
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
	bbd_init_stat();
#endif
	return 0;

free_class:
	while (--minor > BBD_MINOR_SHMD) {
		dev_t devno = MKDEV(MAJOR(bbd.dev_num), minor);
		struct cdev *cdev = &bbd.priv[minor].dev;

		device_destroy(bbd.class, devno);
		cdev_del(cdev);
		unregister_chrdev_region(devno, 1);
	}
	class_destroy(bbd.class);
exit:
	return ret;
}

EXPORT_SYMBOL_GPL(bbd_init);

void bbd_exit(void)
{
	int minor;

#ifdef CONFIG_SENSORS_SSP
	/* Shutdown SSP first*/
	pssp_driver->shutdown(&dummy_spi);
#endif

	/* Remove BBD char devices */
	for (minor = BBD_MINOR_SENSOR; minor < BBD_DEVICE_INDEX; minor++) {
		dev_t devno = MKDEV(MAJOR(bbd.dev_num), minor);
		struct cdev *cdev = &bbd.priv[minor].dev;
		const char *name = bbd_dev_name[minor];

		device_destroy(bbd.class, devno);
		cdev_del(cdev);
		unregister_chrdev_region(devno, 1);

		pr_info("(%d,%d) unregistered /dev/%s\n",
				MAJOR(bbd.dev_num), minor, name);
	}

#ifdef DEBUG_1HZ_STAT
	bbd_exit_stat();
#endif
	/* Remove class */
	class_destroy(bbd.class);
}
EXPORT_SYMBOL_GPL(bbd_exit);

MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
