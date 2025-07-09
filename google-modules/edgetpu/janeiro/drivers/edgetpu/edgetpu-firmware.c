// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU firmware loader.
 *
 * Copyright (C) 2019-2020 Google, Inc.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "edgetpu.h"
#include "edgetpu-device-group.h"
#include "edgetpu-firmware.h"
#include "edgetpu-firmware-util.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-pm.h"
#include "edgetpu-sw-watchdog.h"
#include "edgetpu-telemetry.h"

static char *firmware_name;
module_param(firmware_name, charp, 0660);

/*
 * Any tracing level vote with the following bit set will be considered as a default vote.
 */
#define EDGETPU_FW_TRACING_DEFAULT_VOTE BIT(8)

struct edgetpu_fw_tracing {
	struct device *dev;
	struct dentry *dentry;

	/*
	 * Lock to protect the struct members listed below.
	 *
	 * Note that since the request of tracing level adjusting might happen during power state
	 * transitions (i.e., another thread calling edgetpu_firmware_tracing_restore_on_powering()
	 * with pm lock held), one must either use the non-blocking edgetpu_pm_trylock() or make
	 * sure there won't be any new power transition after holding this lock to prevent deadlock.
	 */
	struct mutex lock;
	/* Actual firmware tracing level. */
	unsigned long active_level;
	/* Requested firmware tracing level. */
	unsigned long request_level;
};

struct edgetpu_firmware_private {
	const struct edgetpu_firmware_chip_data *chip_fw;
	void *data; /* for edgetpu_firmware_(set/get)_data */

	struct mutex fw_desc_lock;
	struct edgetpu_firmware_desc fw_desc;
	struct edgetpu_firmware_desc bl1_fw_desc;
	enum edgetpu_firmware_status status;
	struct edgetpu_fw_info fw_info;
	struct edgetpu_fw_tracing fw_tracing;
};

void edgetpu_firmware_set_data(struct edgetpu_firmware *et_fw, void *data)
{
	et_fw->p->data = data;
}

void *edgetpu_firmware_get_data(struct edgetpu_firmware *et_fw)
{
	return et_fw->p->data;
}

static int edgetpu_firmware_load_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc, const char *name,
		enum edgetpu_firmware_flags flags)
{
	const struct edgetpu_firmware_chip_data *chip_fw = et_fw->p->chip_fw;
	struct edgetpu_dev *etdev = et_fw->etdev;
	int ret;

	fw_desc->buf.flags = flags;

	if (chip_fw->alloc_buffer) {
		ret = chip_fw->alloc_buffer(et_fw, &fw_desc->buf);
		if (ret) {
			etdev_err(etdev, "handler alloc_buffer failed: %d\n",
				  ret);
			return ret;
		}
	}

	ret = edgetpu_firmware_chip_load_locked(et_fw, fw_desc, name);
	if (ret) {
		etdev_err(etdev, "firmware request failed: %d\n", ret);
		goto out_free_buffer;
	}

	if (chip_fw->setup_buffer) {
		ret = chip_fw->setup_buffer(et_fw, &fw_desc->buf);
		if (ret) {
			etdev_err(etdev, "handler setup_buffer failed: %d\n",
				  ret);
			goto out_unload_locked;
		}
	}

	return 0;

out_unload_locked:
	edgetpu_firmware_chip_unload_locked(et_fw, fw_desc);
out_free_buffer:
	if (chip_fw->free_buffer)
		chip_fw->free_buffer(et_fw, &fw_desc->buf);
	return ret;
}

static void edgetpu_firmware_unload_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc)
{
	const struct edgetpu_firmware_chip_data *chip_fw = et_fw->p->chip_fw;

	/*
	 * Platform specific implementation for cleaning up allocated buffer.
	 */
	if (chip_fw->teardown_buffer)
		chip_fw->teardown_buffer(et_fw, &fw_desc->buf);
	edgetpu_firmware_chip_unload_locked(et_fw, fw_desc);
	/*
	 * Platform specific implementation for freeing allocated buffer.
	 */
	if (chip_fw->free_buffer)
		chip_fw->free_buffer(et_fw, &fw_desc->buf);
}

static char *fw_flavor_str(enum edgetpu_fw_flavor fw_flavor)
{
	switch (fw_flavor) {
	case FW_FLAVOR_BL1:
		return "stage 2 bootloader";
	case FW_FLAVOR_SYSTEST:
		return "test";
	case FW_FLAVOR_PROD_DEFAULT:
		return "prod";
	case FW_FLAVOR_CUSTOM:
		return "custom";
	default:
	case FW_FLAVOR_UNKNOWN:
		return "unknown";
	}

	/* NOTREACHED */
	return "?";
}

static int edgetpu_firmware_tracing_active_get(void *data, u64 *val)
{
	struct edgetpu_firmware *et_fw = data;
	struct edgetpu_fw_tracing *fw_tracing = &et_fw->p->fw_tracing;

	mutex_lock(&fw_tracing->lock);
	*val = fw_tracing->active_level;
	mutex_unlock(&fw_tracing->lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_edgetpu_firmware_tracing_active, edgetpu_firmware_tracing_active_get,
			 NULL, "%llu\n");

static int edgetpu_firmware_tracing_request_get(void *data, u64 *val)
{
	struct edgetpu_firmware *et_fw = data;
	struct edgetpu_fw_tracing *fw_tracing = &et_fw->p->fw_tracing;

	mutex_lock(&fw_tracing->lock);
	*val = fw_tracing->request_level;
	mutex_unlock(&fw_tracing->lock);

	return 0;
}

/*
 * fw_tracing->lock may optionally be held if the caller wants the new level to be set as a
 * critical section.  If not held the caller is syncing current tracing level but not as a critical
 * section with the calling code.  Firmware tracing levels are not expected to change frequently or
 * via concurrent requests.  Only the code that restore the tracing level at power up requires
 * consistency with the state managed by the calling code.  Since this code is called as part of
 * power up processing, in order to avoid deadlocks, most callers set a requested state and then
 * sync the current state to firmware (if powered on) without holding the lock across the powered-on
 * check, with no harm done if the requested state changed again using a concurrent request.
 */
static int edgetpu_firmware_tracing_set_level(struct edgetpu_firmware *et_fw)
{
	unsigned long active_level;
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct edgetpu_fw_tracing *fw_tracing = &et_fw->p->fw_tracing;
	int ret = edgetpu_kci_firmware_tracing_level(etdev, fw_tracing->request_level,
						     &active_level);

	if (ret)
		etdev_warn(et_fw->etdev, "Failed to set firmware tracing level to %lu: %d",
			   fw_tracing->request_level, ret);
	else
		fw_tracing->active_level =
			(fw_tracing->request_level & EDGETPU_FW_TRACING_DEFAULT_VOTE) ?
				EDGETPU_FW_TRACING_DEFAULT_VOTE : active_level;

	return ret;
}

static int edgetpu_firmware_tracing_request_set(void *data, u64 val)
{
	struct edgetpu_firmware *et_fw = data;
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct edgetpu_fw_tracing *fw_tracing = &et_fw->p->fw_tracing;
	int ret = 0;

	mutex_lock(&fw_tracing->lock);
	fw_tracing->request_level = val;
	mutex_unlock(&fw_tracing->lock);

	if (edgetpu_pm_get_if_powered(etdev->pm)) {
		ret = edgetpu_firmware_tracing_set_level(et_fw);
		edgetpu_pm_put(etdev->pm);
	}

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_edgetpu_firmware_tracing_request,
			 edgetpu_firmware_tracing_request_get, edgetpu_firmware_tracing_request_set,
			 "%llu\n");

static void edgetpu_firmware_tracing_init(struct edgetpu_firmware *et_fw)
{
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct edgetpu_fw_tracing *fw_tracing = &et_fw->p->fw_tracing;

	fw_tracing->active_level = EDGETPU_FW_TRACING_DEFAULT_VOTE;
	fw_tracing->request_level = EDGETPU_FW_TRACING_DEFAULT_VOTE;
	mutex_init(&fw_tracing->lock);

	fw_tracing->dentry = debugfs_create_dir("fw_tracing", etdev->d_entry);
	if (IS_ERR(fw_tracing->dentry)) {
		etdev_warn(etdev, "Failed to create fw tracing debugfs interface");
		return;
	}

	debugfs_create_file("active", 0440, fw_tracing->dentry, et_fw,
			    &fops_edgetpu_firmware_tracing_active);
	debugfs_create_file("request", 0660, fw_tracing->dentry, et_fw,
			    &fops_edgetpu_firmware_tracing_request);
}

static void edgetpu_firmware_tracing_destroy(struct edgetpu_firmware *et_fw)
{
	debugfs_remove_recursive(et_fw->p->fw_tracing.dentry);
}

static int edgetpu_firmware_tracing_restore_on_powering(struct edgetpu_firmware *et_fw)
{
	int ret = 0;
	struct edgetpu_fw_tracing *fw_tracing = &et_fw->p->fw_tracing;

	mutex_lock(&fw_tracing->lock);
	fw_tracing->active_level = EDGETPU_FW_TRACING_DEFAULT_VOTE;
	if (!(fw_tracing->request_level & EDGETPU_FW_TRACING_DEFAULT_VOTE))
		ret = edgetpu_firmware_tracing_set_level(et_fw);
	mutex_unlock(&fw_tracing->lock);
	return ret;
}

static int edgetpu_firmware_handshake(struct edgetpu_firmware *et_fw)
{
	struct edgetpu_dev *etdev = et_fw->etdev;
	enum edgetpu_fw_flavor fw_flavor;
	struct edgetpu_firmware_buffer *fw_buf;

	etdev_dbg(etdev, "Detecting firmware info...");
	et_fw->p->fw_info.fw_build_time = 0;
	et_fw->p->fw_info.fw_flavor = FW_FLAVOR_UNKNOWN;
	et_fw->p->fw_info.fw_changelist = 0;
	fw_flavor = edgetpu_kci_fw_info(etdev->kci, &et_fw->p->fw_info);
	if (fw_flavor < 0) {
		etdev_err(etdev, "firmware handshake failed: %d", fw_flavor);
		et_fw->p->fw_info.fw_flavor = FW_FLAVOR_UNKNOWN;
		et_fw->p->fw_info.fw_changelist = 0;
		et_fw->p->fw_info.fw_build_time = 0;
		return fw_flavor;
	}

	if (fw_flavor != FW_FLAVOR_BL1) {
		fw_buf = &et_fw->p->fw_desc.buf;
		etdev_info(etdev, "loaded %s firmware%s (%u.%u %u)",
			   fw_flavor_str(fw_flavor),
			   fw_buf->flags & FW_ONDEV ? " on device" : "",
			   etdev->fw_version.major_version,
			   etdev->fw_version.minor_version,
			   et_fw->p->fw_info.fw_changelist);
	} else {
		etdev_dbg(etdev, "loaded stage 2 bootloader");
	}
	/* In case older firmware that doesn't fill out fw_info. */
	et_fw->p->fw_info.fw_flavor = fw_flavor;
	/* don't attempt log/trace handshake if it's the second-stage bootloader */
	if (fw_flavor != FW_FLAVOR_BL1) {
		int ret = edgetpu_telemetry_kci(etdev);

		if (ret)
			etdev_warn(etdev, "telemetry KCI error: %d", ret);
		ret = edgetpu_firmware_tracing_restore_on_powering(et_fw);
		if (ret)
			etdev_warn_ratelimited(etdev, "firmware tracing restore error: %d", ret);
	}
	return 0;
}

/*
 * Do edgetpu_pm_get() but prevent it from running the loaded firmware.
 *
 * On success, caller must later call edgetpu_pm_put() to decrease the reference count.
 *
 * Caller holds firmware lock.
 */
static int edgetpu_firmware_pm_get(struct edgetpu_firmware *et_fw)
{
	enum edgetpu_firmware_status prev = et_fw->p->status;
	int ret;

	/* Prevent platform-specific code from trying to run the previous firmware */
	et_fw->p->status = FW_LOADING;
	etdev_dbg(et_fw->etdev, "Requesting power up for firmware run\n");
	ret = edgetpu_pm_get(et_fw->etdev->pm);
	if (ret)
		et_fw->p->status = prev;
	return ret;
}

static void edgetpu_firmware_set_loading(struct edgetpu_firmware *et_fw)
{
	struct edgetpu_dev *etdev = et_fw->etdev;

	mutex_lock(&etdev->state_lock);
	etdev->state = ETDEV_STATE_FWLOADING;
	mutex_unlock(&etdev->state_lock);

	et_fw->p->status = FW_LOADING;
}

/* Set firmware and etdev state according to @ret, which can be an errno or 0. */
static void edgetpu_firmware_set_state(struct edgetpu_firmware *et_fw, int ret)
{
	struct edgetpu_dev *etdev = et_fw->etdev;

	et_fw->p->status = ret ? FW_INVALID : FW_VALID;

	mutex_lock(&etdev->state_lock);
	if (ret == -EIO)
		etdev->state = ETDEV_STATE_BAD; /* f/w handshake error */
	else if (ret)
		etdev->state = ETDEV_STATE_NOFW; /* other errors */
	else
		etdev->state = ETDEV_STATE_GOOD; /* f/w handshake success */
	mutex_unlock(&etdev->state_lock);
}

enum edgetpu_fw_flavor
edgetpu_firmware_get_flavor(struct edgetpu_firmware *et_fw)
{
	return et_fw->p->fw_info.fw_flavor;
}

uint32_t
edgetpu_firmware_get_cl(struct edgetpu_firmware *et_fw)
{
	return et_fw->p->fw_info.fw_changelist;
}

uint64_t
edgetpu_firmware_get_build_time(struct edgetpu_firmware *et_fw)
{
	return et_fw->p->fw_info.fw_build_time;
}

/*
 * Try edgetpu_firmware_lock() if it's not locked yet.
 *
 * Returns 1 if the lock is acquired successfully, 0 otherwise.
 */
int edgetpu_firmware_trylock(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;

	if (!et_fw)
		return 1;
	return mutex_trylock(&et_fw->p->fw_desc_lock);
}

/*
 * Grab firmware lock to protect against firmware state changes.
 * Locks out firmware loading / unloading while caller performs ops that are
 * incompatible with a change in firmware status.  Does not care whether or not
 * the device is joined to a group.
 */
int edgetpu_firmware_lock(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;

	if (!et_fw)
		return -EINVAL;
	mutex_lock(&et_fw->p->fw_desc_lock);
	return 0;
}

/* Drop f/w lock, let any pending firmware load proceed. */
void edgetpu_firmware_unlock(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;

	if (!et_fw)
		return;
	mutex_unlock(&et_fw->p->fw_desc_lock);
}

/*
 * Lock firmware for loading.  Disallow group join for device during load.
 * Failed if device is already joined to a group and is in use.
 */
static int edgetpu_firmware_load_lock(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;

	if (!et_fw) {
		etdev_err(
			etdev,
			"Cannot load firmware when no loader is available\n");
		return -EINVAL;
	}
	mutex_lock(&et_fw->p->fw_desc_lock);

	/* Disallow group join while loading, fail if already joined */
	if (!edgetpu_set_group_join_lockout(etdev, true)) {
		etdev_err(
			etdev,
			"Cannot load firmware because device is in use");
		mutex_unlock(&et_fw->p->fw_desc_lock);
		return -EBUSY;
	}
	return 0;
}

/* Unlock firmware after lock held for loading, re-allow group join. */
static void edgetpu_firmware_load_unlock(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;

	if (!et_fw) {
		etdev_dbg(etdev,
			  "Unlock firmware when no loader available\n");
		return;
	}
	edgetpu_set_group_join_lockout(etdev, false);
	mutex_unlock(&et_fw->p->fw_desc_lock);
}

int edgetpu_firmware_run_locked(struct edgetpu_firmware *et_fw,
				const char *name,
				enum edgetpu_firmware_flags flags)
{
	const struct edgetpu_firmware_chip_data *chip_fw = et_fw->p->chip_fw;
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct edgetpu_firmware_desc new_fw_desc;
	int ret;
	bool is_bl1_run = (flags & FW_BL1);

	edgetpu_firmware_set_loading(et_fw);
	if (!is_bl1_run)
		edgetpu_sw_wdt_stop(etdev);

	memset(&new_fw_desc, 0, sizeof(new_fw_desc));
	ret = edgetpu_firmware_load_locked(et_fw, &new_fw_desc, name, flags);
	if (ret)
		goto out_failed;

	etdev_dbg(etdev, "run fw %s flags=%#x", name, flags);
	if (chip_fw->prepare_run) {
		/* Note this may recursively call us to run BL1 */
		ret = chip_fw->prepare_run(et_fw, &new_fw_desc.buf);
		if (ret)
			goto out_unload_new_fw;
	}

	/*
	 * Previous firmware buffer is not used anymore when the CPU runs on
	 * new firmware buffer. Unload this before et_fw->p->fw_buf is
	 * overwritten by new buffer information.
	 */
	if (!is_bl1_run) {
		edgetpu_firmware_unload_locked(et_fw, &et_fw->p->fw_desc);
		et_fw->p->fw_desc = new_fw_desc;
	} else {
		edgetpu_firmware_unload_locked(et_fw, &et_fw->p->bl1_fw_desc);
		et_fw->p->bl1_fw_desc = new_fw_desc;
	}

	ret = edgetpu_firmware_handshake(et_fw);

	/* Don't start wdt if loaded firmware is second stage bootloader. */
	if (!ret && !is_bl1_run && et_fw->p->fw_info.fw_flavor != FW_FLAVOR_BL1)
		edgetpu_sw_wdt_start(etdev);

	if (!ret && !is_bl1_run && chip_fw->launch_complete)
		chip_fw->launch_complete(et_fw);
	else if (ret && chip_fw->launch_failed)
		chip_fw->launch_failed(et_fw, ret);
	edgetpu_firmware_set_state(et_fw, ret);
	/* If previous firmware was metrics v1-only reset that flag and probe this again. */
	if (etdev->usage_stats)
		etdev->usage_stats->use_metrics_v1 = false;
	return ret;

out_unload_new_fw:
	edgetpu_firmware_unload_locked(et_fw, &new_fw_desc);
out_failed:
	if (chip_fw->launch_failed)
		chip_fw->launch_failed(et_fw, ret);
	edgetpu_firmware_set_state(et_fw, ret);
	return ret;
}

int edgetpu_firmware_run(struct edgetpu_dev *etdev, const char *name,
			 enum edgetpu_firmware_flags flags)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	int ret;

	if (!et_fw)
		return -ENODEV;
	ret = edgetpu_firmware_load_lock(etdev);
	if (ret) {
		etdev_err(etdev, "%s: lock failed (%d)\n", __func__, ret);
		return ret;
	}
	/* will be overwritten when we successfully parse the f/w header */
	etdev->fw_version.kci_version = EDGETPU_INVALID_KCI_VERSION;
	ret = edgetpu_firmware_pm_get(et_fw);
	if (!ret) {
		ret = edgetpu_firmware_run_locked(et_fw, name, flags);
		edgetpu_pm_put(etdev->pm);
	}

	edgetpu_firmware_load_unlock(etdev);

	return ret;
}

int edgetpu_firmware_run_default_locked(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	const char *run_firmware_name =
		et_fw->p->chip_fw->default_firmware_name;

	if (firmware_name && *firmware_name)
		run_firmware_name = firmware_name;

	return edgetpu_firmware_run_locked(etdev->firmware, run_firmware_name,
					   FW_DEFAULT);
}

int edgetpu_firmware_run_default(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	const char *run_firmware_name =
		et_fw->p->chip_fw->default_firmware_name;

	if (firmware_name && *firmware_name)
		run_firmware_name = firmware_name;

	return edgetpu_firmware_run(etdev, run_firmware_name, FW_DEFAULT);
}

bool edgetpu_firmware_is_loading(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;

	return et_fw && et_fw->p->status == FW_LOADING;
}

/* Caller must hold firmware lock. */
enum edgetpu_firmware_status
edgetpu_firmware_status_locked(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;

	if (!et_fw)
		return FW_INVALID;
	return et_fw->p->status;
}

/* Caller must hold firmware lock. For unit tests. */
void
edgetpu_firmware_set_status_locked(struct edgetpu_dev *etdev,
				   enum edgetpu_firmware_status status)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;

	if (et_fw)
		et_fw->p->status = status;
}

/* Caller must hold firmware lock for loading. */
int edgetpu_firmware_restart_locked(struct edgetpu_dev *etdev, bool force_reset)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	const struct edgetpu_firmware_chip_data *chip_fw = et_fw->p->chip_fw;
	int ret = -1;

	edgetpu_firmware_set_loading(et_fw);
	edgetpu_sw_wdt_stop(etdev);
	/*
	 * Try restarting the firmware first, fall back to normal firmware start
	 * if this fails.
	 */
	if (chip_fw->restart)
		ret = chip_fw->restart(et_fw, force_reset);
	if (ret && chip_fw->prepare_run) {
		ret = chip_fw->prepare_run(et_fw, &et_fw->p->fw_desc.buf);
		if (ret)
			goto out;
	}
	ret = edgetpu_firmware_handshake(et_fw);
	if (!ret)
		edgetpu_sw_wdt_start(etdev);
out:
	edgetpu_firmware_set_state(et_fw, ret);
	return ret;
}

ssize_t edgetpu_firmware_get_name(struct edgetpu_dev *etdev, char *buf,
				  size_t buflen)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	int ret;
	const char *fw_name;

	if (!et_fw)
		goto fw_none;

	mutex_lock(&et_fw->p->fw_desc_lock);
	if (edgetpu_firmware_status_locked(etdev) != FW_VALID)
		goto unlock_fw_none;
	fw_name = et_fw->p->fw_desc.buf.name;
	if (!fw_name)
		goto unlock_fw_none;
	ret = scnprintf(buf, buflen, "%s\n", fw_name);
	mutex_unlock(&et_fw->p->fw_desc_lock);
	return ret;

unlock_fw_none:
	mutex_unlock(&et_fw->p->fw_desc_lock);
fw_none:
	return scnprintf(buf, buflen, "[none]\n");
}

static ssize_t load_firmware_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_firmware_get_name(etdev, buf, PAGE_SIZE);
}

static ssize_t load_firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_firmware *et_fw = etdev->firmware;
	int ret;
	char *name;

	if (!et_fw)
		return -ENODEV;

	name = edgetpu_fwutil_name_from_attr_buf(buf);
	if (IS_ERR(name))
		return PTR_ERR(name);

	etdev_info(etdev, "loading firmware %s\n", name);
	ret = edgetpu_firmware_run(etdev, name, 0);

	kfree(name);

	if (ret)
		return ret;
	return count;
}

static DEVICE_ATTR_RW(load_firmware);

static ssize_t firmware_type_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_firmware *et_fw = etdev->firmware;
	int ret;

	if (!et_fw)
		return -ENODEV;
	ret = scnprintf(buf, PAGE_SIZE, "%s\n",
			fw_flavor_str(et_fw->p->fw_info.fw_flavor));
	return ret;
}
static DEVICE_ATTR_RO(firmware_type);

static ssize_t firmware_version_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_firmware *et_fw = etdev->firmware;
	int ret;

	if (!et_fw)
		return -ENODEV;

	if (etdev->fw_version.kci_version == EDGETPU_INVALID_KCI_VERSION)
		ret = -ENODATA;
	else
		ret = scnprintf(buf, PAGE_SIZE, "%u.%u vii=%u kci=%u cl=%u\n",
				etdev->fw_version.major_version,
				etdev->fw_version.minor_version,
				etdev->fw_version.vii_version,
				etdev->fw_version.kci_version,
				et_fw->p->fw_info.fw_changelist);
	return ret;
}
static DEVICE_ATTR_RO(firmware_version);

static struct attribute *dev_attrs[] = {
	&dev_attr_load_firmware.attr,
	&dev_attr_firmware_type.attr,
	&dev_attr_firmware_version.attr,
	NULL,
};

static const struct attribute_group edgetpu_firmware_attr_group = {
	.attrs = dev_attrs,
};

static void edgetpu_firmware_wdt_timeout_action(void *data)
{
	int ret;
	struct edgetpu_dev *etdev = data;
	struct edgetpu_firmware *et_fw = etdev->firmware;

	etdev->watchdog_timeout_count++;
	/* Don't attempt f/w restart if device is off. */
	if (!edgetpu_is_powered(etdev))
		return;

	/*
	 * Zero the FW state of open mailboxes so that when the runtime releases
	 * groups the CLOSE_DEVICE KCIs won't be sent.
	 */
	edgetpu_handshake_clear_fw_state(&etdev->mailbox_manager->open_devices);
	edgetpu_fatal_error_notify(etdev, EDGETPU_ERROR_WATCHDOG_TIMEOUT);

	/* Another procedure is loading the firmware, let it do the work. */
	if (edgetpu_firmware_is_loading(etdev))
		return;

	/* edgetpu_firmware_lock() here never fails */
	edgetpu_firmware_lock(etdev);

	ret = edgetpu_firmware_pm_get(et_fw);
	if (!ret) {
		ret = edgetpu_firmware_restart_locked(etdev, true);
		edgetpu_pm_put(etdev->pm);
	}
	edgetpu_firmware_unlock(etdev);
}

int edgetpu_firmware_create(struct edgetpu_dev *etdev,
			    const struct edgetpu_firmware_chip_data *chip_fw)
{
	struct edgetpu_firmware *et_fw;
	int ret;

	if (etdev->firmware)
		return -EBUSY;

	et_fw = kzalloc(sizeof(*et_fw), GFP_KERNEL);
	if (!et_fw)
		return -ENOMEM;
	et_fw->etdev = etdev;

	et_fw->p = kzalloc(sizeof(*et_fw->p), GFP_KERNEL);
	if (!et_fw->p) {
		ret = -ENOMEM;
		goto out_kfree_et_fw;
	}
	et_fw->p->chip_fw = chip_fw;

	mutex_init(&et_fw->p->fw_desc_lock);

	ret = device_add_group(etdev->dev, &edgetpu_firmware_attr_group);
	if (ret)
		goto out_kfree_et_fw_p;

	if (chip_fw->after_create) {
		ret = chip_fw->after_create(et_fw);
		if (ret) {
			etdev_dbg(etdev,
				  "%s: after create handler failed: %d\n",
				  __func__, ret);
			goto out_device_remove_group;
		}
	}

	etdev->firmware = et_fw;
	ret = edgetpu_sw_wdt_create(etdev, EDGETPU_ACTIVE_DEV_BEAT_MS,
				    EDGETPU_DORMANT_DEV_BEAT_MS);
	if (ret)
		etdev_err(etdev, "Failed to create sw wdt instance\n");
	else
		edgetpu_sw_wdt_set_handler(
			etdev, edgetpu_firmware_wdt_timeout_action, etdev);
	edgetpu_firmware_tracing_init(et_fw);
	return 0;

out_device_remove_group:
	device_remove_group(etdev->dev, &edgetpu_firmware_attr_group);
out_kfree_et_fw_p:
	kfree(et_fw->p);
out_kfree_et_fw:
	kfree(et_fw);
	return ret;
}

void edgetpu_firmware_destroy(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	const struct edgetpu_firmware_chip_data *chip_fw;

	if (!et_fw)
		return;
	edgetpu_sw_wdt_destroy(etdev);

	if (et_fw->p) {
		chip_fw = et_fw->p->chip_fw;
		/*
		 * Platform specific implementation, which includes stop
		 * running firmware.
		 */
		if (chip_fw->before_destroy)
			chip_fw->before_destroy(et_fw);
	}

	device_remove_group(etdev->dev, &edgetpu_firmware_attr_group);

	if (et_fw->p) {
		mutex_lock(&et_fw->p->fw_desc_lock);
		edgetpu_firmware_unload_locked(et_fw, &et_fw->p->fw_desc);
		edgetpu_firmware_unload_locked(et_fw, &et_fw->p->bl1_fw_desc);
		mutex_unlock(&et_fw->p->fw_desc_lock);
		edgetpu_firmware_tracing_destroy(et_fw);
	}

	etdev->firmware = NULL;

	kfree(et_fw->p);
	kfree(et_fw);
}

/* debugfs mappings dump */
void edgetpu_firmware_mappings_show(struct edgetpu_dev *etdev,
				    struct seq_file *s)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	struct edgetpu_firmware_buffer *fw_buf;
	phys_addr_t fw_iova_target;
	unsigned long iova;

	if (!et_fw)
		return;
	fw_buf = &et_fw->p->fw_desc.buf;
	if (!fw_buf->vaddr)
		return;
	fw_iova_target = fw_buf->dram_tpa ? fw_buf->dram_tpa : fw_buf->dma_addr;
	iova = edgetpu_chip_firmware_iova(etdev);
	seq_printf(s, "  %#lx %lu fw - %pad %s\n", iova,
		   DIV_ROUND_UP(fw_buf->alloc_size, PAGE_SIZE), &fw_iova_target,
		   fw_buf->flags & FW_ONDEV ? "dev" : "");
}
