// SPDX-License-Identifier: GPL-2.0
/*
 * File operations for EdgeTPU ML accel chips.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>

#include "edgetpu-config.h"
#include "edgetpu-device-group.h"
#include "edgetpu-dmabuf.h"
#include "edgetpu-dram.h"
#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mapping.h"
#include "edgetpu-pm.h"
#include "edgetpu-telemetry.h"
#include "edgetpu-wakelock.h"
#include "edgetpu.h"

#ifdef EDGETPU_FEATURE_INTEROP
#include <soc/google/tpu-ext.h>
#endif

#define CREATE_TRACE_POINTS
#include <trace/events/edgetpu.h>

#define DRIVER_VERSION "1.0"

static struct class *edgetpu_class;
static dev_t edgetpu_basedev;
static atomic_t char_minor = ATOMIC_INIT(-1);

static struct dentry *edgetpu_debugfs_dir;

#define LOCK(client) mutex_lock(&client->group_lock)
#define UNLOCK(client) mutex_unlock(&client->group_lock)
/*
 * Locks @client->group_lock and checks whether @client is the leader of a group.
 * If @client is not the leader of a group, unlocks group_lock and returns false.
 * If @client is the leader of a group, returns true with group_lock held.
 */
static inline bool lock_check_group_leader(struct edgetpu_client *client)
{
	LOCK(client);
	if (!client->group || !edgetpu_device_group_is_leader(client->group, client)) {
		UNLOCK(client);
		return false;
	}
	return true;
}

bool is_edgetpu_file(struct file *file)
{
	if (edgetpu_is_external_wrapper_class_file(file))
		return true;
	return file->f_op == &edgetpu_fops;
}

int edgetpu_open(struct edgetpu_dev_iface *etiface, struct file *file)
{
	struct edgetpu_client *client;

	/* Set client pointer to NULL if error creating client. */
	file->private_data = NULL;
	client = edgetpu_client_add(etiface);
	if (IS_ERR(client))
		return PTR_ERR(client);
	file->private_data = client;
	return 0;
}

static int edgetpu_fs_open(struct inode *inode, struct file *file)
{
	struct edgetpu_dev_iface *etiface =
		container_of(inode->i_cdev, struct edgetpu_dev_iface, cdev);

	return edgetpu_open(etiface, file);
}

static int edgetpu_fs_release(struct inode *inode, struct file *file)
{
	struct edgetpu_client *client = file->private_data;

	if (!client)
		return 0;

	edgetpu_client_remove(client);

	return 0;
}

static int edgetpu_ioctl_set_eventfd(struct edgetpu_client *client,
				     struct edgetpu_event_register __user *argp)
{
	int ret;
	struct edgetpu_event_register eventreg;

	if (copy_from_user(&eventreg, argp, sizeof(eventreg)))
		return -EFAULT;

	if (!lock_check_group_leader(client))
		return -EINVAL;
	ret = edgetpu_group_set_eventfd(client->group, eventreg.event_id, eventreg.eventfd);
	UNLOCK(client);
	return ret;
}

static int edgetpu_ioctl_unset_eventfd(struct edgetpu_client *client,
				       uint event_id)
{
	if (!lock_check_group_leader(client))
		return -EINVAL;
	edgetpu_group_unset_eventfd(client->group, event_id);
	UNLOCK(client);
	return 0;
}

static int
edgetpu_ioctl_set_perdie_eventfd(struct edgetpu_client *client,
				 struct edgetpu_event_register __user *argp)
{
	struct edgetpu_dev *etdev = client->etdev;
	struct edgetpu_event_register eventreg;

	if (copy_from_user(&eventreg, argp, sizeof(eventreg)))
		return -EFAULT;

	if (perdie_event_id_to_num(eventreg.event_id) >=
	    EDGETPU_NUM_PERDIE_EVENTS)
		return -EINVAL;
	client->perdie_events |= 1 << perdie_event_id_to_num(eventreg.event_id);

	switch (eventreg.event_id) {
	case EDGETPU_PERDIE_EVENT_LOGS_AVAILABLE:
		return edgetpu_telemetry_set_event(etdev, EDGETPU_TELEMETRY_LOG,
						   eventreg.eventfd);
	case EDGETPU_PERDIE_EVENT_TRACES_AVAILABLE:
		return edgetpu_telemetry_set_event(
			etdev, EDGETPU_TELEMETRY_TRACE, eventreg.eventfd);
	default:
		return -EINVAL;
	}
}

static int edgetpu_ioctl_unset_perdie_eventfd(struct edgetpu_client *client,
					      uint event_id)
{
	struct edgetpu_dev *etdev = client->etdev;

	if (perdie_event_id_to_num(event_id) >= EDGETPU_NUM_PERDIE_EVENTS)
		return -EINVAL;
	client->perdie_events &= ~(1 << perdie_event_id_to_num(event_id));

	switch (event_id) {
	case EDGETPU_PERDIE_EVENT_LOGS_AVAILABLE:
		edgetpu_telemetry_unset_event(etdev, EDGETPU_TELEMETRY_LOG);
		break;
	case EDGETPU_PERDIE_EVENT_TRACES_AVAILABLE:
		edgetpu_telemetry_unset_event(etdev, EDGETPU_TELEMETRY_TRACE);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int edgetpu_ioctl_finalize_group(struct edgetpu_client *client)
{
	struct edgetpu_device_group *group;
	int ret;

	/* Finalization has to be performed with device on. */
	ret = edgetpu_pm_get(client->etdev->pm);
	if (ret) {
		etdev_err(client->etdev, "finalize group: power on failed: %d", ret);
		return ret;
	}

	LOCK(client);
	group = client->group;
	if (!group || !edgetpu_device_group_is_leader(group, client))
		goto out_unlock;

	/*
	 * Hold the wakelock since we need to decide whether VII should be
	 * initialized during finalization.
	 */
	edgetpu_wakelock_lock(client->wakelock);
	ret = edgetpu_device_group_finalize(group);
	edgetpu_wakelock_unlock(client->wakelock);
out_unlock:
	UNLOCK(client);
	edgetpu_pm_put(client->etdev->pm);
	return ret;
}

static int edgetpu_ioctl_join_group(struct edgetpu_client *client,
				    u64 leader_fd)
{
	struct fd f = fdget(leader_fd);
	struct file *file = f.file;
	struct edgetpu_client *leader;
	int ret;
	struct edgetpu_device_group *group;

	if (!file) {
		ret = -EBADF;
		goto out;
	}
	if (!is_edgetpu_file(file)) {
		ret = -EINVAL;
		goto out;
	}
	leader = file->private_data;
	if (!leader) {
		ret = -EINVAL;
		goto out;
	}
	mutex_lock(&leader->group_lock);
	if (!leader->group ||
	    !edgetpu_device_group_is_leader(leader->group, leader)) {
		ret = -EINVAL;
		mutex_unlock(&leader->group_lock);
		goto out;
	}
	group = edgetpu_device_group_get(leader->group);
	mutex_unlock(&leader->group_lock);

	ret = edgetpu_device_group_add(group, client);
	edgetpu_device_group_put(group);
out:
	fdput(f);
	return ret;
}

static int edgetpu_ioctl_create_group(struct edgetpu_client *client,
				      struct edgetpu_mailbox_attr __user *argp)
{
	struct edgetpu_mailbox_attr attr;
	struct edgetpu_device_group *group;

	if (copy_from_user(&attr, argp, sizeof(attr)))
		return -EFAULT;

	group = edgetpu_device_group_alloc(client, &attr);
	if (IS_ERR(group))
		return PTR_ERR(group);

	edgetpu_device_group_put(group);
	return 0;
}

static int edgetpu_ioctl_map_buffer(struct edgetpu_client *client,
				    struct edgetpu_map_ioctl __user *argp)
{
	struct edgetpu_device_group *group;
	struct edgetpu_map_ioctl ibuf;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	trace_edgetpu_map_buffer_start(&ibuf);

	if (!lock_check_group_leader(client))
		return -EINVAL;
	/* to prevent group being released when we perform map/unmap later */
	group = edgetpu_device_group_get(client->group);
	/*
	 * Don't hold @client->group_lock on purpose since
	 * 1. We don't care whether @client still belongs to @group.
	 * 2. get_user_pages_fast called by edgetpu_device_group_map() will hold
	 *    mm->mmap_sem, we need to prevent our locks being held around it.
	 */
	UNLOCK(client);
	ret = edgetpu_device_group_map(group, &ibuf);
	if (ret)
		goto out;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		edgetpu_device_group_unmap(group, ibuf.die_index,
					   ibuf.device_address,
					   EDGETPU_MAP_SKIP_CPU_SYNC);
		ret = -EFAULT;
	}

out:
	edgetpu_device_group_put(group);
	trace_edgetpu_map_buffer_end(&ibuf);

	return ret;
}

static int edgetpu_ioctl_unmap_buffer(struct edgetpu_client *client,
				      struct edgetpu_map_ioctl __user *argp)
{
	struct edgetpu_map_ioctl ibuf;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (!lock_check_group_leader(client))
		return -EINVAL;
	ret = edgetpu_device_group_unmap(client->group, ibuf.die_index, ibuf.device_address,
					 ibuf.flags);
	UNLOCK(client);
	return ret;
}

static int
edgetpu_ioctl_allocate_device_buffer(struct edgetpu_client *client, u64 size)
{
#ifndef EDGETPU_HAS_DEVICE_DRAM
	return -ENOTTY;
#else
	return edgetpu_device_dram_getfd(client, size);
#endif /* EDGETPU_HAS_DEVICE_DRAM */
}

static int edgetpu_ioctl_sync_buffer(struct edgetpu_client *client,
				     struct edgetpu_sync_ioctl __user *argp)
{
	int ret;
	struct edgetpu_sync_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;
	if (!lock_check_group_leader(client))
		return -EINVAL;
	ret = edgetpu_device_group_sync_buffer(client->group, &ibuf);
	UNLOCK(client);
	return ret;
}

static int
edgetpu_ioctl_map_dmabuf(struct edgetpu_client *client,
			 struct edgetpu_map_dmabuf_ioctl __user *argp)
{
	struct edgetpu_device_group *group;
	struct edgetpu_map_dmabuf_ioctl ibuf;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	trace_edgetpu_map_dmabuf_start(&ibuf);

	if (!lock_check_group_leader(client))
		return -EINVAL;
	/* to prevent group being released when we perform unmap on fault */
	group = edgetpu_device_group_get(client->group);
	ret = edgetpu_map_dmabuf(group, &ibuf);
	UNLOCK(client);
	if (ret)
		goto out;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		edgetpu_unmap_dmabuf(group, ibuf.die_index,
				     ibuf.device_address);
		ret = -EFAULT;
	}

out:
	edgetpu_device_group_put(group);
	trace_edgetpu_map_dmabuf_end(&ibuf);

	return ret;
}

static int
edgetpu_ioctl_unmap_dmabuf(struct edgetpu_client *client,
			   struct edgetpu_map_dmabuf_ioctl __user *argp)
{
	int ret;
	struct edgetpu_map_dmabuf_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;
	if (!lock_check_group_leader(client))
		return -EINVAL;
	ret = edgetpu_unmap_dmabuf(client->group, ibuf.die_index, ibuf.device_address);
	UNLOCK(client);
	return ret;
}

static int edgetpu_ioctl_sync_fence_create(
	struct edgetpu_client *client,
	struct edgetpu_create_sync_fence_data __user *datap)
{
	struct edgetpu_create_sync_fence_data data;
	int ret;

	if (copy_from_user(&data, (void __user *)datap, sizeof(data)))
		return -EFAULT;
	LOCK(client);
	if (!client->group) {
		etdev_err(client->etdev, "client creating sync fence not joined to a device group");
		UNLOCK(client);
		return -EINVAL;
	}
	ret = edgetpu_sync_fence_create(client->group, &data);
	UNLOCK(client);
	if (ret)
		return ret;
	if (copy_to_user((void __user *)datap, &data, sizeof(data)))
		ret = -EFAULT;
	return ret;
}

static int edgetpu_ioctl_sync_fence_signal(
	struct edgetpu_signal_sync_fence_data __user *datap)
{
	struct edgetpu_signal_sync_fence_data data;

	if (copy_from_user(&data, (void __user *)datap, sizeof(data)))
		return -EFAULT;
	return edgetpu_sync_fence_signal(&data);
}

static int
edgetpu_ioctl_map_bulk_dmabuf(struct edgetpu_client *client,
			      struct edgetpu_map_bulk_dmabuf_ioctl __user *argp)
{
	struct edgetpu_device_group *group;
	struct edgetpu_map_bulk_dmabuf_ioctl ibuf;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (!lock_check_group_leader(client))
		return -EINVAL;
	/* to prevent group being released when we perform unmap on fault */
	group = edgetpu_device_group_get(client->group);
	ret = edgetpu_map_bulk_dmabuf(group, &ibuf);
	UNLOCK(client);
	if (ret)
		goto out;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		edgetpu_unmap_bulk_dmabuf(group, ibuf.device_address);
		ret = -EFAULT;
	}
out:
	edgetpu_device_group_put(group);
	return ret;
}

static int edgetpu_ioctl_unmap_bulk_dmabuf(
	struct edgetpu_client *client,
	struct edgetpu_map_bulk_dmabuf_ioctl __user *argp)
{
	int ret;
	struct edgetpu_map_bulk_dmabuf_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;
	if (!lock_check_group_leader(client))
		return -EINVAL;
	ret = edgetpu_unmap_bulk_dmabuf(client->group, ibuf.device_address);
	UNLOCK(client);
	return ret;
}

static int edgetpu_ioctl_sync_fence_status(
	struct edgetpu_sync_fence_status __user *datap)
{
	struct edgetpu_sync_fence_status data;
	int ret;

	if (copy_from_user(&data, (void __user *)datap, sizeof(data)))
		return -EFAULT;
	ret = edgetpu_sync_fence_status(&data);
	if (ret)
		return ret;
	if (copy_to_user((void __user *)datap, &data, sizeof(data)))
		ret = -EFAULT;
	return ret;
}

static int edgetpu_ioctl_fw_version(struct edgetpu_dev *etdev,
				    struct edgetpu_fw_version __user *argp)
{
	if (etdev->fw_version.kci_version == EDGETPU_INVALID_KCI_VERSION)
		return -ENODEV;
	if (copy_to_user(argp, &etdev->fw_version, sizeof(*argp)))
		return -EFAULT;
	return 0;
}

static int edgetpu_ioctl_tpu_timestamp(struct edgetpu_client *client,
				       __u64 __user *argp)
{
	u64 timestamp;
	int ret = 0;

	if (!edgetpu_wakelock_lock(client->wakelock)) {
		edgetpu_wakelock_unlock(client->wakelock);
		ret = -EAGAIN;
	} else {
		timestamp = edgetpu_chip_tpu_timestamp(client->etdev);
		edgetpu_wakelock_unlock(client->wakelock);
		if (copy_to_user(argp, &timestamp, sizeof(*argp)))
			ret = -EFAULT;
	}
	return ret;
}

static bool edgetpu_ioctl_check_permissions(struct file *file, uint cmd)
{
	return file->f_mode & FMODE_WRITE;
}

static int edgetpu_ioctl_release_wakelock(struct edgetpu_client *client)
{
	int count;

	LOCK(client);
	edgetpu_wakelock_lock(client->wakelock);
	/* when NO_WAKELOCK: count should be 1 so here is a no-op */
	count = edgetpu_wakelock_release(client->wakelock);
	if (count < 0) {
		edgetpu_wakelock_unlock(client->wakelock);
		UNLOCK(client);
		return count;
	}
	if (!count) {
		if (client->group)
			edgetpu_group_close_and_detach_mailbox(client->group);
	}
	edgetpu_wakelock_unlock(client->wakelock);
	UNLOCK(client);
	edgetpu_pm_put(client->etdev->pm);
	etdev_dbg(client->etdev, "%s: wakelock req count = %u", __func__,
		  count);
	return 0;
}

static int edgetpu_ioctl_acquire_wakelock(struct edgetpu_client *client)
{
	int count;
	int ret;
	struct edgetpu_thermal *thermal = client->etdev->thermal;

	if (edgetpu_thermal_is_suspended(thermal)) {
		/* TPU is thermal suspended, so fail acquiring wakelock */
		etdev_warn_ratelimited(client->etdev,
		       "wakelock acquire rejected due to device thermal limit exceeded");
		return -EAGAIN;
	}

	ret = edgetpu_pm_get(client->etdev->pm);
	if (ret) {
		etdev_warn(client->etdev, "%s: pm_get failed (%d)",
			   __func__, ret);
		return ret;
	}

	LOCK(client);
	/*
	 * Update client PID; the client may have been passed from the
	 * edgetpu service that originally created it to a new process.
	 * By the time the client holds TPU wakelocks it will have been
	 * passed to the new owning process.
	 */
	client->pid = current->pid;
	client->tgid = current->tgid;
	edgetpu_wakelock_lock(client->wakelock);
	/* when NO_WAKELOCK: count should be 1 so here is a no-op */
	count = edgetpu_wakelock_acquire(client->wakelock);
	if (count < 0) {
		ret = count;
		goto error_wakelock_unlock;
	}
	if (!count) {
		if (client->group)
			ret = edgetpu_group_attach_and_open_mailbox(client->group);
		if (ret) {
			etdev_warn(client->etdev, "failed to attach mailbox: %d", ret);
			edgetpu_wakelock_release(client->wakelock);
			/* fall through to error handling below */
		}
	}

error_wakelock_unlock:
	edgetpu_wakelock_unlock(client->wakelock);
	UNLOCK(client);

	if (ret) {
		etdev_err(client->etdev, "client pid %d failed to acquire wakelock", client->pid);
		edgetpu_pm_put(client->etdev->pm);
	} else {
		etdev_dbg(client->etdev, "%s: wakelock req count = %u", __func__, count + 1);
	}

	return ret;
}

static int
edgetpu_ioctl_dram_usage(struct edgetpu_dev *etdev,
			 struct edgetpu_device_dram_usage __user *argp)
{
	struct edgetpu_device_dram_usage dram;

	dram.allocated = edgetpu_device_dram_used(etdev);
	dram.available = edgetpu_device_dram_available(etdev);
	if (copy_to_user(argp, &dram, sizeof(*argp)))
		return -EFAULT;
	return 0;
}

static int
edgetpu_ioctl_acquire_ext_mailbox(struct edgetpu_client *client,
				  struct edgetpu_ext_mailbox_ioctl __user *argp)
{
	struct edgetpu_ext_mailbox_ioctl ext_mailbox;
	int ret;

	if (copy_from_user(&ext_mailbox, argp, sizeof(ext_mailbox)))
		return -EFAULT;

	ret = edgetpu_chip_acquire_ext_mailbox(client, &ext_mailbox);
	if (ret)
		etdev_err(client->etdev, "client pid %d failed to acquire ext mailbox",
			  client->pid);
	return ret;
}

static int
edgetpu_ioctl_release_ext_mailbox(struct edgetpu_client *client,
				  struct edgetpu_ext_mailbox_ioctl __user *argp)
{
	struct edgetpu_ext_mailbox_ioctl ext_mailbox;

	if (copy_from_user(&ext_mailbox, argp, sizeof(ext_mailbox)))
		return -EFAULT;

	return edgetpu_chip_release_ext_mailbox(client, &ext_mailbox);
}

static int edgetpu_ioctl_get_fatal_errors(struct edgetpu_client *client,
					  __u32 __user *argp)
{
	u32 fatal_errors = 0;
	int ret = 0;

	LOCK(client);
	if (client->group)
		fatal_errors = edgetpu_group_get_fatal_errors(client->group);
	UNLOCK(client);
	if (copy_to_user(argp, &fatal_errors, sizeof(fatal_errors)))
		ret = -EFAULT;
	return ret;
}

long edgetpu_ioctl(struct file *file, uint cmd, ulong arg)
{
	struct edgetpu_client *client = file->private_data;
	void __user *argp = (void __user *)arg;
	long ret;

	if (!client)
		return -ENODEV;

	if (!edgetpu_ioctl_check_permissions(file, cmd))
		return -EPERM;

	switch (cmd) {
	case EDGETPU_MAP_BUFFER:
		ret = edgetpu_ioctl_map_buffer(client, argp);
		break;
	case EDGETPU_UNMAP_BUFFER:
		ret = edgetpu_ioctl_unmap_buffer(client, argp);
		break;
	case EDGETPU_SET_EVENTFD:
		ret = edgetpu_ioctl_set_eventfd(client, argp);
		break;
	case EDGETPU_CREATE_GROUP:
		ret = edgetpu_ioctl_create_group(client, argp);
		break;
	case EDGETPU_JOIN_GROUP:
		ret = edgetpu_ioctl_join_group(client, (u64)argp);
		break;
	case EDGETPU_FINALIZE_GROUP:
		ret = edgetpu_ioctl_finalize_group(client);
		break;
	case EDGETPU_SET_PERDIE_EVENTFD:
		ret = edgetpu_ioctl_set_perdie_eventfd(client, argp);
		break;
	case EDGETPU_UNSET_EVENT:
		ret = edgetpu_ioctl_unset_eventfd(client, arg);
		break;
	case EDGETPU_UNSET_PERDIE_EVENT:
		ret = edgetpu_ioctl_unset_perdie_eventfd(client, arg);
		break;
	case EDGETPU_SYNC_BUFFER:
		ret = edgetpu_ioctl_sync_buffer(client, argp);
		break;
	case EDGETPU_MAP_DMABUF:
		ret = edgetpu_ioctl_map_dmabuf(client, argp);
		break;
	case EDGETPU_UNMAP_DMABUF:
		ret = edgetpu_ioctl_unmap_dmabuf(client, argp);
		break;
	case EDGETPU_ALLOCATE_DEVICE_BUFFER:
		ret = edgetpu_ioctl_allocate_device_buffer(client, (u64)argp);
		break;
	case EDGETPU_CREATE_SYNC_FENCE:
		ret = edgetpu_ioctl_sync_fence_create(client, argp);
		break;
	case EDGETPU_SIGNAL_SYNC_FENCE:
		ret = edgetpu_ioctl_sync_fence_signal(argp);
		break;
	case EDGETPU_MAP_BULK_DMABUF:
		ret = edgetpu_ioctl_map_bulk_dmabuf(client, argp);
		break;
	case EDGETPU_UNMAP_BULK_DMABUF:
		ret = edgetpu_ioctl_unmap_bulk_dmabuf(client, argp);
		break;
	case EDGETPU_SYNC_FENCE_STATUS:
		ret = edgetpu_ioctl_sync_fence_status(argp);
		break;
	case EDGETPU_RELEASE_WAKE_LOCK:
		ret = edgetpu_ioctl_release_wakelock(client);
		break;
	case EDGETPU_ACQUIRE_WAKE_LOCK:
		ret = edgetpu_ioctl_acquire_wakelock(client);
		break;
	case EDGETPU_FIRMWARE_VERSION:
		ret = edgetpu_ioctl_fw_version(client->etdev, argp);
		break;
	case EDGETPU_GET_TPU_TIMESTAMP:
		ret = edgetpu_ioctl_tpu_timestamp(client, argp);
		break;
	case EDGETPU_GET_DRAM_USAGE:
		ret = edgetpu_ioctl_dram_usage(client->etdev, argp);
		break;
	case EDGETPU_ACQUIRE_EXT_MAILBOX:
		ret = edgetpu_ioctl_acquire_ext_mailbox(client, argp);
		break;
	case EDGETPU_RELEASE_EXT_MAILBOX:
		ret = edgetpu_ioctl_release_ext_mailbox(client, argp);
		break;
	case EDGETPU_GET_FATAL_ERRORS:
		ret = edgetpu_ioctl_get_fatal_errors(client, argp);
		break;
	default:
		return -ENOTTY; /* unknown command */
	}

	return ret;
}

static long edgetpu_fs_ioctl(struct file *file, uint cmd, ulong arg)
{
	return edgetpu_ioctl(file, cmd, arg);
}

/* Map a region of device/coherent memory. */
static int edgetpu_fs_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct edgetpu_client *client = file->private_data;

	if (!client)
		return -ENODEV;

	return edgetpu_mmap(client, vma);
}

#ifndef EDGETPU_FEATURE_MOBILE
static struct edgetpu_dumpregs_range common_statusregs_ranges[] = {
	{
		.firstreg = EDGETPU_REG_AON_RESET,
		.lastreg = EDGETPU_REG_AON_FORCE_QUIESCE,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_DMA_PAUSED,
		.lastreg = EDGETPU_REG_USER_HIB_DMA_PAUSED,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_ERROR_STATUS,
		.lastreg = EDGETPU_REG_USER_HIB_ERROR_MASK,
	},
	{
		.firstreg = EDGETPU_REG_SC_CURRENTPC,
		.lastreg = EDGETPU_REG_SC_DECODEPC,
	},
	{
		.firstreg = EDGETPU_REG_SC_ERROR,
		.lastreg = EDGETPU_REG_SC_ERROR_MASK,
	},
	{
		.firstreg = EDGETPU_REG_SC_ERROR_INFO,
		.lastreg = EDGETPU_REG_SC_ERROR_INFO,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_INSTRQ_TAIL,
		.lastreg = EDGETPU_REG_USER_HIB_INSTRQ_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_SC_HOST_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_SC_HOST_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_FATALERR_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_FATALERR_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_SNAPSHOT,
		.lastreg = EDGETPU_REG_USER_HIB_SNAPSHOT,
	},
};

static struct edgetpu_dumpregs_range common_tile_statusregs_ranges[] = {
	{
		.firstreg = EDGETPU_REG_TILECONF1_DEEPSLEEP,
		.lastreg = EDGETPU_REG_TILECONF1_DEEPSLEEP,
	},
	{
		.firstreg = EDGETPU_REG_TILECONF1_ERROR_TILE,
		.lastreg = EDGETPU_REG_TILECONF1_ERROR_MASK_TILE,
	},
	{
		.firstreg = EDGETPU_REG_TILECONF1_ERROR_INFO_TILE,
		.lastreg = EDGETPU_REG_TILECONF1_ERROR_INFO_TILE,
	},
};

static void dump_statusregs_ranges(
	struct seq_file *s, struct edgetpu_dev *etdev,
	struct edgetpu_dumpregs_range *ranges, int nranges)
{
	int i;
	enum edgetpu_csrs reg;
	uint64_t val;

	for (i = 0; i < nranges; i++) {
		for (reg = ranges[i].firstreg; reg <= ranges[i].lastreg;
		     reg += sizeof(val)) {
			val = edgetpu_dev_read_64(etdev, reg);
			seq_printf(s, "%#08x: %#016llx\n", reg, val);
		}
	}
}

static void dump_mboxes(struct seq_file *s, struct edgetpu_dev *etdev)
{
	enum edgetpu_csrs base;
	uint32_t val;
	int mbox_id;
	int n_p2p_mbox_dump = EDGETPU_NUM_P2P_MAILBOXES;

	/* Dump VII mailboxes plus P2P (if any) + KCI. */
	for (mbox_id = 0, base = EDGETPU_MBOX_BASE;
	     mbox_id < EDGETPU_NUM_VII_MAILBOXES + n_p2p_mbox_dump + 1;
	     mbox_id++, base += EDGETPU_MBOX_CSRS_SIZE) {
		int offset;

		for (offset = 0x0; offset <= 0x40; offset += sizeof(val)) {
			val = edgetpu_dev_read_32(etdev, base + offset);
			seq_printf(s, "%#08x: %#08x\n", base + offset, val);
		}
		for (offset = 0x1000; offset <= 0x1014; offset += sizeof(val)) {
			val = edgetpu_dev_read_32(etdev, base + offset);
			seq_printf(s, "%#08x: %#08x\n", base + offset, val);
		}
		for (offset = 0x1800; offset <= 0x1818; offset += sizeof(val)) {
			val = edgetpu_dev_read_32(etdev, base + offset);
			seq_printf(s, "%#08x: %#08x\n", base + offset, val);
		}
	}
}

static int statusregs_show(struct seq_file *s, void *data)
{
	struct edgetpu_dev *etdev = s->private;
	int tileid;

	dump_statusregs_ranges(s, etdev, common_statusregs_ranges,
			       ARRAY_SIZE(common_statusregs_ranges));
	dump_statusregs_ranges(s, etdev, edgetpu_chip_statusregs_ranges,
			       edgetpu_chip_statusregs_nranges);
	for (tileid = 0; tileid < EDGETPU_NTILES; tileid++) {
		edgetpu_dev_write_64(etdev, EDGETPU_REG_USER_HIB_TILECONFIG1,
				     tileid);
		seq_printf(s, "tile %d:\n", tileid);
		dump_statusregs_ranges(
			s, etdev, common_tile_statusregs_ranges,
			ARRAY_SIZE(common_tile_statusregs_ranges));
		dump_statusregs_ranges(s, etdev,
				       edgetpu_chip_tile_statusregs_ranges,
				       edgetpu_chip_tile_statusregs_nranges);
	}
	dump_mboxes(s, etdev);
	return 0;
}

static int statusregs_open(struct inode *inode, struct file *file)
{
	return single_open(file, statusregs_show, inode->i_private);
}

static const struct file_operations statusregs_ops = {
	.open = statusregs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
	.release = single_release,
};
#endif /* EDGETPU_FEATURE_MOBILE */

static int mappings_show(struct seq_file *s, void *data)
{
	struct edgetpu_dev *etdev = s->private;
	struct edgetpu_list_group *l;
	struct edgetpu_device_group *group;

	mutex_lock(&etdev->groups_lock);

	etdev_for_each_group(etdev, l, group)
		edgetpu_group_mappings_show(group, s);

	mutex_unlock(&etdev->groups_lock);
	edgetpu_kci_mappings_show(etdev, s);
	return 0;
}

static int mappings_open(struct inode *inode, struct file *file)
{
	return single_open(file, mappings_show, inode->i_private);
}

static const struct file_operations mappings_ops = {
	.open = mappings_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
	.release = single_release,
};

static int syncfences_open(struct inode *inode, struct file *file)
{
	return single_open(file, edgetpu_sync_fence_debugfs_show, inode->i_private);
}

static const struct file_operations syncfences_ops = {
	.open = syncfences_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
	.release = single_release,
};

static int edgetpu_pm_debugfs_set_wakelock(void *data, u64 val)
{
	struct edgetpu_dev *etdev = data;
	int ret = 0;

	if (val)
		ret = edgetpu_pm_get(etdev->pm);
	else
		edgetpu_pm_put(etdev->pm);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_wakelock, NULL, edgetpu_pm_debugfs_set_wakelock,
			 "%llu\n");

static void edgetpu_fs_setup_debugfs(struct edgetpu_dev *etdev)
{
	etdev->d_entry =
		debugfs_create_dir(etdev->dev_name, edgetpu_debugfs_dir);
	if (IS_ERR_OR_NULL(etdev->d_entry)) {
		etdev_warn(etdev, "Failed to setup debugfs\n");
		return;
	}
	debugfs_create_file("mappings", 0440, etdev->d_entry,
			    etdev, &mappings_ops);
	debugfs_create_file("syncfences", 0440, etdev->d_entry, etdev, &syncfences_ops);
	debugfs_create_file("wakelock", 0220, etdev->d_entry, etdev,
			    &fops_wakelock);
#ifndef EDGETPU_FEATURE_MOBILE
	debugfs_create_file("statusregs", 0440, etdev->d_entry, etdev,
			    &statusregs_ops);
#endif
}

static ssize_t firmware_crash_count_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", etdev->firmware_crash_count);
}
static DEVICE_ATTR_RO(firmware_crash_count);

static ssize_t watchdog_timeout_count_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", etdev->watchdog_timeout_count);
}
static DEVICE_ATTR_RO(watchdog_timeout_count);

static ssize_t clients_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_list_device_client *lc;
	ssize_t len;
	ssize_t ret = 0;

	mutex_lock(&etdev->clients_lock);
	for_each_list_device_client(etdev, lc) {
		struct edgetpu_device_group *group;

		mutex_lock(&lc->client->group_lock);
		group = lc->client->group;
		len = scnprintf(buf, PAGE_SIZE - ret,
				"pid %d tgid %d group %d wakelock %d\n",
				lc->client->pid, lc->client->tgid,
				group ? group->workload_id : -1,
				NO_WAKELOCK(lc->client->wakelock) ?
				0 : lc->client->wakelock->req_count);
		mutex_unlock(&lc->client->group_lock);
		buf += len;
		ret += len;
	}
	mutex_unlock(&etdev->clients_lock);
	return ret;
}
static DEVICE_ATTR_RO(clients);

static ssize_t show_group(struct edgetpu_dev *etdev,
			  struct edgetpu_device_group *group, char *buf,
			  ssize_t buflen)
{
	enum edgetpu_context_id context =
		edgetpu_group_context_id_locked(group);
	struct edgetpu_list_group_client *lc;
	ssize_t len;
	ssize_t ret = 0;

	len = scnprintf(buf, buflen - ret, "group %u ", group->workload_id);
	buf += len;
	ret += len;

	switch (group->status) {
	case EDGETPU_DEVICE_GROUP_WAITING:
		len = scnprintf(buf, buflen - ret, "forming ");
		buf += len;
		ret += len;
		break;
	case EDGETPU_DEVICE_GROUP_FINALIZED:
		break;
	case EDGETPU_DEVICE_GROUP_ERRORED:
		len = scnprintf(buf, buflen - ret, "error %#x ",
				group->fatal_errors);
		buf += len;
		ret += len;
		break;
	case EDGETPU_DEVICE_GROUP_DISBANDED:
		len = scnprintf(buf, buflen - ret, "disbanded\n");
		ret += len;
		return ret;
	}

	if (context == EDGETPU_CONTEXT_INVALID)
		len = scnprintf(buf, buflen - ret, "context (none) ");
	else if (context & EDGETPU_CONTEXT_DOMAIN_TOKEN)
		len = scnprintf(buf, buflen - ret, "context detached %#x ",
				context & ~(EDGETPU_CONTEXT_DOMAIN_TOKEN));
	else
		len = scnprintf(buf, buflen - ret, "context mbox %d ",
				context);
	buf += len;
	ret += len;
	len = scnprintf(buf, buflen - ret, "vcid %u %s%s\n",
			group->vcid, group->dev_inaccessible ? "i" : "",
			group->ext_mailbox ? "x" : "");
	buf += len;
	ret += len;

	for_each_list_group_client(lc, group) {
		len = scnprintf(buf, buflen - ret, "client %s %d:%d\n",
				lc->client->etiface->name,
				lc->client->pid, lc->client->tgid);
		buf += len;
		ret += len;
	}

	len = scnprintf(buf, buflen - ret, "mappings %zd %zdB\n",
			group->host_mappings.count +
			group->dmabuf_mappings.count,
			edgetpu_group_mappings_total_size(group));
	buf += len;
	ret += len;
	return ret;
}

static ssize_t groups_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_device_group *group;
	struct edgetpu_list_group *lg;
	ssize_t ret = 0;

	mutex_lock(&etdev->groups_lock);
	etdev_for_each_group(etdev, lg, group) {
		edgetpu_device_group_get(group);
		ret += show_group(etdev, group, buf + ret, PAGE_SIZE - ret);
		edgetpu_device_group_put(group);
	}
	mutex_unlock(&etdev->groups_lock);
	return ret;
}
static DEVICE_ATTR_RO(groups);

static struct attribute *edgetpu_dev_attrs[] = {
	&dev_attr_firmware_crash_count.attr,
	&dev_attr_watchdog_timeout_count.attr,
	&dev_attr_clients.attr,
	&dev_attr_groups.attr,
	NULL,
};

static const struct attribute_group edgetpu_attr_group = {
	.attrs = edgetpu_dev_attrs,
};

const struct file_operations edgetpu_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.mmap = edgetpu_fs_mmap,
	.open = edgetpu_fs_open,
	.release = edgetpu_fs_release,
	.unlocked_ioctl = edgetpu_fs_ioctl,
};

static int edgeptu_fs_add_interface(struct edgetpu_dev *etdev, struct edgetpu_dev_iface *etiface,
				    const struct edgetpu_iface_params *etiparams)
{
	char dev_name[EDGETPU_DEVICE_NAME_MAX];
	int ret;

	etiface->name = etiparams->name ? etiparams->name : etdev->dev_name;
	snprintf(dev_name, EDGETPU_DEVICE_NAME_MAX, "%s",
			 etiface->name);

	dev_dbg(etdev->dev, "adding interface: %s", dev_name);

	etiface->devno = MKDEV(MAJOR(edgetpu_basedev),
			     atomic_add_return(1, &char_minor));
	cdev_init(&etiface->cdev, &edgetpu_fops);
	ret = cdev_add(&etiface->cdev, etiface->devno, 1);
	if (ret) {
		dev_err(etdev->dev, "%s: error %d adding cdev for dev %d:%d\n",
			etdev->dev_name, ret, MAJOR(etiface->devno),
			MINOR(etiface->devno));
		return ret;
	}

	etiface->etcdev = device_create(edgetpu_class, etdev->dev, etiface->devno,
				      etdev, "%s", dev_name);
	if (IS_ERR(etiface->etcdev)) {
		ret = PTR_ERR(etiface->etcdev);
		dev_err(etdev->dev, "%s: failed to create char device: %d\n",
			dev_name, ret);
		cdev_del(&etiface->cdev);
		return ret;
	}

	if (etiparams->name)
		etiface->d_entry =
			debugfs_create_symlink(etiparams->name, edgetpu_debugfs_dir,
					       etdev->dev_name);
	return 0;
}

/* Called from edgetpu core to add new edgetpu device files. */
int edgetpu_fs_add(struct edgetpu_dev *etdev, const struct edgetpu_iface_params *etiparams,
		   int num_ifaces)
{
	int ret;
	int i;

	etdev->num_ifaces = 0;
	dev_dbg(etdev->dev, "%s: adding %u interfaces\n", __func__, num_ifaces);

	for (i = 0; i < num_ifaces; i++) {
		etdev->etiface[i].etdev = etdev;
		ret = edgeptu_fs_add_interface(etdev, &etdev->etiface[i], &etiparams[i]);
		if (ret)
			return ret;
		etdev->num_ifaces++;
	}

	ret = device_add_group(etdev->dev, &edgetpu_attr_group);
	edgetpu_fs_setup_debugfs(etdev);
	if (ret)
		etdev_warn(etdev, "edgetpu attr group create failed: %d", ret);
	return 0;
}

void edgetpu_fs_remove(struct edgetpu_dev *etdev)
{
	int i;
	device_remove_group(etdev->dev, &edgetpu_attr_group);
	for (i = 0; i < etdev->num_ifaces; i++) {
		struct edgetpu_dev_iface *etiface = &etdev->etiface[i];

		debugfs_remove(etiface->d_entry);
		device_destroy(edgetpu_class, etiface->devno);
		etiface->etcdev = NULL;
		cdev_del(&etiface->cdev);
	}
	debugfs_remove_recursive(etdev->d_entry);
}

static void edgetpu_debugfs_global_setup(void)
{
	edgetpu_debugfs_dir = debugfs_create_dir("edgetpu", NULL);
	if (IS_ERR_OR_NULL(edgetpu_debugfs_dir)) {
		pr_warn(DRIVER_NAME " error creating edgetpu debugfs dir\n");
		return;
	}
}

int __init edgetpu_fs_init(void)
{
	int ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
	edgetpu_class = class_create(THIS_MODULE, "edgetpu");
#else
	edgetpu_class = class_create("edgetpu");
#endif
	if (IS_ERR(edgetpu_class)) {
		pr_err(DRIVER_NAME " error creating edgetpu class: %ld\n",
		       PTR_ERR(edgetpu_class));
		return PTR_ERR(edgetpu_class);
	}

	ret = alloc_chrdev_region(&edgetpu_basedev, 0, EDGETPU_DEV_MAX,
				  DRIVER_NAME);
	if (ret) {
		pr_err(DRIVER_NAME " char driver registration failed: %d\n",
		       ret);
		class_destroy(edgetpu_class);
		return ret;
	}
	pr_debug(DRIVER_NAME " registered major=%d\n", MAJOR(edgetpu_basedev));
	edgetpu_debugfs_global_setup();
	return 0;
}

void __exit edgetpu_fs_exit(void)
{
	debugfs_remove_recursive(edgetpu_debugfs_dir);
	unregister_chrdev_region(edgetpu_basedev, EDGETPU_DEV_MAX);
	class_destroy(edgetpu_class);
}

struct dentry *edgetpu_fs_debugfs_dir(void)
{
	return edgetpu_debugfs_dir;
}

MODULE_DESCRIPTION("Google EdgeTPU file operations");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
#ifdef GIT_REPO_TAG
MODULE_INFO(gitinfo, GIT_REPO_TAG);
#endif
