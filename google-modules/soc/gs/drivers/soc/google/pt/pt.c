// SPDX-License-Identifier: GPL-2.0-only
/*
 * pt.c
 *
 * Partition API for system cache management API.
 *
 * Copyright 2019 Google LLC
 *
 * Author: cozette@google.com
 */

#include <soc/google/pt.h>
#include "pt_trace.h"
#include <linux/list.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched/clock.h>
#include <linux/vmalloc.h>
#include <asm/sysreg.h>


#define PT_SYSCTL_ENTRY 9
#define PT_COMMAND_SIZE 128
#define PT_COMMAND_SIZE_STR "127"

struct pt_pts {
	bool enabled;
	u32 size; /* current size of the partition, 0 if disabled */
	ptid_t ptid; /* partition index */
	int property_index; /* index in the driver properties */
	struct pt_handle *handle;
	struct pt_driver *driver; /* driver managing this partition */
	struct list_head resize_list; /* resize_thread callback list */
};

struct pt_properties {
	struct device_node **nodes;
	int cnt; /* Number of nodes */
};

/*
 * Locking:
 * - Driver are not thread safe:
 *      All access to driver (except pbha) are protected by the driver->mt
 *	(->alloc()/->enable()/->disable()/->free())
 *	These access aways happen while a handle->mt is also taken
 * - Handle data and driver data are consistent,
 *      All access to handle are protected by handle->mt
 *      (pt_client_*())
 * - Handle list and pts are changed atomically
 *      Adding/removing handle and changing pts are protected by
 *	pt_internal_data->sl
 * - resize_callback() could happen in any context and handle->mt or
 *	driver->mt can be taken. They will be forwarded to resize_thread.
 */

struct pt_driver { /* one per driver */
	/* partition properties in driver node */
	struct pt_properties *properties;
	struct list_head list;
	const struct pt_ops *ops;
	spinlock_t lock; /* serialize access to the driver */
	int ref;
	int ioctl_ret; /* return of last successful ops->ioctl() */
	struct device_node *node; /* driver node */
	void *data; /* driver private data */
	struct ctl_table_header *sysctl_header;
	struct ctl_table sysctl_table[PT_SYSCTL_ENTRY];
};

struct pt_global {
	ptpbha_t pbha;
	ptid_t ptid;
	int property_index;
	struct pt_driver *driver;
};

struct {
	struct list_head driver_list;
	struct list_head handle_list;
	struct ctl_table_header *sysctl_header;
	spinlock_t sl;
	char sysctl_command[PT_COMMAND_SIZE];
	char sysctl_node_name[PT_COMMAND_SIZE];
	struct ctl_table sysctl_table[PT_SYSCTL_ENTRY];
	u32 timestamp;
	u32 size;
	int enabled;

	/* Global pids */
	struct pt_global global_leftover;
	struct pt_global global_bypass;

	/* Data for resize_callback thread */
	struct task_struct *resize_thread;
	struct list_head resize_list; /* callback to call */
	wait_queue_head_t resize_wq; /* wait for new callback */
} pt_internal_data;

enum pt_fn {
	PT_FN_REGISTER = 0,
	PT_FN_ENABLE = 1,
	PT_FN_DISABLE = 2,
	PT_FN_UNREGISTER = 3,
	PT_FN_RESIZE = 4,
	PT_FN_MUTATE = 5,
	PT_FN_DISABLE_NO_FREE = 6,
	PT_FN_FREE = 7,
	PT_FN_TEST = 8,
	PT_FN_GLOBAL_PID = 9,
	PT_FN_GLOBAL_PBHA = 10,
	PT_FN_IOCTL = 11,
};

static void pt_trace(struct pt_handle *handle, int id, bool enable)
{
	struct pt_properties *properties;
	int property_index;
	const char *name;
	struct pt_driver *driver;
	ptid_t ptid;

	ptid = handle->pts[id].ptid;
	driver = handle->pts[id].driver;
	properties = driver->properties;
	property_index = handle->pts[id].property_index;
	name = properties->nodes[property_index]->name;
	trace_pt_enable(handle->node->name, name, enable, ptid);
}

/*
 * Get the next resize callback pts.
 * Wait for new pts if resize_list empty
 * Wake up thread waiting for the last in progress to be completed.
 */
static struct pt_pts *pt_resize_list_next(u32 *size)
{
	struct pt_pts *pts = NULL;

	lockdep_assert_held(&pt_internal_data.sl);

	if (!list_empty(&pt_internal_data.resize_list)) {
		pts = list_first_entry(&pt_internal_data.resize_list,
					struct pt_pts, resize_list);
		list_del(&pts->resize_list);
		pts->resize_list.next = NULL;
		pts->resize_list.prev = NULL;
		*size = pts->size;
	}

	return pts;
}

/*
 * Add a new resize callback pts.
 * Wake up resize_thread if needed
 */
static void pt_resize_list_add(struct pt_pts *pts, u32 size)
{
	bool waking = false;
	unsigned long flags;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	if ((pts->resize_list.next == NULL) && (pts->enabled)
		&& (pts->size != size)) {
		waking = list_empty(&pt_internal_data.resize_list);
		list_add(&pts->resize_list, &pt_internal_data.resize_list);
	}
	pts->size = size;
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);

	if (waking)
		wake_up(&pt_internal_data.resize_wq);
}

/*
 * Flush and disable pts resize callback.
 * If a resize callback is in progress, we wait for its completion.
 */
static bool pt_resize_list_disable(struct pt_pts *pts)
{
	unsigned long flags;
	bool enabled;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	enabled = pts->enabled;
	pts->enabled = false;
	if (pts->resize_list.next != NULL) {
		list_del(&pts->resize_list);
		pts->resize_list.next = NULL;
		pts->resize_list.prev = NULL;
		pts->size = 0;
	}
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);

	return enabled;
}

/*
 * Allow resize callback on the pts.
 */
static void pt_resize_list_enable(struct pt_pts *pts)
{
	unsigned long flags;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	pts->enabled = true;
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);
}

/*
 * Thread calling the resize callback.
 * This thread doesn't hold any lock, so the callback can
 * call pt_client_*()
 */
static int pt_resize_thread(void *data)
{
	u32 size;
	struct pt_pts *pts;
	struct pt_handle *handle;
	struct pt_driver *driver;
	pt_resize_callback_t resize_callback = NULL;
	int id;

	while (!kthread_should_stop()) {
		spin_lock_irq(&pt_internal_data.sl);

		pts = pt_resize_list_next(&size);
		if (pts != NULL) {
			handle = pts->handle;
			resize_callback = handle->resize_callback;
			id = ((char *)pts - (char *)handle->pts) / sizeof(handle->pts[0]);
			resize_callback(handle->data, id, size);
			driver = pts->driver;
			trace_pt_resize_callback(
				handle->node->name,
				driver->properties->nodes[pts->property_index]->name, false,
				(int)size, pts->ptid);
		}

		spin_unlock_irq(&pt_internal_data.sl);

		/* List was empty, wait to be notified by pt_resize_list_add */
		if (pts == NULL) {
			int ret;
			do {
				ret = wait_event_interruptible(
					pt_internal_data.resize_wq,
					!list_empty(&pt_internal_data.resize_list) ||
						kthread_should_stop());
			} while (ret);
		}
	}

	return 0;
}

/*
 * Resize callback used for global partition
 */
static void pt_resize_internal_nop(void *data, size_t size)
{
}

static void pt_resize_internal(void *data, size_t size)
{
	struct pt_pts *pts = (struct pt_pts *)data;
	struct pt_handle *handle = pts->handle;

	trace_pt_resize_callback(pts->handle->node->name,
		pts->driver->properties->nodes[pts->property_index]->name,
		true, (int)size, pts->ptid);
	if (handle->resize_callback) {
		pt_resize_list_add(pts, size);
	} else {
		/* pts->size is usually set by pt_resize_list_add(). If we
		 * don't call pt_resize_list_add(), we need to set pts->size
		 * here.
		 */
		unsigned long flags;

		spin_lock_irqsave(&pt_internal_data.sl, flags);
		pts->size = size;
		spin_unlock_irqrestore(&pt_internal_data.sl, flags);
	}
}

/*
 * Helper for driver->alloc()
 */
static bool pt_driver_alloc(struct pt_handle *handle, int id)
{
	ptid_t ptid;
	struct pt_driver *driver = handle->pts[id].driver;
	void *data = driver->data;
	int property_index = handle->pts[id].property_index;
	unsigned long flags;

	spin_lock_irqsave(&driver->lock, flags);
	ptid = driver->ops->alloc(data,
				  property_index,
				  &handle->pts[id],
				  pt_resize_internal);
	if (ptid != PT_PTID_INVALID)
		handle->pts[id].ptid = ptid;
	spin_unlock_irqrestore(&driver->lock, flags);
	return ptid != PT_PTID_INVALID;
}

static void pt_driver_enable(struct pt_handle *handle, int id)
{
	ptid_t ptid = handle->pts[id].ptid;
	struct pt_driver *driver = handle->pts[id].driver;
	void *data = driver->data;
	unsigned long flags;

	spin_lock_irqsave(&driver->lock, flags);
	driver->ops->enable(data, ptid);
	spin_unlock_irqrestore(&driver->lock, flags);
}

static void pt_driver_disable(struct pt_handle *handle, int id)
{
	ptid_t ptid = handle->pts[id].ptid;
	struct pt_driver *driver = handle->pts[id].driver;
	void *data = driver->data;
	unsigned long flags;

	spin_lock_irqsave(&driver->lock, flags);
	driver->ops->disable(data, ptid);
	spin_unlock_irqrestore(&handle->pts[id].driver->lock, flags);
}

static void pt_driver_free(struct pt_handle *handle, int id)
{
	ptid_t ptid = handle->pts[id].ptid;
	struct pt_driver *driver = handle->pts[id].driver;
	void *data = driver->data;
	unsigned long flags;

	spin_lock_irqsave(&driver->lock, flags);
	driver->ops->free(data, ptid);
	handle->pts[id].ptid = PT_PTID_INVALID;
	spin_unlock_irqrestore(&handle->pts[id].driver->lock, flags);
}

static int pt_driver_mutate(struct pt_handle *handle, int old_id, int new_id)
{
	ptid_t new_ptid;
	ptid_t old_ptid = handle->pts[old_id].ptid;
	struct pt_driver *driver = handle->pts[old_id].driver;
	void *data = driver->data;
	int new_property_index = handle->pts[new_id].property_index;
	unsigned long flags;

	spin_lock_irqsave(&driver->lock, flags);
	new_ptid = driver->ops->mutate(data, old_ptid,
					&handle->pts[new_id],
					new_property_index);

	if (new_ptid != PT_PTID_INVALID) {
		handle->pts[old_id].ptid = PT_PTID_INVALID;
		handle->pts[new_id].ptid = new_ptid;
		handle->pts[old_id].enabled = false;
		handle->pts[new_id].enabled = true;
	}
	spin_unlock_irqrestore(&driver->lock, flags);
	return new_ptid;
}

static void pt_driver_put(struct pt_driver *driver)
{
	unsigned long flags;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	if (driver->ref < 1)
		panic("%s %s has already been registered\n",
			__func__, driver->node->name);
	driver->ref--;
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);
}

static int pt_driver_get(const char *name, struct pt_driver **driver,
	int *property_index)
{
	int i;
	unsigned long flags;
	int ret = -ENOENT;
	struct pt_driver *driver_temp = NULL;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	list_for_each_entry(driver_temp,
		&pt_internal_data.driver_list, list) {
		for (i = 0; i < driver_temp->properties->cnt; i++) {
			if (strcmp(driver_temp->properties->nodes[i]->name,
					name))
				continue;
			*property_index = i;
			*driver = driver_temp;
			driver_temp->ref++;
			ret = 0;
			break;
		}
		if (ret >= 0)
			break;
	}
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);
	return ret;
}

static int pt_driver_get_by_name(const char *driver_name,
				struct pt_driver **driver)
{
	unsigned long flags;
	struct pt_driver *driver_temp = NULL;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	list_for_each_entry(driver_temp,
		&pt_internal_data.driver_list, list) {
		if (strcmp(driver_name, driver_temp->node->name))
			continue;
		*driver = driver_temp;
		driver_temp->ref++;
		spin_unlock_irqrestore(&pt_internal_data.sl, flags);
		return 0;
	}
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);
	return -1;
}

static int pt_driver_ioctl3(struct pt_driver *driver, int arg0,
				int arg1, int arg2)
{
	int ret;
	int args[3];
	unsigned long flags;

	if (!driver->ops->ioctl)
		return -EINVAL;
	spin_lock_irqsave(&driver->lock, flags);
	args[0] = arg0;
	args[1] = arg1;
	args[2] = arg2;
	ret = driver->ops->ioctl(driver->data, 3, args);
	if (ret >= 0)
		driver->ioctl_ret = args[0];
	spin_unlock_irqrestore(&driver->lock, flags);
	return ret;
}

struct list_head *pt_get_handle_list(void)
{
	if (pt_internal_data.sysctl_header)
		return (&pt_internal_data.handle_list);

	return NULL;
}
EXPORT_SYMBOL(pt_get_handle_list);

static struct pt_handle *pt_handle_search(struct device_node *node)
{
	struct pt_handle *temp = NULL;

	list_for_each_entry(temp, &pt_internal_data.handle_list, list)
		if (temp->node == node)
			return temp;
	return NULL;
}

static void pt_handle_check(struct pt_handle *handle, int id)
{
	if ((id >= handle->id_cnt) || (id < 0))
		panic("%s %s %d unknown\n", __func__, handle->node->name, id);
}

static void pt_handle_sysctl_register(struct pt_handle *handle)
{
	int id;
	struct ctl_table *sysctl_table;
	int entry_cnt = 6 + handle->id_cnt + 1;

	sysctl_table = kmalloc_array(entry_cnt, sizeof(*sysctl_table),
					GFP_KERNEL);
	if (!sysctl_table)
		return;
	memset(sysctl_table, 0, sizeof(*sysctl_table) * entry_cnt);
	sysctl_table[0].procname = "dev";
	sysctl_table[0].mode = 0550;
	sysctl_table[0].child = &sysctl_table[2];
	sysctl_table[2].procname = "pt";
	sysctl_table[2].mode = 0550;
	sysctl_table[2].child = &sysctl_table[4];
	sysctl_table[4].procname = handle->node->name;
	sysctl_table[4].mode = 0550;
	sysctl_table[4].child = &sysctl_table[6];
	for (id = 0; id < handle->id_cnt; id++) {
		struct device_node **nodes =
			handle->pts[id].driver->properties->nodes;
		int property_index = handle->pts[id].property_index;

		sysctl_table[6 + id].procname = nodes[property_index]->name;
		sysctl_table[6 + id].data = &handle->pts[id];

		// Don't show the handle and driver pointers in sysctl
		sysctl_table[6 + id].maxlen =
			(long long)&(((struct pt_pts *)NULL)->handle);
		sysctl_table[6 + id].mode = 0444;
		sysctl_table[6 + id].proc_handler = proc_dointvec;
	}
	handle->sysctl_header =
		register_sysctl_table(sysctl_table);
	if (IS_ERR(handle->sysctl_header)) {
		handle->sysctl_header = NULL;
		kfree(sysctl_table);
	} else
		handle->sysctl_table = sysctl_table;
}

static void pt_handle_sysctl_unregister(struct pt_handle *handle)
{
	if (!handle->sysctl_header)
		return;
	unregister_sysctl_table(handle->sysctl_header);
	kfree(handle->sysctl_table);
}

void pt_global_disable(struct pt_global *global)
{
	void *data;
	int property_index;
	unsigned long flags;

	if (!global->driver)
		return;

	spin_lock_irqsave(&global->driver->lock, flags);

	if (global->ptid == PT_PTID_INVALID)
		goto out_unlock;

	data = global->driver->data;
	property_index = global->property_index;
	global->ptid = global->driver->ops->alloc(data,
						  property_index,
						  NULL,
						  pt_resize_internal_nop);
	if (global->ptid == PT_PTID_INVALID)
		goto out_unlock;

	global->driver->ops->disable(data, global->ptid);
	global->driver->ops->free(data, global->ptid);
	global->ptid = PT_PTID_INVALID;
	global->pbha = PT_PTID_INVALID;

out_unlock:
	spin_unlock_irqrestore(&global->driver->lock, flags);
}

static void pt_global_enable(struct pt_global *global)
{
	void *data;
	int property_index;
	unsigned long flags;

	if (!global->driver)
		return;

	spin_lock_irqsave(&global->driver->lock, flags);

	if (global->ptid != PT_PTID_INVALID)
		goto out_unlock;

	data = global->driver->data;
	property_index = global->property_index;
	global->ptid = global->driver->ops->alloc(data,
						  property_index,
						  NULL,
						  pt_resize_internal_nop);
	if (global->ptid == PT_PTID_INVALID)
		goto out_unlock;

	global->driver->ops->enable(data, global->ptid);
	global->pbha = global->driver->ops->pbha(data, global->ptid);

out_unlock:
	spin_unlock_irqrestore(&global->driver->lock, flags);
}

ptpbha_t pt_pbha(struct device_node *node, int id)
{
	ptpbha_t pbha = PT_PBHA_INVALID;
	ptid_t ptid;
	unsigned long flags;
	struct pt_handle *handle;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	handle = pt_handle_search(node);
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);
	if (!handle || id >= handle->id_cnt)
		return PT_PBHA_INVALID;
	spin_lock_irqsave(&handle->lock, flags);
	ptid = handle->pts[id].ptid;
	if (ptid != PT_PTID_INVALID) {
		pbha = handle->pts[id].driver->ops->pbha(
				handle->pts[id].driver->data,
				ptid);
	}
	spin_unlock_irqrestore(&handle->lock, flags);
	return pbha;
}

ptpbha_t pt_pbha_global(enum pt_global_t type)
{
	struct pt_global *global = NULL;

	if (type == PT_GLOBAL_LEFTOVER)
		global = &pt_internal_data.global_leftover;
	else if (type == PT_GLOBAL_BYPASS)
		global = &pt_internal_data.global_bypass;
	else
		return PT_PBHA_INVALID;
	pt_global_enable(global);
	return global->pbha;
}

/*
 * Get a global pid for leftover cache (PT_GLOBAL_LEFTOVER)
 * or not cachable (PT_GLOBAL_BYPASS).
 */
ptid_t pt_pid_global(enum pt_global_t type)
{
	struct pt_global *global = NULL;

	if (type == PT_GLOBAL_LEFTOVER)
		global = &pt_internal_data.global_leftover;
	else if (type == PT_GLOBAL_BYPASS)
		global = &pt_internal_data.global_bypass;
	else
		return PT_PTID_INVALID;
	pt_global_enable(global);
	return global->ptid;
}


ptid_t pt_client_mutate(struct pt_handle *handle, int old_id, int new_id)
{
	size_t size;
	return pt_client_mutate_size(handle, old_id, new_id, &size);
}

ptid_t pt_client_mutate_size(struct pt_handle *handle, int old_id, int new_id, size_t *size)
{
	ptid_t ptid;
	struct pt_driver *driver;
	unsigned long flags;

	driver = handle->pts[old_id].driver;
	if (driver != handle->pts[new_id].driver)
		return PT_PTID_INVALID;
	pt_handle_check(handle, old_id);
	pt_handle_check(handle, new_id);
	spin_lock_irqsave(&handle->lock, flags);
	if ((!handle->pts[old_id].enabled) || (handle->pts[new_id].enabled)) {
		spin_unlock_irqrestore(&handle->lock, flags);
		return PT_PTID_INVALID;
	}
	ptid = pt_driver_mutate(handle, old_id, new_id);
	pt_trace(handle, old_id, false);
	pt_trace(handle, new_id, true);
	/* Update by driver callback */
	*size = handle->pts[new_id].size;
	spin_unlock_irqrestore(&handle->lock, flags);
	return ptid;

}

void pt_client_disable_no_free(struct pt_handle *handle, int id)
{
	unsigned long flags;
	pt_handle_check(handle, id);
	spin_lock_irqsave(&handle->lock, flags);
	if (!pt_resize_list_disable(&handle->pts[id])) {
		spin_unlock_irqrestore(&handle->lock, flags);
		return;
	}
	pt_driver_disable(handle, id);
	pt_trace(handle, id, false);
	spin_unlock_irqrestore(&handle->lock, flags);
}

void pt_client_free(struct pt_handle *handle, int id)
{
	unsigned long flags;
	spin_lock_irqsave(&handle->lock, flags);
	pt_handle_check(handle, id);
	if (handle->pts[id].enabled) {
		spin_unlock_irqrestore(&handle->lock, flags);
		return;
	}
	if (handle->pts[id].ptid != PT_PTID_INVALID)
		pt_driver_free(handle, id);
	spin_unlock_irqrestore(&handle->lock, flags);
}

void pt_client_disable(struct pt_handle *handle, int id)
{
	pt_client_disable_no_free(handle, id);
	pt_client_free(handle, id);
}

ptid_t pt_client_enable_size(struct pt_handle *handle, int id, size_t *size)
{
	unsigned long flags;
	ptid_t ptid;
	spin_lock_irqsave(&handle->lock, flags);
	pt_handle_check(handle, id);
	if (handle->pts[id].enabled) {
		spin_unlock_irqrestore(&handle->lock, flags);
		return -EINVAL;
	}
	ptid = handle->pts[id].ptid;
	/* pt_disable_no_free() can have left ptid allocated */
	if (ptid == PT_PTID_INVALID)
		pt_driver_alloc(handle, id);

	ptid = handle->pts[id].ptid;
	if (ptid != PT_PTID_INVALID) {
		pt_driver_enable(handle, id);
		pt_trace(handle, id, true);
	}
	/* Update by driver callback */
	*size = handle->pts[id].size;
	spin_unlock_irqrestore(&handle->lock, flags);
	if (ptid != PT_PTID_INVALID)
		pt_resize_list_enable(&handle->pts[id]);
	return ptid;
}

ptid_t pt_client_enable(struct pt_handle *handle, int id)
{
	size_t size;
	return pt_client_enable_size(handle, id, &size);
}


void pt_client_unregister(struct pt_handle *handle)
{
	int id;
	unsigned long flags;

	spin_lock_irqsave(&handle->lock, flags);
	for (id = 0; id < handle->id_cnt; id++) {
		if (handle->pts[id].ptid != PT_PTID_INVALID)
			panic("%s %s %d enabled\n", __func__,
				handle->node->name, id);
		pt_driver_put(handle->pts[id].driver);
	}
	spin_lock(&pt_internal_data.sl);
	pt_internal_data.timestamp++;
	list_del(&handle->list);
	spin_unlock(&pt_internal_data.sl);
	pt_handle_sysctl_unregister(handle);

	for (id = 0; id < handle->id_cnt; id++) {
		ptid_t ptid = handle->pts[id].ptid;

		if (ptid == PT_PTID_INVALID)
			continue;
		spin_lock(&handle->pts[id].driver->lock);
		handle->pts[id].driver->ops->free(
			handle->pts[id].driver->data, ptid);
		spin_unlock(&handle->pts[id].driver->lock);
	}

	spin_unlock_irqrestore(&handle->lock, flags);

	kfree(handle);
}

struct pt_handle *pt_client_register(struct device_node *node, void *data,
	pt_resize_callback_t resize_callback)
{
	int id;
	unsigned long flags;
	struct pt_handle *handle;
	const char *propname = "pt_id";
	int len = of_property_count_strings(node, propname);
	size_t size;

	if (len <= 0)
		return ERR_PTR(-EEXIST);
	size = sizeof(*handle) + sizeof(*(handle->pts)) * len;
	handle = kmalloc(size, GFP_KERNEL);
	if (!handle)
		return ERR_PTR(-ENOMEM);
	memset(handle, 0, size);
	handle->data = data;
	handle->id_cnt = len;
	handle->node = node;
	handle->resize_callback = resize_callback;
	handle->pts = (struct pt_pts *)(handle + 1);
	spin_lock_init(&handle->lock);

	// Check/Extract pts from property
	for (id = 0; id < handle->id_cnt; id++) {
		const char *name;
		struct pt_driver *driver;
		int index;
		int ret = of_property_read_string_index(node,
						propname,
						id,
						&name);

		if (ret != 0) {
			kfree(handle);
			return ERR_PTR(-ENOENT);
		}
		if (pt_driver_get(name, &driver, &index) < 0) {
			kfree(handle);
			return ERR_PTR(-ENOENT);
		}
		handle->pts[id].driver = driver;
		handle->pts[id].property_index = index;
		handle->pts[id].handle = handle;
		handle->pts[id].enabled = false;
		handle->pts[id].ptid = PT_PTID_INVALID;
	}
	pt_handle_sysctl_register(handle);

	// Check if the node was not registered
	spin_lock_irqsave(&pt_internal_data.sl, flags);
	if (pt_handle_search(node))
		panic("%s %s has already been registered\n",
			__func__, node->name);
	list_add(&handle->list,  &pt_internal_data.handle_list);
	pt_internal_data.timestamp++;
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);
	return handle;
}

struct pt_driver *pt_driver_register(struct device_node *node,
	const struct pt_ops *ops, void *data)
{
	unsigned long flags;
	struct pt_driver *driver;
	int cnt = 0;
	size_t size;
	struct device_node *child;

	for (child = of_get_next_child(node, NULL); child;
	     child = of_get_next_child(node, child)) {
		int size;

		if (!of_get_property(child, "id_size_priority", &size))
			continue;
		cnt++;
	}

	size = sizeof(struct pt_driver) + sizeof(struct pt_properties)
		+ cnt * sizeof(struct device_node *);
	driver = kmalloc(size, GFP_KERNEL);
	if (driver == NULL)
		return NULL;
	memset(driver, 0, size);
	spin_lock_init(&driver->lock);
	driver->ops = ops;
	driver->node = node;
	driver->data = data;
	driver->ref = 0;
	driver->properties = (struct pt_properties *)(driver + 1);
	driver->properties->nodes =
		(struct device_node **)(driver->properties + 1);
	driver->properties->cnt = cnt;

	cnt = 0;
	for (child = of_get_next_child(node, NULL); child;
		child = of_get_next_child(node, child)) {
		int size;

		if (!of_get_property(child, "id_size_priority", &size))
			continue;
		driver->properties->nodes[cnt] = child;

		if ((!pt_internal_data.global_leftover.driver)
			&& (strcmp(child->name, "LEFTOVER") == 0)) {
			struct pt_global *global =
				&pt_internal_data.global_leftover;
			global->driver = driver;
			global->property_index = cnt;
		} else 	if ((!pt_internal_data.global_bypass.driver)
				&& (strcmp(child->name, "BYPASS") == 0)) {
			struct pt_global *global =
				&pt_internal_data.global_bypass;
			global->driver = driver;
			global->property_index = cnt;
		}
		cnt++;
	}

	driver->sysctl_table[0].procname = "dev";
	driver->sysctl_table[0].mode = 0550;
	driver->sysctl_table[0].child = &driver->sysctl_table[2];
	driver->sysctl_table[2].procname = "pt";
	driver->sysctl_table[2].mode = 0550;
	driver->sysctl_table[2].child = &driver->sysctl_table[4];
	driver->sysctl_table[4].procname = driver->node->name;
	driver->sysctl_table[4].mode = 0550;
	driver->sysctl_table[4].child = &driver->sysctl_table[6];
	driver->sysctl_table[6].procname = "ref";
	driver->sysctl_table[6].data = &driver->ref;
	driver->sysctl_table[6].maxlen = sizeof(driver->ref);
	driver->sysctl_table[6].mode = 0440;
	driver->sysctl_table[6].proc_handler = proc_dointvec;
	driver->sysctl_table[7].procname = "ioctl_ret";
	driver->sysctl_table[7].data = &driver->ioctl_ret;
	driver->sysctl_table[7].maxlen = sizeof(driver->ioctl_ret);
	driver->sysctl_table[7].mode = 0440;
	driver->sysctl_table[7].proc_handler = proc_dointvec;

	driver->sysctl_header = register_sysctl_table(
					&driver->sysctl_table[0]);
	if (IS_ERR(driver->sysctl_header))
		driver->sysctl_header = NULL;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	list_add(&driver->list,  &pt_internal_data.driver_list);
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);

	return driver;
}

int pt_driver_unregister(struct pt_driver *driver)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	if (driver->ref != 0)
		ret = -EBUSY;
	else
		list_del(&driver->list);
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);
	if ((ret == 0) && (driver->sysctl_header))
		unregister_sysctl_table(driver->sysctl_header);
	if (ret == 0)
		kfree(driver);
	return ret;
}

int pt_driver_get_property_value(struct pt_driver *driver, int property_index,
				 int index, u32 *value)
{
	if (driver == NULL)
		return -EINVAL;
	if (driver->properties->cnt <= property_index)
		return -ENOENT;
	return of_property_read_u32_index(
		driver->properties->nodes[property_index],
		"id_size_priority", index, value);
}

void pt_driver_log_module(const char *driver_name, const char *fn_name, u64 arg0,
	u64 arg1, u64 arg2, u64 arg3, int ret, u64 sec_ret0,
	u64 sec_ret1, u64 sec_ret2)
{
	trace_pt_driver_log(driver_name, fn_name, arg0, arg1, arg2, arg3,
		ret, sec_ret0, sec_ret1, sec_ret2);
}

static void pt_test_resize_callback(void *data, int id, size_t size_allocated)
{
	msleep(100);
}


struct pt_test_data {
	size_t size;
	char *data;
};

static uint64_t pt_test_memcpy(long loop, struct pt_test_data *test)
{
	unsigned long prime_number = 103483;
	uint64_t time;
	long i;
	char *data = (char *)test->data;
	size_t size = test->size;

	for (i = 0; i < size / sizeof(unsigned long); i++) {
		((unsigned long *)data)[i] =
			(unsigned long)(i % prime_number);
	}
	preempt_disable();
	time = sched_clock();
	for (i = 0; i < loop; i++) {
		/*
		 * make sure the memcpy is always changing the memory values
		 * We have 4 area, area 0 contains A, area 1 contains B
		 * we do:
		 * - memcpy(area0   , area3  ) : area become BBAB
		 * - memcpy(area1   , area2  ) : area become BAAB
		 * - memcpy(area0-1 , area2-3) : area become ABAB
		 */
		memcpy(data         , data + size / 2 + size / 4, size / 4);
		memcpy(data + size/4, data + size / 2           , size / 4);

		memcpy(data         , data + size / 2           , size / 2);
	}

	time = sched_clock() - time;
	preempt_enable();

	return time;
}

static uint64_t pt_test_chasing(long loop, struct pt_test_data *test)
{
	uint64_t cache_line_size = 64;
	uint64_t time;
	long i;
	long j;
	long next;
	unsigned long index = 0;
	volatile unsigned long *data = (volatile unsigned long *)test->data;
	size_t size = test->size / sizeof (*data);

	for (i = 0; i < size; i++) {
		if (i == size - 1) {
			next = 0;
		} else if (i >= size/2) {
			next = i + cache_line_size / sizeof(*data) - size/2;
		} else {
			next = i + size / 2;
		}
		data[i] = next;
	}

	preempt_disable();
	time = sched_clock();
	for (i = 0; i < loop; i++)
		for (j = 0; j < size / cache_line_size; j++)
			index = data[index];

	time = sched_clock() - time;
	preempt_enable();

	return time;
}


#define CLUSTERRCTLR sys_reg(3, 0, 15, 3 ,4)

static void pt_testfct(const char *name)
{
	long size;
	long loop_chasing = 1024;
	long loop_memcpy = 1024;
	uint64_t time_chasing;
	uint64_t time_memcpy;
	uint64_t clusterectlr;
	struct pt_test_data test;

	clusterectlr = read_sysreg_s(CLUSTERRCTLR);
	pr_info("pt: clusterectlr 0x%llx\n", clusterectlr);

	for (size = 128*1024; size < 32*1024*1024; size += 16*1024) {
		test.data = vmalloc(size);
		if (IS_ERR(test.data)) {
			return;
		}
		test.size = size;

		loop_chasing = loop_chasing / 2;
		do {
			loop_chasing = loop_chasing * 2;
			time_chasing = pt_test_chasing(loop_chasing, &test);
		} while (time_chasing < 500000000 /* .5s */ );

		loop_memcpy = loop_memcpy / 2;
		do {
			loop_memcpy = loop_memcpy * 2;
			time_memcpy = pt_test_memcpy(loop_memcpy, &test);
		} while (time_memcpy < 500000000 /* .5s */ );

		vfree(test.data);
		pr_info("%ld %ld %lld %ld %lld TESTPT %s\n",
			size, loop_chasing, time_chasing,
			loop_memcpy, time_memcpy, name);

		if (time_memcpy > 1000000000 /* 1s */) {
			loop_memcpy = loop_memcpy / 2;
		}
		if (time_chasing > 1000000000 /* 1s */) {
			loop_chasing = loop_chasing / 2;
		}
	}

}

static int pt_sysctl_command(struct ctl_table *ctl, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;
	int retscanf;
	int id;
	int size_newid; // used as size or new_id
	int fn;
	int arg3;
	unsigned long flags;
	struct pt_handle *handle = NULL;
	struct device_node *node;
	struct pt_driver *driver;

	if (!write)
		return -EPERM;
	ret = proc_dostring(ctl, write, buffer, lenp, ppos);
	if (ret != 0)
		return ret;
	retscanf = sscanf(pt_internal_data.sysctl_command,
			"%" PT_COMMAND_SIZE_STR "s %d %d %d %d",
		pt_internal_data.sysctl_node_name,
		&fn, &id, &size_newid, &arg3);
	node = of_find_node_by_name(NULL, pt_internal_data.sysctl_node_name);

	/*if (IS_ERR(node))
		return -ENOENT;*/

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	handle = pt_handle_search(node);
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);

	switch (fn) {
	case PT_FN_REGISTER:
		if (retscanf < 2)
			return -EINVAL;
		if (handle)
			return -EEXIST;
		handle = pt_client_register(node, (void *)node->name,
				pt_test_resize_callback);
		if (IS_ERR(handle))
			return PTR_ERR(handle);
		break;
	case PT_FN_ENABLE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_client_enable(handle, id);
		break;
	case PT_FN_DISABLE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_client_disable(handle, id);
		break;
	case PT_FN_UNREGISTER:
		if (retscanf < 2)
			return -EINVAL;
		if (!handle)
			return -ENOENT;
		pt_client_unregister(handle);
		break;
	case PT_FN_RESIZE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_resize_internal(&handle->pts[id], size_newid);
		break;
	case PT_FN_MUTATE:
		if (retscanf < 4)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt)
			|| (size_newid >= handle->id_cnt))
			return -ENOENT;
		pt_client_mutate(handle, id, size_newid);
		break;
	case PT_FN_DISABLE_NO_FREE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_client_disable_no_free(handle, id);
		break;
	case PT_FN_FREE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_client_free(handle, id);
		break;
	case PT_FN_TEST:
		pt_testfct(pt_internal_data.sysctl_node_name);
		break;
	case PT_FN_GLOBAL_PBHA:
		pr_info("Global pbha %d is %d\n", id,
				(int)pt_pbha_global((enum pt_global_t)id));
		break;
	case PT_FN_GLOBAL_PID:
		pr_info("Global pid %d is %d\n", id,
				(int)pt_pid_global((enum pt_global_t)id));
		break;
	case PT_FN_IOCTL:
		if (retscanf < 5)
			return -EINVAL;
		if (pt_driver_get_by_name(
			pt_internal_data.sysctl_node_name, &driver) < 0)
			return -ENOENT;
		return pt_driver_ioctl3(driver, id, size_newid, arg3);
	default:
		return -EINVAL;
	}
	return 0;
}

static int __init pt_init(void)
{
	struct ctl_table *sysctl_table;

	memset(&pt_internal_data, 0, sizeof(pt_internal_data));
	spin_lock_init(&pt_internal_data.sl);
	pt_internal_data.global_leftover.ptid = PT_PTID_INVALID;
	pt_internal_data.global_leftover.pbha = PT_PBHA_INVALID;
	pt_internal_data.global_leftover.driver = NULL;
	pt_internal_data.global_bypass.ptid = PT_PTID_INVALID;
	pt_internal_data.global_bypass.pbha = PT_PBHA_INVALID;
	pt_internal_data.global_bypass.driver = NULL;
	INIT_LIST_HEAD(&pt_internal_data.handle_list);
	INIT_LIST_HEAD(&pt_internal_data.driver_list);
	INIT_LIST_HEAD(&pt_internal_data.resize_list);
	init_waitqueue_head(&pt_internal_data.resize_wq);
	sysctl_table = &pt_internal_data.sysctl_table[0];
	sysctl_table[0].procname = "dev";
	sysctl_table[0].mode = 0550;
	sysctl_table[0].child = &sysctl_table[2];
	sysctl_table[2].procname = "pt";
	sysctl_table[2].mode = 0550;
	sysctl_table[2].child = &sysctl_table[4];
	sysctl_table[4].procname = "command";
	sysctl_table[4].data = &pt_internal_data.sysctl_command[0];
	sysctl_table[4].maxlen = sizeof(pt_internal_data.sysctl_command);
	sysctl_table[4].mode = 0200;
	sysctl_table[4].proc_handler = pt_sysctl_command;
	sysctl_table[5].procname = "enabled";
	sysctl_table[5].data = &pt_internal_data.enabled;
	sysctl_table[5].maxlen = sizeof(pt_internal_data.enabled);
	sysctl_table[5].mode = 0440;
	sysctl_table[5].proc_handler = proc_dointvec;
	sysctl_table[6].procname = "size";
	sysctl_table[6].data = &pt_internal_data.size;
	sysctl_table[6].maxlen = sizeof(pt_internal_data.size);
	sysctl_table[6].mode = 0440;
	sysctl_table[6].proc_handler = proc_dointvec;
	pt_internal_data.sysctl_header = register_sysctl_table(sysctl_table);
	if (IS_ERR(pt_internal_data.sysctl_header))
		pt_internal_data.sysctl_header = NULL;
	pt_internal_data.resize_thread = kthread_run(pt_resize_thread, NULL,
							"PT_resize");
	return 0;
}

static void __exit pt_exit(void)
{
	if (pt_internal_data.sysctl_header)
		unregister_sysctl_table(pt_internal_data.sysctl_header);
	// TODO: wait for client unregister
}

subsys_initcall(pt_init);
module_exit(pt_exit);

EXPORT_SYMBOL(pt_pbha);
EXPORT_SYMBOL(pt_pbha_global);

EXPORT_SYMBOL(pt_client_register);
EXPORT_SYMBOL(pt_client_enable);
EXPORT_SYMBOL(pt_client_mutate);
EXPORT_SYMBOL(pt_client_mutate_size);
EXPORT_SYMBOL(pt_client_disable);
EXPORT_SYMBOL(pt_client_unregister);
EXPORT_SYMBOL(pt_client_enable_size);
EXPORT_SYMBOL(pt_client_disable_no_free);
EXPORT_SYMBOL(pt_client_free);

EXPORT_SYMBOL(pt_driver_register);
EXPORT_SYMBOL(pt_driver_unregister);
EXPORT_SYMBOL(pt_driver_get_property_value);
EXPORT_SYMBOL(pt_driver_log_module);


MODULE_DESCRIPTION("PT API");
MODULE_AUTHOR("<cozette@google.com>");
MODULE_LICENSE("GPL");

// TODO: manage this module ref count and unload
// TODO: manage driver modules and handle ref count

/*
 * Test
 * cat << OLI > pt.test.sh
 * echo 1 > /sys/kernel/debug/tracing/tracing_on
 * echo 1 > /sys/kernel/debug/tracing/events/pt/pt_enable/enable
 * echo 1 > /sys/kernel/debug/tracing/events/pt/pt_resize_callback/enable
 * cd /proc/sys/dev/pt
 * find . -print -exec cat {} \; 2>/dev/null
 * echo gpio_keys 0 0 >  /proc/sys/dev/pt/command
 * find . -print -exec cat {} \; 2>/dev/null
 * find .
 * echo gpio_keys 1 0 >  /proc/sys/dev/pt/command
 * find . -print -exec cat {} \; 2>/dev/null
 * echo gpio_keys 4 0 32767 >  /proc/sys/dev/pt/command
 * echo gpio_keys 2 0 >  /proc/sys/dev/pt/command
 * find . -print -exec cat {} \; 2>/dev/null
 * echo gpio_keys 3 0 >  /proc/sys/dev/pt/command
 * find . -print -exec cat {} \; 2>/dev/null
 * cat /sys/kernel/debug/tracing/trace
 * OLI
 * sh pt.test.sh
 */
