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

#include "pt.h"
#include "pt_trace.h"
#include <linux/list.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define PT_SYSCTL_ENTRY 8
#define PT_COMMAND_SIZE 128
#define PT_COMMAND_SIZE_STR "128"

struct pt_pts {
	bool enabled;
	u32 size; /* current size of the partition, 0 if disabled */
	int ptid; /* partition index */
	int property_index; /* index in the driver properties */
	struct pt_handle *handle;
	struct pt_driver *driver; /* driver managing this partition */
};

struct pt_properties {
	struct device_node **nodes;
	int cnt; /* Number of nodes */
};

/*
 * Locking:
 * - All access to driver (except pbha) are protected by the driver->mt
 *	(->alloc()/->enable()/->disable()/->free())
 *	These access aways happen while a handle->mt is also taken
 * - All access to handle are protected by handle->mt
 *      (pt_enable()/pt_disable()/pt_free()/pt_disable_no_free())
 * - Adding/removing handle and changing size of a pts is protected by
 *	pt_internal_data->sl
 * - resize_callback() could happen in any context and handle->mt or
 *	driver->mt can already be taken.
 *	Because of that, currently calling pt_* from resize_callback is not
 *	supported. It wil be with some limitation when adding a kernel thread
 *	to process then.
 */

struct pt_handle { /* one per client */
	struct mutex mt; /* serialize write access to the handle */
	struct list_head list;
	pt_resize_callback_t resize_callback;
	int id_cnt;
	struct pt_pts *pts; /* client partitions */
	struct device_node *node; /* client node */
	struct ctl_table *sysctl_table;
	struct ctl_table_header *sysctl_header;
	void *data; /* client private data */
};

struct pt_driver {
	/* partition properties in driver node */
	struct pt_properties *properties;
	struct list_head list;
	const struct pt_ops *ops;
	struct mutex mt; /* serialize access to the driver */
	int ref;
	struct device_node *node; /* driver node */
	void *data; /* driver private data */
	struct ctl_table_header *sysctl_header;
	struct ctl_table sysctl_table[PT_SYSCTL_ENTRY];
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
} pt_internal_data;

enum pt_fn {
	PT_FN_REGISTER = 0,
	PT_FN_ENABLE = 1,
	PT_FN_DISABLE = 2,
	PT_FN_UNREGISTER = 3,
	PT_FN_RESIZE = 4,
	PT_FN_MUTATE = 5,
	PT_FN_DISABLE_NO_FREE = 6,
	PT_FN_FREE = 7
};

static void pt_trace(struct pt_handle *handle, int id, bool enable)
{
	struct pt_properties *properties;
	int property_index;
	const char *name;
	struct pt_driver *driver;
	int ptid;

	ptid = handle->pts[id].ptid;
	driver = handle->pts[id].driver;
	properties = driver->properties;
	property_index = handle->pts[id].property_index;
	name = properties->nodes[property_index]->name;
	trace_pt_enable(handle->node->name, name, enable, ptid);
}

static void pt_internal_resize(void *data, size_t size)
{
	// TODO: use wq for the resize_callback
	unsigned long flags;
	pt_resize_callback_t resize_callback = NULL;
	struct pt_pts *pts = (struct pt_pts *)data;
	struct pt_handle *handle = pts->handle;
	struct pt_properties *properties = pts->driver->properties;
	int id = ((char *)data - (char *)handle->pts) / sizeof(handle->pts[0]);

	trace_pt_resize_callback(handle->node->name,
		properties->nodes[pts->property_index]->name,
		true, (int)size, pts[id].ptid);

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	if ((pts->size == 0) && (size > 0))
		pt_internal_data.enabled++;
	if ((pts->size > 0) && (size == 0))
		pt_internal_data.enabled--;
	if ((pts->size != size) && (pts->enabled))
		resize_callback = handle->resize_callback;
	pt_internal_data.size -= pts->size;
	pts->size = size;
	pt_internal_data.size += pts->size;
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);

	if (resize_callback)
		resize_callback(handle->data, id, size);

	trace_pt_resize_callback(handle->node->name,
		properties->nodes[pts->property_index]->name,
		false, (int)size, pts[id].ptid);
}

static bool pt_driver_alloc(struct pt_handle *handle, int id)
{
	int ptid;
	struct pt_driver *driver = handle->pts[id].driver;
	void *data = driver->data;
	int property_index = handle->pts[id].property_index;

	mutex_lock(&driver->mt);
	ptid = driver->ops->alloc(data,
				  property_index,
				  &handle->pts[id],
				  pt_internal_resize);
	if (ptid != PT_PTID_INVALID)
		handle->pts[id].ptid = ptid;
	mutex_unlock(&handle->pts[id].driver->mt);
	return ptid != PT_PTID_INVALID;
}

static void pt_driver_enable(struct pt_handle *handle, int id)
{
	int ptid = handle->pts[id].ptid;
	struct pt_driver *driver = handle->pts[id].driver;
	void *data = driver->data;

	mutex_lock(&driver->mt);
	driver->ops->enable(data, ptid);
	handle->pts[id].enabled = true;
	/*
	 * enabled set after calling ->enable()
	 * so, the callback to pt_internal_resize() in ->disable()
	 * will only update handle->pts[id].size,
	 * but won't call the handle resize_callback
	 */
	mutex_unlock(&driver->mt);
}

static void pt_driver_disable(struct pt_handle *handle, int id)
{
	int ptid = handle->pts[id].ptid;
	struct pt_driver *driver = handle->pts[id].driver;
	void *data = driver->data;

	mutex_lock(&driver->mt);
	/*
	 * enabled cleared before calling ->disable()
	 * so, the callback to pt_internal_resize() in ->disable()
	 * will only update handle->pts[id].size,
	 * but won't call the handle resize_callback
	 */
	handle->pts[id].enabled = false;
	driver->ops->disable(data, ptid);
	mutex_unlock(&handle->pts[id].driver->mt);
}

static void pt_driver_free(struct pt_handle *handle, int id)
{
	int ptid = handle->pts[id].ptid;
	struct pt_driver *driver = handle->pts[id].driver;
	void *data = driver->data;

	mutex_lock(&driver->mt);
	driver->ops->free(data, ptid);
	handle->pts[id].ptid = PT_PTID_INVALID;
	mutex_unlock(&handle->pts[id].driver->mt);
}

static int pt_driver_mutate(struct pt_handle *handle, int old_id, int new_id)
{
	int new_ptid;
	int old_ptid = handle->pts[old_id].ptid;
	struct pt_driver *driver = handle->pts[old_id].driver;
	void *data = driver->data;
	int new_property_index = handle->pts[new_id].property_index;

	mutex_lock(&driver->mt);
	new_ptid = driver->ops->mutate(data, old_ptid,
					&handle->pts[new_id],
					new_property_index);

	if (new_ptid != PT_PTID_INVALID) {
		handle->pts[old_id].ptid = PT_PTID_INVALID;
		handle->pts[new_id].ptid = new_ptid;
		handle->pts[old_id].enabled = false;
		handle->pts[new_id].enabled = true;
	}
	mutex_unlock(&driver->mt);
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

int pt_pbha(struct device_node *node, int id)
{
	int pbha = PT_PBHA_INVALID;
	int ptid;
	unsigned long flags;
	struct pt_handle *handle;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	handle = pt_handle_search(node);
	if (id >= handle->id_cnt)
		handle = NULL;
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);
	if (!handle)
		return PT_PBHA_INVALID;
	mutex_lock(&handle->mt);
	ptid = handle->pts[id].ptid;
	if (ptid != PT_PTID_INVALID) {
		pbha = handle->pts[id].driver->ops->pbha(
				handle->pts[id].driver->data,
				ptid);
	}
	mutex_unlock(&handle->mt);
	return pbha;
}

int pt_mutate(struct pt_handle *handle, int old_id, int new_id)
{
	int ptid;
	struct pt_driver *driver;

	driver = handle->pts[old_id].driver;
	if (driver != handle->pts[new_id].driver)
		return PT_PTID_INVALID;
	pt_handle_check(handle, old_id);
	pt_handle_check(handle, new_id);
	mutex_lock(&handle->mt);
	if ((!handle->pts[old_id].enabled) || (handle->pts[new_id].enabled)) {
		mutex_unlock(&handle->mt);
		return PT_PTID_INVALID;
	}
	ptid = pt_driver_mutate(handle, old_id, new_id);
	pt_trace(handle, old_id, false);
	pt_trace(handle, new_id, true);
	mutex_unlock(&handle->mt);
	return ptid;

}

void pt_disable_no_free(struct pt_handle *handle, int id)
{
	mutex_lock(&handle->mt);
	pt_handle_check(handle, id);
	if (!handle->pts[id].enabled) {
		mutex_unlock(&handle->mt);
		return;
	}
	pt_driver_disable(handle, id);
	pt_trace(handle, id, false);
	mutex_unlock(&handle->mt);
}

void pt_free(struct pt_handle *handle, int id)
{
	mutex_lock(&handle->mt);
	pt_handle_check(handle, id);
	if (handle->pts[id].enabled) {
		mutex_unlock(&handle->mt);
		return;
	}
	if (handle->pts[id].ptid != PT_PTID_INVALID)
		pt_driver_free(handle, id);
	mutex_unlock(&handle->mt);
}

void pt_disable(struct pt_handle *handle, int id)
{
	pt_disable_no_free(handle, id);
	pt_free(handle, id);
}

int pt_enable(struct pt_handle *handle, int id)
{
	int ptid;

	mutex_lock(&handle->mt);
	pt_handle_check(handle, id);
	if (handle->pts[id].enabled) {
		mutex_unlock(&handle->mt);
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
	mutex_unlock(&handle->mt);
	return ptid;
}

bool pt_info_get(int iterator, uint64_t *timestamp, struct pt_info *info)
{
	bool success = false;

	// TODO
	return success;
}

void pt_unregister(struct pt_handle *handle)
{
	int id;
	unsigned long flags;

	mutex_lock(&handle->mt);
	for (id = 0; id < handle->id_cnt; id++) {
		if (handle->pts[id].ptid != PT_PTID_INVALID)
			panic("%s %s %d enabled\n", __func__,
				handle->node->name, id);
		pt_driver_put(handle->pts[id].driver);
	}
	spin_lock_irqsave(&pt_internal_data.sl, flags);
	pt_internal_data.timestamp++;
	list_del(&handle->list);
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);
	pt_handle_sysctl_unregister(handle);

	for (id = 0; id < handle->id_cnt; id++) {
		int ptid = handle->pts[id].ptid;

		if (ptid == PT_PTID_INVALID)
			continue;
		mutex_lock(&handle->pts[id].driver->mt);
		handle->pts[id].driver->ops->free(
			handle->pts[id].driver->data, ptid);
		mutex_unlock(&handle->pts[id].driver->mt);
	}
	kfree(handle);
}

struct pt_handle *pt_register(struct device_node *node, void *data,
	pt_resize_callback_t resize_callback)
{
	int id;
	unsigned long flags;
	struct pt_handle *handle;
	const char *propname = "pt_id";
	int len = of_property_count_strings(node, propname);
	size_t size;

	if (len <= 0)
		return ERR_PTR(EEXIST);
	size = sizeof(*handle) + sizeof(*(handle->pts)) * len;
	handle = kmalloc(size, GFP_KERNEL);
	if (IS_ERR(handle))
		return ERR_PTR(ENOMEM);
	memset(handle, 0, size);
	handle->data = data;
	handle->id_cnt = len;
	handle->node = node;
	handle->resize_callback = resize_callback;
	handle->pts = (struct pt_pts *)(handle + 1);
	mutex_init(&handle->mt);

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
			return ERR_PTR(ENOENT);
		}
		if (pt_driver_get(name, &driver, &index) < 0) {
			kfree(handle);
			return ERR_PTR(ENOENT);
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

static bool pt_test_resize_callback(void *data, int id, int size_allocated)
{
	msleep(100);
	return true;
}

static int pt_sysctl_command(struct ctl_table *ctl, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;
	int retscanf;
	int id;
	int size_newid; // used as size or new_id
	int fn;
	unsigned long flags;
	struct pt_handle *handle = NULL;
	struct device_node *node;

	if (!write)
		return -EPERM;
	ret = proc_dostring(ctl, write, buffer, lenp, ppos);
	if (ret != 0)
		return ret;
	retscanf = sscanf(pt_internal_data.sysctl_command,
			"%" PT_COMMAND_SIZE_STR "s %d %d %d",
		pt_internal_data.sysctl_node_name, &fn, &id, &size_newid);
	node = of_find_node_by_name(NULL, pt_internal_data.sysctl_node_name);

	if (IS_ERR(node))
		return -ENOENT;

	spin_lock_irqsave(&pt_internal_data.sl, flags);
	handle = pt_handle_search(node);
	spin_unlock_irqrestore(&pt_internal_data.sl, flags);

	switch (fn) {
	case PT_FN_REGISTER:
		if (retscanf < 2)
			return -EINVAL;
		if (handle)
			return -EEXIST;
		handle = pt_register(node, (void *)node->name,
				pt_test_resize_callback);
		if (IS_ERR(handle))
			return -PTR_ERR(handle);
		break;
	case PT_FN_ENABLE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_enable(handle, id);
		break;
	case PT_FN_DISABLE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_disable(handle, id);
		break;
	case PT_FN_UNREGISTER:
		if (retscanf < 2)
			return -EINVAL;
		if (!handle)
			return -ENOENT;
		pt_unregister(handle);
		break;
	case PT_FN_RESIZE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_internal_resize(&handle->pts[id], size_newid);
		break;
	case PT_FN_MUTATE:
		if (retscanf < 4)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt)
			|| (size_newid >= handle->id_cnt))
			return -ENOENT;
		pt_mutate(handle, id, size_newid);
		break;
	case PT_FN_DISABLE_NO_FREE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_disable_no_free(handle, id);
		break;
	case PT_FN_FREE:
		if (retscanf < 3)
			return -EINVAL;
		if ((!handle) || (id >= handle->id_cnt))
			return -ENOENT;
		pt_free(handle, id);
		break;
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
	INIT_LIST_HEAD(&pt_internal_data.handle_list);
	INIT_LIST_HEAD(&pt_internal_data.driver_list);
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
