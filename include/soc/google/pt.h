/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * pt.h
 *
 * Partition API for system cache management API.
 *
 * Copyright 2019 Google LLC
 *
 * Author: cozette@google.com
 */

#ifndef __GOOGLE_PT_H_
#define __GOOGLE_PT_H_
#include <linux/of.h>


#define PT_PTID_INVALID -1
#define PT_PBHA_INVALID -1
#define PT_PBHA_ENABLE 0x8000
#define PT_PBHA_MASK 0x0fff

enum pt_global_t {
	PT_GLOBAL_BYPASS = 0,
	PT_GLOBAL_LEFTOVER = 1,
	PT_GLOBAL_LAST
};

typedef int ptid_t; /* valid slc PID or PT_PTID_INVALID */
typedef int ptpbha_t; /* valid PBHA or PT_PBHA_INVALID */
/*
 * data: the "data" given in pt_client_register
 * id: the index in devicetree.
 * size_allocated: size allocated in bytes
 */
typedef void (*pt_resize_callback_t)(void *data, int id,
					size_t size_allocated);
/*
 * API for client requesting PT
 */
struct pt_handle { /* one per client */
	spinlock_t lock; /* serialize write access to the handle */
	struct list_head list;
	pt_resize_callback_t resize_callback;
	int id_cnt;
	struct pt_pts *pts; /* client partitions */
	struct device_node *node; /* client node */
	struct ctl_table *sysctl_table;
	struct ctl_table_header *sysctl_header;
	void *data; /* client private data */
};

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)

/*
 * Register a callback and a device node for managing partition (ptid)
 * associated to that device node.
 */
struct pt_handle *pt_client_register(struct device_node *node, void *data,
	pt_resize_callback_t resize_callback);

/*
 * Free resource used to managed partition.
 * Panic if not all id are disabled and freed.
 */
void pt_client_unregister(struct pt_handle *handle);

/* Enable id and allocate a ptid for it if none was already allocated */
ptid_t pt_client_enable(struct pt_handle *handle, int id);

/*
 * Enable id and allocate a ptid for it if none was already allocated
 * size is output only and will contain the size allocated.
 * Any further size change will be notified through callback.
 */
ptid_t pt_client_enable_size(struct pt_handle *handle, int id, size_t *size);

/* Disable id and free its ptid */
void pt_client_disable(struct pt_handle *handle, int id);

/* Disabled id, but its ptid stay allocated */
void pt_client_disable_no_free(struct pt_handle *handle, int id);

/* Free id ptid, this id must have been disabled with pt_disable_no_free() */
void pt_client_free(struct pt_handle *handle, int id);

/*
 * Transfer the ptid of old_id to new_id
 * It will keep the same data cached, but change the parammeters.
 * return:
 *	Success: same ptid that the ptid returned at pt_client_enable(old_id)
 * 	Error: PT_PTID_INVALID: no data for new_id or old_id unallocated
 *	Panic: if new_id/old_id can't share the same pbha
 */
ptid_t pt_client_mutate(struct pt_handle *handle, int old_id, int new_id);

/*
 * See pt_client_mutate
 * size is output only and will contain the size allocated.
 */
ptid_t pt_client_mutate_size(struct pt_handle *handle, int old_id, int new_id, size_t *size);

/*
 * Get the pbha for a previously enabled or allocated ptid
 */
ptpbha_t pt_pbha(struct device_node *node, int id);

/*
 * Get a global pbha for leftover cache (PT_GLOBAL_LEFTOVER)
 * or not cachable (PT_GLOBAL_BYPASS), never fail.
 */
ptpbha_t pt_pbha_global(enum pt_global_t type);

/*
 * Get a global pid for leftover cache (PT_GLOBAL_LEFTOVER)
 * or not cachable (PT_GLOBAL_BYPASS), never fail.
 */
ptid_t pt_pid_global(enum pt_global_t type);

#else /* CONFIG_SLC_PARTITION_MANAGER */

static inline struct pt_handle *pt_client_register(struct device_node *node,
						void *data,
						pt_resize_callback_t
						resize_callback)
{
	return ERR_PTR(-ENOSYS);
}

static inline void pt_client_unregister(struct pt_handle *handle)
{
}

static inline ptid_t pt_client_enable(struct pt_handle *handle, int id)
{
	return PT_PTID_INVALID;
}

static inline void pt_client_disable(struct pt_handle *handle, int id)
{
}

static inline void pt_client_disable_no_free(struct pt_handle *handle, int id)
{
}

static inline void pt_client_free(struct pt_handle *handle, int id)
{
}

static inline ptid_t pt_client_mutate(struct pt_handle *handle, int old_id,
				int new_id)
{
	return PT_PTID_INVALID;
}

static inline ptid_t pt_client_mutate_size(struct pt_handle *handle, int old_id,
				int new_id, size_t *size)
{
	return PT_PTID_INVALID;
}

static inline ptpbha_t pt_pbha(struct device_node *node, int id)
{
	return 0;
}

static inline ptpbha_t pt_pbha_global(enum pt_global_t type)
{
	return 0;
}

static inline ptid_t pt_pid_global(enum pt_global_t type)
{
	return PT_PTID_INVALID;
}

#endif /* CONFIG_SLC_PARTITION_MANAGER */

/*
 * API for driver implementing PT
 */
struct pt_driver;
struct pt_ops {
	/*
	 * Call to alloc/free/enable/disable/mutate are serialize by pt.c
	 * Functions alloc/free/enable/disable/mutate may call resize()
	 * callback.
	 *
	 * Error return PT_PTID_INVALID
	 */

	/*
	 * Allocate a ptid and set the resize callback.
	 * It does not enable it.
	 */
	ptid_t (*alloc)(void *data, int property_index, void *resize_data,
		void (*resize)(void *data, size_t size));
	/*
	 * Free a previously allocated ptid which is no longer enabled
	 */
	void (*free)(void *data, ptid_t ptid);
	/*
	 * Enable an alocated ptid. The ptid must not be currently enabled.
	 */
	void (*enable)(void *data, ptid_t ptid);
	/*
	 * Disable a ptid. The ptid must be currently enabled.
	 */
	void (*disable)(void *data, ptid_t ptid);

	/*
	 * Change a ptid parameters to new parameters coming
	 * from new_property_index.
	 * The old ptid must be currently enabled.
	 *
	 * return ptid given or PT_PTID_INVALID is error.
	 */
	ptid_t (*mutate)(void *data, ptid_t ptid, void *resize_data,
			int new_property_index);

	/*
	 * Call to pbha is NOT serialized by pt.c
	 *
	 * return pbha for ptid on success
	 * 	PT_PBHA_INVALID on error
	 */
	ptpbha_t (*pbha)(void *data, int ptid);

	/*
	 * Call internal function of the driver
	 * args[0] is return value
	 *
	 * return >=0 on success
	 */
	int (*ioctl)(void *data, int arg_cnt, int *args);
};

struct pt_driver *pt_driver_register(struct device_node *node,
	const struct pt_ops *ops, void *data);
int pt_driver_unregister(struct pt_driver *driver);
int pt_driver_get_property_value(struct pt_driver *driver, int property_index,
	int index, u32 *value);
void pt_driver_log_module(const char *driver_name, const char *fn_name, u64 arg0,
	u64 arg1, u64 arg2, u64 arg3, int ret, u64 sec_ret0,
	u64 sec_ret1, u64 sec_ret2);
struct list_head *pt_get_handle_list(void);

#endif /* __GOOGLE_PT_H_ */
