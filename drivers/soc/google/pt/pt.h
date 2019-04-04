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

/*
 * API for client requesting PT
 */
struct pt_handle;
struct pt_info {
	const char *node;
	const char *id_name;
	int id;
	int size_index;
	u32 ptid;
	uint64_t size;
	bool enabled;
	bool resize_in_progress;
};
typedef bool  (*pt_resize_callback_t)(void *data, int id, int size_allocated);

struct pt_handle *pt_register(struct device_node *node, void *data,
	pt_resize_callback_t resize_callback);
void pt_unregister(struct pt_handle *handle);

/* Enable id a allocate a ptid for it if none was already allocated */
int  pt_enable(struct pt_handle *handle, int id);

/* Disable id and free its ptid */
void pt_disable(struct pt_handle *handle, int id);

/* Disabled id, but its ptid stay allocated */
void pt_disable_no_free(struct pt_handle *handle, int id);

/* Free id ptid, this id must have been disabled with pt_disable_no_free() */
void pt_free(struct pt_handle *handle, int id);

/*
 * Transfer the ptid of old_id to new_id
 * It will keep the same data cached, but change the parammeters.
 */
int  pt_mutate(struct pt_handle *handle, int old_id, int new_id);

/*
 * Get the pbha of a device/id, need the id to be enabled or
 * allocated (pt_enable()/pt_disable_no_free())
 */
int  pt_pbha(struct device_node *node, int id);
bool pt_info_get(int iterator, uint64_t *timestamp, struct pt_info *info);

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
	 * Alloc allocate a ptid and set the resize callback.
	 * It does not enable it.
	 */
	int (*alloc)(void *data, int property_index, void *resize_data,
		void (*resize)(void *data, size_t size));
	/*
	 * Free free a ptid allocated before, but which in no more enabled
	 */
	void (*free)(void *data, int ptid);
	/*
	 * Enable an alocated ptid. The ptid must not be currently enabled.
	 */
	void (*enable)(void *data, int ptid);
	/*
	 * Disable a ptid. The ptid must be currently enabled.
	 */
	void (*disable)(void *data, int ptid);

	/*
	 * Change a ptid parametters to new parameters coming
	 * from new_property_index.
	 * The old ptid must be currently enabled.
	 */
	int (*mutate)(void *data, int ptid, void *resize_data,
			int new_property_index);

	/*
	 * Call to pbha is NOT serialized by pt.c
	 *
	 * Error return PT_PBHA_INVALID
	 */
	int (*pbha)(void *data, int ptid);
};

struct pt_driver *pt_driver_register(struct device_node *node,
	const struct pt_ops *ops, void *data);
int pt_driver_unregister(struct pt_driver *driver);
int pt_driver_get_property_value(struct pt_driver *driver, int property_index,
	int index, u32 *value);
#endif /* __GOOGLE_PT_H_ */
