/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GCIP helpers for accessing resources for debugging.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GCIP_RESOURCE_ACCESSOR_H__
#define __GCIP_RESOURCE_ACCESSOR_H__

#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/list.h>

struct gcip_resource_accessor {
	/* For logging.  */
	struct device *dev;
	/* The list of all resource resources for debugging. */
	struct list_head resource_list;
	/* Protects the list headed by @resource_list. */
	spinlock_t resource_list_lock;
	/* The dentry object of the created deubgfs file. */
	struct dentry *dentry;
	/* The last query address. */
	phys_addr_t last_query_addr;
	/* The last query width. */
	unsigned int last_query_width;
};

/* The wrapper to store @resource objects in a list. */
struct gcip_resource_list_element {
	struct resource resource;
	struct list_head list;
};

/**
 * Creates a resource accessor and creates a "resource-accessor" debugfs file, which is an interface
 * to read/write resources if the requested address is located in the pre-registered resource
 * ranges.
 *
 * To read data from a specific physical address, the commands are:
 *          "echo {@addr} {@width} > <debugfs>/resource-accessor"
 *          "cat <debugfs>/resource-accessor"
 * After the first command, if @addr is located in a registered region, the address is recorded and
 * the value with size @width of @addr is printed out by the second command.
 * @addr is interpreted as an 8-byte hex value.
 * @width is interpreted as a 4-byte decimal value. Only 1,2,4,8 are valid values.
 *
 * To write data at a specific physical address, the command is:
 *         "echo {@addr} {@width} {@value} > <debugfs>/resource-accessor"
 * After the command, if @addr is located in a registered region, @value with size @width is written
 * to @addr.
 * @addr is interpreted as an 8-byte hex value.
 * @width is interpreted as a 4-byte decimal value. Only 1,2,4,8 are valid values.
 * @value is interpreted as a hex value with @width size.
 * After the writing, "cat <debugfs>/resource-accessor" would read data from @addr as well.
 *
 * Examples (assuming address 0xffff000012345678 is registered as a device memory resource):
 * Read a 4-byte value:
 *         # echo 0xffff000012345678 4 > <debugfs>/resource-accessor
 *         # cat <debugfs>/resource-accessor
 *         0xffff000012345678: 0xdeadbeef
 * Write a 4-byte value:
 *         # echo 0xffff000012345678 4 0xdeadbeef > <debugfs>/resource-accessor
 *         # cat <debugfs>/resource-accessor
 *         0xffff000012345678: 0xdeadbeef
 *
 * @dev: The device for which the resource allocator is being created.
 * @parent_dentry: The dentry of the debugfs directory to create "resource-accessor" under it.
 *
 * Returns the created resource accessor on success.
 * Returns a negative errno on error.
 */
struct gcip_resource_accessor *gcip_resource_accessor_create(struct device *dev,
							     struct dentry *parent_dentry);

/**
 * Removes the created debugfs file, clears the list and releases resources.
 *
 * @accessor: pointer to the resource accessor.
 */
void gcip_resource_accessor_destroy(struct gcip_resource_accessor *accessor);

/**
 * Registers a resource (CSR chunk or reserved resource) to be accessible by the debugfs file.
 *
 * @accessor: The pointer to the resource accessor.
 * @resource: The resource object to be accessible by the debugfs file.
 *
 * Returns 0 on success or negative error value.
 */
int gcip_register_accessible_resource(struct gcip_resource_accessor *accessor,
				      const struct resource *resource);

#endif /* __GCIP_RESOURCE_ACCESSOR_H__ */
