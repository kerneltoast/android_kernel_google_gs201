/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Bus Manager
 *
 * Copyright 2023 Google LLC.
 */

#ifndef LWIS_BUS_MANAGER_H_
#define LWIS_BUS_MANAGER_H_

#include "lwis_device.h"
#include "lwis_util.h"
#include "lwis_periodic_io.h"
#include "lwis_transaction.h"

#define LWIS_DEFAULT_DEVICE_GROUP 0

/*
 * enum lwis_device_priority_level:
 * Defines the device priority level
 * in which the requests will be executed.
 */
enum lwis_device_priority_level {
	DEVICE_HIGH_PRIORITY,
	DEVICE_MEDIUM_PRIORITY,
	DEVICE_LOW_PRIORITY,
	MAX_DEVICE_PRIORITY_LEVELS
};

/*
 * enum lwis_client_connection:
 * Defines the client connection status being requested.
 */
enum lwis_client_connection { CLIENT_CONNECT, CLIENT_DISCONNECT };

/*
 * struct lwis_bus_manager_list:
 * Holds bus manager list
 */
struct lwis_bus_manager_list {
	struct list_head bus_manager_list_head;
};

/*
 * struct lwis_bus_manager_identifier:
 * Holds a pointer to the bus manager
 */
struct lwis_bus_manager_identifier {
	struct list_head bus_manager_list_node;
	struct lwis_bus_manager *bus_manager;
	int bus_manager_handle;
	int32_t bus_type;
};

/*
 * lwis_process_queue:
 * This maintains the process queue for a given bus.
 * This is a collection of process request nodes that identify
 * the lwis device requests in order they were queued.
 * The scheduler is set to operate requests in a
 * first in-first out manner, starting and updating the head
 * and working towards the tail end.
 */
struct lwis_process_queue {
	/* Head node for the process queue */
	struct list_head head;
	/* Total number of devices that are queued to be processed */
	int number_of_nodes;
};

/*
 *  struct lwis_bus_manager
 *  This defines the main attributes for LWIS Bus Manager.
 */
struct lwis_bus_manager {
	/* Unique identifier for this bus manager */
	int bus_id;
	/* Identifies the device type being managed for this bus */
	int32_t bus_type;
	/* Name of Bus manager corresponds to the name of the LWIS Bus */
	char bus_name[LWIS_MAX_NAME_STRING_LEN];
	/* Lock to control access to bus transfers */
	struct mutex bus_lock;
	/* Lock to control access to the process queue for this bus */
	struct mutex process_queue_lock;
	/* Bus thread priority */
	u32 bus_thread_priority;
	/* Worker thread */
	struct kthread_worker bus_worker;
	struct task_struct *bus_worker_thread;
	/* Queue of all LWIS devices that have data their process queues */
	struct lwis_process_queue bus_process_queue[MAX_DEVICE_PRIORITY_LEVELS];
	/* List of LWIS devices using this bus */
	struct list_head connected_devices;
	/* Total number of physically connected devices to the bus
	 * This count is set while probe/unprobe sequence
	 */
	int number_of_connected_devices;
	/* Control access to the high priority transaction queue for this bus */
	spinlock_t transaction_queue_lock;
	/* Queue for devices with high priority transactions */
	struct lwis_process_queue high_priority_transaction_queue;
};

/*
 * struct lwis_connected_device:
 * This maintains the structure to identify the connected devices
 * to a given bus. This will be used to guard the bus against processing
 * any illegal device entries.
 */
struct lwis_connected_device {
	struct lwis_device *connected_device;
	struct list_head connected_device_node;
};

void lwis_bus_manager_lock_bus(struct lwis_device *lwis_dev);

void lwis_bus_manager_unlock_bus(struct lwis_device *lwis_dev);

struct lwis_bus_manager *lwis_bus_manager_get(struct lwis_device *lwis_dev);

int lwis_bus_manager_create(struct lwis_device *lwis_dev);

void lwis_bus_manager_disconnect_device(struct lwis_device *lwis_dev);

void lwis_bus_manager_process_worker_queue(struct lwis_client *client);

void lwis_bus_manager_flush_worker(struct lwis_device *lwis_dev);

void lwis_bus_manager_list_initialize(void);

void lwis_bus_manager_list_deinitialize(void);

int lwis_bus_manager_connect_client(struct lwis_client *connecting_client);

void lwis_bus_manager_disconnect_client(struct lwis_client *disconnecting_client);

int lwis_bus_manager_add_high_priority_client(struct lwis_client *client);

#endif /* LWIS_BUS_MANAGER_H */
