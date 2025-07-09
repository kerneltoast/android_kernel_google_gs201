// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Bus Manager
 *
 * Copyright 2023 Google LLC.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-bus-manager: " fmt

#include "lwis_device.h"
#include "lwis_bus_manager.h"
#include "lwis_device_i2c.h"
#include "lwis_device_ioreg.h"
#include "lwis_bus_scheduler.h"

bool lwis_bus_manager_debug;
module_param(lwis_bus_manager_debug, bool, 0644);

/*
 * Defines the global list of bus managers shared among various LWIS devices
 * Each manager would control the transfers on a single LWIS bus
 */
static struct mutex bus_manager_list_lock;
static struct lwis_bus_manager_list bus_manager_list;

/*
 * is_valid_connected_device:
 * Makes sure a valid client connected to the bus executes job on this manager
 */
static bool is_valid_connected_device(struct lwis_device *lwis_dev,
				      struct lwis_bus_manager *bus_manager)
{
	struct lwis_connected_device *connected_lwis_device;
	struct list_head *connected_device_node, *connected_device_tmp_node;

	if ((!lwis_dev) || (!bus_manager))
		return false;

	list_for_each_safe(connected_device_node, connected_device_tmp_node,
			   &bus_manager->connected_devices) {
		connected_lwis_device = list_entry(
			connected_device_node, struct lwis_connected_device, connected_device_node);
		if (connected_lwis_device->connected_device == lwis_dev)
			return true;
	}

	return false;
}

/*
 * process_high_priority_transaction_queue:
 * Process high priority transactions on the clients
 * queued on the high priority transaction queue.
 */
static void process_high_priority_transaction_queue(struct lwis_bus_manager *bus_manager)
{
	struct lwis_process_queue *high_priority_process_queue = NULL;
	struct list_head *request, *request_tmp;
	struct lwis_process_request *processing_node;
	struct lwis_client *processing_client = NULL;
	struct lwis_device *processing_dev = NULL;
	unsigned long flags;

	if (!bus_manager)
		return;

	high_priority_process_queue = &bus_manager->high_priority_transaction_queue;
	if (!high_priority_process_queue)
		return;

	spin_lock_irqsave(&bus_manager->transaction_queue_lock, flags);

	if (lwis_process_request_queue_is_empty(high_priority_process_queue)) {
		spin_unlock_irqrestore(&bus_manager->transaction_queue_lock, flags);
		return;
	}
	list_for_each_safe(request, request_tmp, &high_priority_process_queue->head) {
		processing_node = list_entry(request, struct lwis_process_request, request_node);
		processing_client = processing_node->requesting_client;
		processing_dev = processing_client->lwis_dev;
		if (lwis_bus_manager_debug) {
			dev_info(processing_dev->dev,
				 "Processing high priority client %p on bus %s\n",
				 processing_client, bus_manager->bus_name);
		}
		if (is_valid_connected_device(processing_dev, bus_manager)) {
			spin_unlock_irqrestore(&bus_manager->transaction_queue_lock, flags);
			lwis_process_transactions_in_queue(
				processing_node->requesting_client,
				/*process_high_priority_transaction=*/true);
			spin_lock_irqsave(&bus_manager->transaction_queue_lock, flags);
		}

		if (lwis_bus_manager_debug) {
			dev_info(processing_dev->dev,
				 "Removing client %s(%p) from high priority queue on bus %s\n",
				 processing_dev->name, processing_client, bus_manager->bus_name);
		}
		list_del(&processing_node->request_node);
		processing_node->requesting_client = NULL;
		kfree(processing_node);
		processing_node = NULL;
		high_priority_process_queue->number_of_nodes--;
	}

	spin_unlock_irqrestore(&bus_manager->transaction_queue_lock, flags);
}

/*
 * insert_bus_manager_id_in_list:
 * Inserts the newly created instance of bus manager in the list
 */
static int insert_bus_manager_id_in_list(struct lwis_bus_manager *bus_manager, int bus_handle)
{
	struct lwis_bus_manager_identifier *bus_manager_identifier_node = NULL;

	bus_manager_identifier_node =
		kzalloc(sizeof(struct lwis_bus_manager_identifier), GFP_KERNEL);
	if (!bus_manager_identifier_node)
		return -ENOMEM;

	bus_manager_identifier_node->bus_manager_handle = bus_handle;
	bus_manager_identifier_node->bus_manager = bus_manager;
	bus_manager_identifier_node->bus_type = bus_manager->bus_type;
	INIT_LIST_HEAD(&bus_manager_identifier_node->bus_manager_list_node);

	mutex_lock(&bus_manager_list_lock);
	list_add_tail(&bus_manager_identifier_node->bus_manager_list_node,
		      &bus_manager_list.bus_manager_list_head);
	mutex_unlock(&bus_manager_list_lock);

	return 0;
}

/*
 * delete_bus_manager_id_in_list:
 * Deletes the newly created instance of LWIS bus manager in the list
 */
static void delete_bus_manager_id_in_list(int bus_handle, int32_t bus_type)
{
	struct lwis_bus_manager_identifier *bus_manager_identifier_node;
	struct list_head *bus_manager_list_node;
	struct list_head *bus_manager_list_tmp_node;

	if ((bus_type != DEVICE_TYPE_I2C) && (bus_type != DEVICE_TYPE_IOREG))
		return;

	mutex_lock(&bus_manager_list_lock);
	list_for_each_safe(bus_manager_list_node, bus_manager_list_tmp_node,
			   &bus_manager_list.bus_manager_list_head) {
		bus_manager_identifier_node =
			list_entry(bus_manager_list_node, struct lwis_bus_manager_identifier,
				   bus_manager_list_node);

		if (bus_manager_identifier_node->bus_manager_handle != bus_handle)
			continue;

		if (bus_manager_identifier_node->bus_type != bus_type)
			continue;

		list_del(&bus_manager_identifier_node->bus_manager_list_node);
		kfree(bus_manager_identifier_node);
	}
	mutex_unlock(&bus_manager_list_lock);
}

/*
 * find_bus_manager:
 * Returns a valid Bus Manager for a valid bus_handle.
 * Returns NULL if the bus manager hasn't been created for this handle.
 */
static struct lwis_bus_manager *find_bus_manager(int bus_handle, int32_t type)
{
	struct lwis_bus_manager *bus_manager = NULL;
	struct list_head *bus_manager_list_node;
	struct list_head *bus_manager_list_tmp_node;
	struct lwis_bus_manager_identifier *bus_manager_identifier;

	mutex_lock(&bus_manager_list_lock);
	list_for_each_safe(bus_manager_list_node, bus_manager_list_tmp_node,
			   &bus_manager_list.bus_manager_list_head) {
		bus_manager_identifier =
			list_entry(bus_manager_list_node, struct lwis_bus_manager_identifier,
				   bus_manager_list_node);

		if (bus_manager_identifier->bus_manager_handle != bus_handle)
			continue;

		if (bus_manager_identifier->bus_type != type)
			continue;

		bus_manager = bus_manager_identifier->bus_manager;
	}
	mutex_unlock(&bus_manager_list_lock);

	return bus_manager;
}

/*
 * stop_kthread_workers:
 * Stop Bus worker thread, one per bus.
 */
static void stop_kthread_workers(struct lwis_bus_manager *bus_manager, struct lwis_device *lwis_dev)
{
	if (!bus_manager)
		return;

	if (!IS_ERR(bus_manager->bus_worker_thread)) {
		if (lwis_bus_manager_debug)
			dev_err(lwis_dev->dev, "%s: destroying LWIS Bus Manager thread\n",
				__func__);

		kthread_stop(bus_manager->bus_worker_thread);
	}
}

/*
 * create_kthread_workers:
 * Creates worker threads, one per bus.
 */
static int create_kthread_workers(struct lwis_bus_manager *bus_manager,
				  struct lwis_device *lwis_dev)
{
	char bus_thread_name[LWIS_MAX_NAME_STRING_LEN];

	scnprintf(bus_thread_name, LWIS_MAX_NAME_STRING_LEN, "lwis_%s", bus_manager->bus_name);
	kthread_init_worker(&bus_manager->bus_worker);
	bus_manager->bus_worker_thread =
		kthread_run(kthread_worker_fn, &bus_manager->bus_worker, bus_thread_name);
	if (IS_ERR(bus_manager->bus_worker_thread)) {
		dev_err(lwis_dev->dev, "Creation of bus_worker_thread failed for bus %s\n",
			bus_manager->bus_name);
		return -EINVAL;
	}
	return 0;
}

/*
 * check_thread_priority:
 * Checks if the lwis device being connected has the same priority as other devices
 * connected on the same bus.
 * Prints a warning message if there is a difference between the priorities on the
 * device threads.
 */
static void check_thread_priority(struct lwis_bus_manager *bus_manager,
				  struct lwis_device *lwis_dev)
{
	if (bus_manager->bus_thread_priority != lwis_dev->transaction_thread_priority) {
		dev_warn(lwis_dev->dev,
			 "Mismatching thread priority for Bus manager(%d), device(%d)\n",
			 bus_manager->bus_thread_priority, lwis_dev->transaction_thread_priority);
	}
}

/*
 * set_thread_priority:
 * Sets the priority for bus threads.
 */
static int set_thread_priority(struct lwis_bus_manager *bus_manager, struct lwis_device *lwis_dev)
{
	int ret = 0;

	bus_manager->bus_thread_priority = lwis_dev->transaction_thread_priority;
	if (bus_manager->bus_thread_priority != 0) {
		ret = lwis_set_kthread_priority(lwis_dev, bus_manager->bus_worker_thread,
						bus_manager->bus_thread_priority);
	}
	return ret;
}

/*
 * set_bus_manager_name:
 * Builds and sets the Bus manager name.
 */
static void set_bus_manager_name(struct lwis_bus_manager *bus_manager)
{
	switch (bus_manager->bus_type) {
	case DEVICE_TYPE_I2C:
		scnprintf(bus_manager->bus_name, LWIS_MAX_NAME_STRING_LEN, "I2C_Bus_%X",
			  bus_manager->bus_id);
		break;
	case DEVICE_TYPE_IOREG:
		scnprintf(bus_manager->bus_name, LWIS_MAX_NAME_STRING_LEN, "IOREG_Bus_%X",
			  bus_manager->bus_id);
		break;
	default:
		break;
	}
}

/*
 * destroy_bus_manager:
 * Destroys this instance of the LWIS bus manager.
 */
static void destroy_bus_manager(struct lwis_bus_manager *bus_manager, struct lwis_device *lwis_dev)
{
	int i = 0;
	unsigned long flags;

	if (!bus_manager)
		return;

	dev_dbg(lwis_dev->dev, "Destroying LWIS Bus Manager: %s\n", bus_manager->bus_name);
	mutex_lock(&bus_manager->process_queue_lock);
	for (i = 0; i < MAX_DEVICE_PRIORITY_LEVELS; i++)
		lwis_process_request_queue_destroy(&bus_manager->bus_process_queue[i]);
	mutex_unlock(&bus_manager->process_queue_lock);

	spin_lock_irqsave(&bus_manager->transaction_queue_lock, flags);
	lwis_process_request_queue_destroy(&bus_manager->high_priority_transaction_queue);
	spin_unlock_irqrestore(&bus_manager->transaction_queue_lock, flags);

	delete_bus_manager_id_in_list(bus_manager->bus_id, bus_manager->bus_type);

	kfree(bus_manager);
}

/*
 * connect_device_to_bus_manager:
 * Connects a lwis device to this instance of the bus manager.
 */
static int connect_device_to_bus_manager(struct lwis_bus_manager *bus_manager,
					 struct lwis_device *lwis_dev)
{
	int ret = 0;
	struct lwis_connected_device *connect_lwis_device;

	connect_lwis_device = kzalloc(sizeof(struct lwis_connected_device), GFP_KERNEL);
	if (!connect_lwis_device)
		return -ENOMEM;

	connect_lwis_device->connected_device = lwis_dev;
	INIT_LIST_HEAD(&connect_lwis_device->connected_device_node);
	list_add_tail(&connect_lwis_device->connected_device_node, &bus_manager->connected_devices);
	bus_manager->number_of_connected_devices++;

	return ret;
}

static bool device_priority_is_valid(int device_priority)
{
	return ((device_priority >= DEVICE_HIGH_PRIORITY) &&
		(device_priority <= DEVICE_LOW_PRIORITY));
}

/*
 * lwis_bus_manager_process_worker_queue:
 * Function to be called by LWIS bus manager worker thread to
 * pick the next LWIS client that is scheduled for transfer.
 * The process queue will be processed in order of the device priority.
 */
void lwis_bus_manager_process_worker_queue(struct lwis_client *client)
{
	struct lwis_device *lwis_dev;
	struct lwis_bus_manager *bus_manager;
	int i;

	/* The transfers will be processed in fifo order */
	struct lwis_client *client_to_process;
	struct lwis_device *lwis_dev_to_process;
	struct lwis_process_queue *process_queue;
	struct lwis_process_request *process_request;

	struct list_head *client_node, *client_tmp_node;

	lwis_dev = client->lwis_dev;
	bus_manager = lwis_bus_manager_get(lwis_dev);

	if (lwis_bus_manager_debug)
		dev_info(lwis_dev->dev, "%s scheduled by %s\n", bus_manager->bus_name,
			 lwis_dev->name);

	mutex_lock(&bus_manager->process_queue_lock);
	for (i = 0; i < MAX_DEVICE_PRIORITY_LEVELS; i++) {
		process_queue = &bus_manager->bus_process_queue[i];
		list_for_each_safe(client_node, client_tmp_node, &process_queue->head) {
			process_high_priority_transaction_queue(bus_manager);

			process_request =
				list_entry(client_node, struct lwis_process_request, request_node);
			if (!process_request) {
				dev_err(lwis_dev->dev, "LWIS Bus Worker process_request is null\n");
				break;
			}

			client_to_process = process_request->requesting_client;
			if (!client_to_process) {
				dev_err(lwis_dev->dev,
					"LWIS Bus Worker client_to_process is null\n");
				break;
			}

			lwis_dev_to_process = client_to_process->lwis_dev;
			if (!lwis_dev_to_process) {
				dev_err(lwis_dev->dev,
					"LWIS Bus Worker lwis_dev_to_process is null\n");
				break;
			}

			if (lwis_bus_manager_debug)
				dev_info(lwis_dev_to_process->dev, "Processing client start %s\n",
					 lwis_dev_to_process->name);

			if (is_valid_connected_device(lwis_dev_to_process, bus_manager)) {
				lwis_process_transactions_in_queue(
					client_to_process,
					/*process_high_priority_transaction=*/false);
				lwis_process_periodic_io_in_queue(client_to_process);
			}

			if (lwis_bus_manager_debug)
				dev_info(lwis_dev_to_process->dev, "Processing client end %s\n",
					 lwis_dev_to_process->name);
		}
	}
	mutex_unlock(&bus_manager->process_queue_lock);
}

/*
 * lwis_bus_manager_create:
 * Creates a new instance of bus manager.
 */
int lwis_bus_manager_create(struct lwis_device *lwis_dev)
{
	int ret;
	int i;
	struct lwis_bus_manager *bus_manager;
	struct lwis_i2c_device *i2c_dev;
	struct lwis_ioreg_device *ioreg_dev;
	int bus_handle;
	int32_t bus_type;

	/* Create Bus Manager for Specific Device Types */
	switch (lwis_dev->type) {
	case DEVICE_TYPE_I2C:
		i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
		bus_handle = i2c_dev->adapter->nr;
		bus_type = lwis_dev->type;
		break;
	case DEVICE_TYPE_IOREG:
		ioreg_dev = container_of(lwis_dev, struct lwis_ioreg_device, base_dev);
		bus_handle = ioreg_dev->device_group;
		bus_type = lwis_dev->type;
		break;
	default:
		/* Managed Device Types: I2C and IOREG */
		return 0;
	}

	bus_manager = find_bus_manager(bus_handle, bus_type);
	if (!bus_manager) {
		bus_manager = kzalloc(sizeof(struct lwis_bus_manager), GFP_KERNEL);
		if (!bus_manager)
			return -ENOMEM;

		bus_manager->bus_id = bus_handle;
		bus_manager->bus_type = bus_type;
		set_bus_manager_name(bus_manager);

		/* Mutex and Lock initializations */
		mutex_init(&bus_manager->bus_lock);
		mutex_init(&bus_manager->process_queue_lock);
		spin_lock_init(&bus_manager->transaction_queue_lock);

		/* List initializations */
		INIT_LIST_HEAD(&bus_manager->connected_devices);

		/* Create a transfer process queue */
		for (i = 0; i < MAX_DEVICE_PRIORITY_LEVELS; i++)
			lwis_process_request_queue_initialize(&bus_manager->bus_process_queue[i]);

		lwis_process_request_queue_initialize(
			&bus_manager->high_priority_transaction_queue);

		/* Insert this instance of bus manager in the bus manager list */
		ret = insert_bus_manager_id_in_list(bus_manager, bus_handle);
		if (ret < 0)
			goto error_creating_bus_manager;

		/* Create worker thread to serve this bus manager */
		ret = create_kthread_workers(bus_manager, lwis_dev);
		if (ret < 0)
			goto error_creating_bus_manager;

		/* Set priority for the worker threads */
		ret = set_thread_priority(bus_manager, lwis_dev);
		if (ret < 0)
			goto error_creating_bus_manager;
	}

	/* Check the current device's thread priority with respect to the bus priority */
	check_thread_priority(bus_manager, lwis_dev);

	/* Connect this lwis device to the Bus manager found/created */
	ret = connect_device_to_bus_manager(bus_manager, lwis_dev);
	if (ret < 0) {
		dev_err(lwis_dev->dev,
			"Failed to connect device to the corresponding Bus Manager\n");
		goto error_creating_bus_manager;
	}

	dev_info(lwis_dev->dev,
		 "LWIS Bus Manager: %s Connected Device: %s Connected device count: %d\n",
		 bus_manager->bus_name, lwis_dev->name, bus_manager->number_of_connected_devices);

	/* Assign created/found bus manager to specific device type */
	switch (lwis_dev->type) {
	case DEVICE_TYPE_I2C:
		i2c_dev->i2c_bus_manager = bus_manager;
		break;
	case DEVICE_TYPE_IOREG:
		ioreg_dev->ioreg_bus_manager = bus_manager;
		break;
	default:
		break;
	}

	return ret;

error_creating_bus_manager:
	dev_err(lwis_dev->dev, "Error creating LWIS Bus Manager\n");
	delete_bus_manager_id_in_list(bus_handle, bus_type);
	stop_kthread_workers(bus_manager, lwis_dev);
	kfree(bus_manager);
	return -EINVAL;
}

/*
 * lwis_bus_manager_disconnect_device:
 * Disconnects a lwis device from this instance of the LWIS bus manager.
 * Doesn't destroy the instance of LWIS bus manager.
 */
void lwis_bus_manager_disconnect_device(struct lwis_device *lwis_dev)
{
	struct lwis_bus_manager *bus_manager;
	struct lwis_connected_device *connected_lwis_device;
	struct list_head *connected_device_node, *connected_device_tmp_node;
	struct lwis_i2c_device *i2c_dev;
	struct lwis_ioreg_device *ioreg_dev;

	bus_manager = lwis_bus_manager_get(lwis_dev);
	if (!bus_manager)
		return;

	list_for_each_safe(connected_device_node, connected_device_tmp_node,
			   &bus_manager->connected_devices) {
		connected_lwis_device = list_entry(
			connected_device_node, struct lwis_connected_device, connected_device_node);

		/* Reset the bus manager pointer for this LWIS device. */
		if (lwis_check_device_type(lwis_dev, DEVICE_TYPE_I2C)) {
			i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
			i2c_dev->i2c_bus_manager = NULL;
		} else if (lwis_check_device_type(lwis_dev, DEVICE_TYPE_IOREG)) {
			ioreg_dev = container_of(lwis_dev, struct lwis_ioreg_device, base_dev);
			ioreg_dev->ioreg_bus_manager = NULL;
		}

		if (connected_lwis_device->connected_device == lwis_dev) {
			list_del(&connected_lwis_device->connected_device_node);
			kfree(connected_lwis_device);
			bus_manager->number_of_connected_devices--;

			/*
			 * Destroy the bus manager instance if there
			 * are no more LWIS devices connected to it.
			 */
			if (bus_manager->number_of_connected_devices == 0)
				destroy_bus_manager(bus_manager, lwis_dev);

			return;
		}
	}
}

/*
 * lwis_bus_manager_lock_bus:
 * Locks the LWIS bus for a given LWIS Device.
 */
void lwis_bus_manager_lock_bus(struct lwis_device *lwis_dev)
{
	struct lwis_bus_manager *bus_manager = lwis_bus_manager_get(lwis_dev);

	if (bus_manager)
		mutex_lock(&bus_manager->bus_lock);
}

/*
 * lwis_bus_manager_unlock_bus:
 * Unlocks the LWIS bus for a given LWIS Device.
 */
void lwis_bus_manager_unlock_bus(struct lwis_device *lwis_dev)
{
	struct lwis_bus_manager *bus_manager = lwis_bus_manager_get(lwis_dev);

	if (bus_manager)
		mutex_unlock(&bus_manager->bus_lock);
}

/*
 * lwis_bus_manager_get:
 * Gets LWIS Bus Manager for a given lwis device.
 */
struct lwis_bus_manager *lwis_bus_manager_get(struct lwis_device *lwis_dev)
{
	struct lwis_i2c_device *i2c_dev;
	struct lwis_ioreg_device *ioreg_dev;

	switch (lwis_dev->type) {
	case DEVICE_TYPE_I2C:
		i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
		if (i2c_dev)
			return i2c_dev->i2c_bus_manager;
		break;
	case DEVICE_TYPE_IOREG:
		ioreg_dev = container_of(lwis_dev, struct lwis_ioreg_device, base_dev);
		if (ioreg_dev)
			return ioreg_dev->ioreg_bus_manager;
		break;
	default:
		break;
	}
	return NULL;
}

/*
 * lwis_bus_manager_flush_worker:
 * Flushes the LWIS Bus Manager worker.
 */
void lwis_bus_manager_flush_worker(struct lwis_device *lwis_dev)
{
	struct lwis_bus_manager *bus_manager = lwis_bus_manager_get(lwis_dev);

	if (bus_manager)
		kthread_flush_worker(&bus_manager->bus_worker);
}

/*
 * lwis_bus_manager_list_initialize:
 * Initializes bus manager global list. This is the list that holds
 * actual bus manager pointers for a given physical LWIS Bus connection.
 */
void lwis_bus_manager_list_initialize(void)
{
	mutex_init(&bus_manager_list_lock);
	INIT_LIST_HEAD(&bus_manager_list.bus_manager_list_head);
}

/*
 * lwis_bus_manager_list_deinitialize:
 * Deinitializes bus manager global list.
 */
void lwis_bus_manager_list_deinitialize(void)
{
	struct list_head *bus_manager_list_node, *bus_manager_list_tmp_node;
	struct lwis_bus_manager_identifier *bus_manager_identifier;

	mutex_lock(&bus_manager_list_lock);
	list_for_each_safe(bus_manager_list_node, bus_manager_list_tmp_node,
			   &bus_manager_list.bus_manager_list_head) {
		bus_manager_identifier =
			list_entry(bus_manager_list_node, struct lwis_bus_manager_identifier,
				   bus_manager_list_node);
		bus_manager_identifier->bus_manager = NULL;
		list_del(&bus_manager_identifier->bus_manager_list_node);
		kfree(bus_manager_identifier);
	}
	mutex_unlock(&bus_manager_list_lock);
}

/*
 * do_client_connect:
 * Connect client to the bus manager processing node list.
 */
static int do_client_connect(struct lwis_client *connecting_client,
			     struct lwis_bus_manager *bus_manager,
			     struct lwis_process_request *client_node, int device_priority)
{
	struct lwis_process_queue *process_queue;
	struct lwis_process_request *connecting_client_node;

	process_queue = &bus_manager->bus_process_queue[device_priority];

	if ((client_node) && (client_node->requesting_client == connecting_client)) {
		dev_info(connecting_client->lwis_dev->dev,
			 "LWIS client already connected %s(%p) to bus %s\n",
			 connecting_client->lwis_dev->name, connecting_client,
			 bus_manager->bus_name);
		return 0;
	}

	connecting_client_node = kzalloc(sizeof(struct lwis_process_request), GFP_KERNEL);
	if (!connecting_client_node)
		return -ENOMEM;

	connecting_client_node->requesting_client = connecting_client;
	INIT_LIST_HEAD(&connecting_client_node->request_node);
	list_add_tail(&connecting_client_node->request_node, &process_queue->head);
	process_queue->number_of_nodes++;
	dev_info(connecting_client->lwis_dev->dev, "Connecting client %s(%p) to bus %s\n",
		 connecting_client->lwis_dev->name, connecting_client, bus_manager->bus_name);
	return 0;
}

/*
 * do_client_disconnect:
 * Disconnect client from the bus manager processing node list.
 */
static int do_client_disconnect(struct lwis_client *disconnecting_client,
				struct lwis_bus_manager *bus_manager,
				struct lwis_process_request *disconnecting_client_node,
				int device_priority)
{
	struct lwis_process_queue *process_queue = &bus_manager->bus_process_queue[device_priority];

	dev_info(disconnecting_client->lwis_dev->dev,
		 "Disconnecting LWIS client %s(%p) from bus %s\n",
		 disconnecting_client->lwis_dev->name, disconnecting_client, bus_manager->bus_name);
	list_del(&disconnecting_client_node->request_node);
	disconnecting_client_node->requesting_client = NULL;
	kfree(disconnecting_client_node);
	process_queue->number_of_nodes--;
	return 0;
}

/*
 * find_client:
 * Find the client on the bus manager to connect/disconnect from the processing
 * node list.
 */
static int find_client(int device_priority, struct lwis_bus_manager *bus_manager,
		       struct lwis_client *client, enum lwis_client_connection connection_status)
{
	struct list_head *request, *request_tmp;
	struct lwis_process_queue *process_queue;
	struct lwis_process_request *client_node;

	process_queue = &bus_manager->bus_process_queue[device_priority];
	if (!lwis_process_request_queue_is_empty(process_queue)) {
		list_for_each_safe(request, request_tmp, &process_queue->head) {
			client_node =
				list_entry(request, struct lwis_process_request, request_node);
			if (client_node->requesting_client != client)
				continue;

			if (connection_status == CLIENT_CONNECT) {
				return do_client_connect(client, bus_manager, client_node,
							 device_priority);
			} else if (connection_status == CLIENT_DISCONNECT) {
				return do_client_disconnect(client, bus_manager, client_node,
							    device_priority);
			} else {
				dev_err(client->lwis_dev->dev,
					"Invalid client connection status %d", connection_status);
				return -EINVAL;
			}
		}
	}

	/*
	 * Connect the client if:
	 * 1. The process queue is empty.
	 * 2. If there is no matching client found durnig the search in the exiting queue.
	 */
	if (connection_status == CLIENT_CONNECT)
		return do_client_connect(client, bus_manager, NULL, device_priority);

	return 0;
}

/*
 * get_device_priority_and_bus_manager:
 * Get the device priority and LWIS Bus Manager handle for the client.
 */
static int get_device_priority_and_bus_manager(struct lwis_client *client, int *device_priority,
					       struct lwis_bus_manager **bus_manager)
{
	struct lwis_i2c_device *i2c_dev;
	struct lwis_ioreg_device *ioreg_dev;

	/*
	 * 1. This device type check ensures that the devices that do
	 * not need a bus manager do not get a failure due to
	 * bus manager being null when trying to open the lwis client.
	 * 2. Gets the device priority based on the device type.
	 */
	switch (client->lwis_dev->type) {
	case DEVICE_TYPE_I2C:
		i2c_dev = container_of(client->lwis_dev, struct lwis_i2c_device, base_dev);
		*device_priority = i2c_dev->device_priority;
		break;
	case DEVICE_TYPE_IOREG:
		ioreg_dev = container_of(client->lwis_dev, struct lwis_ioreg_device, base_dev);
		*device_priority = ioreg_dev->device_priority;
		break;
	default:
		return 0;
	}

	if (!device_priority_is_valid(*device_priority)) {
		dev_err(client->lwis_dev->dev, "Invalid LWIS bus device priority %d\n",
			*device_priority);
		return -EINVAL;
	}

	/*
	 * This check ensures that the LWIS devices have a valid bus manager to associate
	 * the lwis clients.
	 * Error handling: Default behaviour of IOREG devices is to process transactions
	 * on their own threads. This does not require a manager to serialize and manage
	 * the transactions and events for IOREG devices. Hence, NULL bus manager for
	 * IOREG devices is not necessarily an error.
	 */
	*bus_manager = lwis_bus_manager_get(client->lwis_dev);
	if (!(*bus_manager)) {
		switch (client->lwis_dev->type) {
		case DEVICE_TYPE_I2C:
			dev_err(client->lwis_dev->dev, "LWIS bus manager is NULL\n");
			return -EINVAL;
		case DEVICE_TYPE_IOREG:
			dev_info(client->lwis_dev->dev,
				 "No LWIS bus manager associated with this device.\n");
			return 0;
		default:
			return 0;
		}
	}
	return 0;
}

/*
 * lwis_bus_manager_connect_client:
 * Connects a lwis client to the bus manager to be processed by the worker.
 * The client will be connected to the appropriate priority queue based
 * on the LWIS device priority specified in the dts for the LWIS device node.
 * LWIS client is always connected when a new instance of client is
 * created.
 */
int lwis_bus_manager_connect_client(struct lwis_client *connecting_client)
{
	int ret;
	int device_priority = MAX_DEVICE_PRIORITY_LEVELS;
	struct lwis_bus_manager *bus_manager = NULL;

	ret = get_device_priority_and_bus_manager(connecting_client, &device_priority,
						  &bus_manager);
	if (ret || !bus_manager)
		return ret;

	/*
	 * Search for existing client node in the queue, if client is already connected
	 * to this bus then don't create a new client node.
	 */
	mutex_lock(&bus_manager->process_queue_lock);
	ret = find_client(device_priority, bus_manager, connecting_client, CLIENT_CONNECT);
	mutex_unlock(&bus_manager->process_queue_lock);

	return ret;
}

/*
 * lwis_bus_manager_disconnect_client:
 * Disconnects a lwis client to the bus manager. This will make sure that
 * the released client is not processed further by the worker.
 * The client will be disconnected from the appropriate priority queue based
 * on the device priority specified in the dts for the LWIS device node.
 * LWIS client is always disconnected when the instance of client is
 * released/destroyed.
 */
void lwis_bus_manager_disconnect_client(struct lwis_client *disconnecting_client)
{
	int ret;
	int device_priority = MAX_DEVICE_PRIORITY_LEVELS;
	struct lwis_bus_manager *bus_manager = NULL;

	ret = get_device_priority_and_bus_manager(disconnecting_client, &device_priority,
						  &bus_manager);
	if (ret || !bus_manager)
		return;

	mutex_lock(&bus_manager->process_queue_lock);
	find_client(device_priority, bus_manager, disconnecting_client, CLIENT_DISCONNECT);
	mutex_unlock(&bus_manager->process_queue_lock);
}

/*
 * lwis_bus_manager_add_high_priority_client:
 * Add clients to high priority transaction queue that will process
 * the transactions submitted on this device first regardless of
 * the device's priority. Only the transactions marked as high priority
 * are executed through this queue. All other transactions are processed
 * in the regular device priority order.
 */
int lwis_bus_manager_add_high_priority_client(struct lwis_client *client)
{
	struct lwis_process_queue *high_priority_process_queue;
	struct lwis_process_request *high_priority_client_node;
	struct list_head *request, *request_tmp;
	struct lwis_process_request *search_node;
	struct lwis_bus_manager *bus_manager = lwis_bus_manager_get(client->lwis_dev);
	unsigned long flags;
	bool add_node = true;

	/*
	 * Bus manager will be NULL for non-I2C and non-IOREG devices.
	 * Returning success here if bus manager is NULL will ensure that
	 * non-I2C and non-IOREG devices are not added to the Bus
	 * high priority transaction queue.
	 */
	if (!bus_manager)
		return 0;

	spin_lock_irqsave(&bus_manager->transaction_queue_lock, flags);

	high_priority_process_queue = &bus_manager->high_priority_transaction_queue;
	if (!lwis_process_request_queue_is_empty(high_priority_process_queue)) {
		list_for_each_safe(request, request_tmp, &high_priority_process_queue->head) {
			search_node =
				list_entry(request, struct lwis_process_request, request_node);
			if (search_node->requesting_client == client) {
				if (lwis_bus_manager_debug) {
					dev_info(
						client->lwis_dev->dev,
						"LWIS client %s(%p) already added to high priority queue on bus %s\n",
						client->lwis_dev->name, client,
						bus_manager->bus_name);
				}
				add_node = false;
				break;
			}
		}
	}

	if (add_node) {
		high_priority_client_node =
			kzalloc(sizeof(struct lwis_process_request), GFP_ATOMIC);
		if (!high_priority_client_node) {
			spin_unlock_irqrestore(&bus_manager->transaction_queue_lock, flags);
			return -ENOMEM;
		}

		high_priority_client_node->requesting_client = client;
		INIT_LIST_HEAD(&high_priority_client_node->request_node);
		list_add_tail(&high_priority_client_node->request_node,
			      &high_priority_process_queue->head);
		high_priority_process_queue->number_of_nodes++;
		if (lwis_bus_manager_debug) {
			dev_info(client->lwis_dev->dev,
				 "Adding client %s(%p) to high priority queue on bus %s\n",
				 client->lwis_dev->name, client, bus_manager->bus_name);
		}
	}

	spin_unlock_irqrestore(&bus_manager->transaction_queue_lock, flags);
	return 0;
}
