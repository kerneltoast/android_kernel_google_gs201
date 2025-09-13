/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Base Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_DEVICE_H_
#define LWIS_DEVICE_H_

#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/workqueue.h>

#include "lwis_clock.h"
#include "lwis_commands.h"
#include "lwis_event.h"
#include "lwis_gpio.h"
#include "lwis_interrupt.h"
#include "lwis_phy.h"
#include "lwis_regulator.h"
#include "lwis_transaction.h"

#define LWIS_TOP_DEVICE_COMPAT "google,lwis-top-device"
#define LWIS_I2C_DEVICE_COMPAT "google,lwis-i2c-device"
#define LWIS_IOREG_DEVICE_COMPAT "google,lwis-ioreg-device"
#define LWIS_SLC_DEVICE_COMPAT "google,lwis-slc-device"
#define LWIS_DPM_DEVICE_COMPAT "google,lwis-dpm-device"
#define LWIS_TEST_DEVICE_COMPAT "google,lwis-test-device"
#define LWIS_SPI_DEVICE_COMPAT "google,lwis-spi-device"

#define EVENT_HASH_BITS 8
#define BUFFER_HASH_BITS 8
#define TRANSACTION_HASH_BITS 8
#define PERIODIC_IO_HASH_BITS 8
#define BTS_UNSUPPORTED -1
#define MAX_UNIFIED_POWER_DEVICE 8

/* enum lwis_client_flush_state
 * Client flush states indicate if the client has been issued a
 * flush on the transaction worker threads.
 * Client will move to FLUSHING state only when a direct call to
 * lwis_transaction_client_flush has been made.
 * Once the flush is complete, the client will transition back
 * to NOT_FLUSHING state.
 */
enum lwis_client_flush_state { NOT_FLUSHING, FLUSHING };

/* Forward declaration for lwis_device. This is needed for the declaration for
 * lwis_device_subclass_operations data struct.
 */
struct lwis_device;

/* Forward declaration of a platform specific struct used by platform funcs */
struct lwis_platform;

/* Forward declaration of lwis allocator block manager */
struct lwis_allocator_block_mgr;
int lwis_allocator_init(struct lwis_device *lwis_dev);
void lwis_allocator_release(struct lwis_device *lwis_dev);

/*
 * struct lwis_dev_pwr_ref_cnt
 * This struct is to store the power up/down sequence reference count
 */
struct lwis_dev_pwr_ref_cnt {
	struct device_node *dev_node_seq;
	struct lwis_device *hold_dev;
	int count;
};

/*
 *  struct lwis_core
 *  This struct applies to all LWIS devices that are defined in the
 *  device tree.
 */
struct lwis_core {
	struct class *dev_class;
	struct idr *idr;
	struct cdev *chr_dev;
	struct mutex lock;
	dev_t lwis_devt;
	int device_major;
	struct list_head lwis_dev_list;
	struct lwis_dev_pwr_ref_cnt unified_dev_pwr_map[MAX_UNIFIED_POWER_DEVICE];
	struct dentry *dbg_root;
};

/* struct lwis_device_subclass_operations
 * This struct contains the 'virtual' functions for lwis_device subclasses
 * that are called into by various lwis_device_* code if they are not NULL
 * to allow the subclasses to customize certain behavior
 */
struct lwis_device_subclass_operations {
	/* Called by lwis_device when device register needs to be read/written */
	int (*register_io)(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
			   int access_size);
	/* Called by lwis_device when a read/write memory barrier needs to be inserted */
	int (*register_io_barrier)(struct lwis_device *lwis_dev, bool use_read_barrier,
				   bool use_write_barrier);
	/* called by lwis_device when enabling the device */
	int (*device_enable)(struct lwis_device *lwis_dev);
	/* called by lwis_device when disabling the device */
	int (*device_disable)(struct lwis_device *lwis_dev);
	/* Called by lwis_device any time a particular event_id needs to be
	 * enabled or disabled by the device
	 */
	int (*event_enable)(struct lwis_device *lwis_dev, int64_t event_id, bool enabled);
	/* Called by lwis_device any time flags are updated */
	int (*event_flags_updated)(struct lwis_device *lwis_dev, int64_t event_id,
				   uint64_t old_flags, uint64_t new_flags);
	/* Called by lwis_device any time an event is emitted
	 * Called with lwis_dev->lock locked and IRQs disabled
	 */
	int (*event_emitted)(struct lwis_device *lwis_dev, int64_t event_id, void **payload_ptrptr,
			     size_t *payload_size_ptr);
	/* Called by lwis_device when device closes */
	int (*close)(struct lwis_device *lwis_dev);
};

/*
 * struct lwis_device_power_sequence_info
 * This struct is to store the power up/down sequence information
 */
struct lwis_device_power_sequence_info {
	char name[LWIS_MAX_NAME_STRING_LEN];
	char type[LWIS_MAX_NAME_STRING_LEN];
	int delay_us;
};

/*
 * struct lwis_device_power_sequence_list
 * This struct is to store the power up/down sequence list
 */
struct lwis_device_power_sequence_list {
	struct lwis_device_power_sequence_info *seq_info;
	/* Count of power sequence info */
	int count;
};

/* struct lwis_client_debug_info
 * This struct applies to each of the LWIS clients, and the purpose is to
 * store information in help debugability.
 */
#define TRANSACTION_DEBUG_HISTORY_SIZE 8
struct lwis_client_debug_info {
	struct lwis_transaction_history transaction_hist[TRANSACTION_DEBUG_HISTORY_SIZE];
	int cur_transaction_hist_idx;
};

/*
 * struct lwis_register_io_info
 * This struct is to store the register read write info
 */
struct lwis_register_io_info {
	struct lwis_io_entry io_entry;
	size_t access_size;
	int64_t start_timestamp;
};

/* struct lwis_device_debug_info
 * This struct applies to each of the LWIS devices, and the purpose is to
 * store information in help debugability.
 */
#define EVENT_DEBUG_HISTORY_SIZE 16
#define IO_ENTRY_DEBUG_HISTORY_SIZE 8
struct lwis_device_debug_info {
	struct lwis_device_event_state_history event_hist[EVENT_DEBUG_HISTORY_SIZE];
	int cur_event_hist_idx;
	struct lwis_register_io_info io_entry_hist[IO_ENTRY_DEBUG_HISTORY_SIZE];
	int cur_io_entry_hist_idx;
};

/*
 *  struct lwis_device
 *  This struct applies to each of the LWIS devices, e.g. /dev/lwis*
 */
#define MAX_BTS_BLOCK_NUM 4
struct lwis_device {
	struct lwis_platform *platform;
	int id;
	int32_t type;
	char name[LWIS_MAX_NAME_STRING_LEN];
	struct device *dev;
	struct device *k_dev;
	struct platform_device *plat_dev;
	bool reset_gpios_present;
	struct gpio_descs *reset_gpios;
	bool enable_gpios_present;
	struct gpio_descs *enable_gpios;
	bool shared_enable_gpios_present;
	struct gpio_descs *shared_enable_gpios;
	uint32_t enable_gpios_settle_time;
	struct lwis_regulator_list *regulators;
	struct lwis_clock_list *clocks;
	struct pinctrl *mclk_ctrl;
	bool mclk_present;
	uint32_t shared_pinctrl;
	struct lwis_interrupt_list *irqs;
	struct lwis_phy_list *phys;
	struct list_head dev_list;

	/* Enabled state of the device */
	int enabled;
	/* Mark if the client called device suspend */
	bool is_suspended;
	/* Mutex used to synchronize access between clients */
	struct mutex interclient_lock;
	/* Spinlock used to synchronize access to the device struct */
	spinlock_t lock;
	/* List of clients opened for this device */
	struct list_head clients;
	/* Hash table of device-specific per-event state/control data */
	DECLARE_HASHTABLE(event_states, EVENT_HASH_BITS);
	/* Virtual function table for sub classes */
	struct lwis_device_subclass_operations vops;
	/* Heartbeat timer structure */
	struct timer_list heartbeat_timer;
	/* Register-related properties */
	unsigned int native_addr_bitwidth;
	unsigned int native_value_bitwidth;
	/* Point to lwis_top_dev */
	struct lwis_device *top_dev;
#ifdef CONFIG_DEBUG_FS
	/* DebugFS directory and files */
	struct dentry *dbg_dir;
	struct dentry *dbg_dev_info_file;
	struct dentry *dbg_event_file;
	struct dentry *dbg_transaction_file;
	struct dentry *dbg_buffer_file;
	struct dentry *dbg_reg_io_file;
#endif
	/* Structure to store info to help debugging device data */
	struct lwis_device_debug_info debug_info;

	/* clock family this device belongs to */
	int clock_family;
	/* number of BTS blocks */
	int bts_block_num;
	/* BTS block names*/
	const char *bts_block_names[MAX_BTS_BLOCK_NUM];
	/* indexes to bandwidth traffic shaper */
	int bts_indexes[MAX_BTS_BLOCK_NUM];
	/* BTS scenario name */
	const char *bts_scenario_name;
	/* BTS scenario index */
	unsigned int bts_scenario;

	/* Power sequence handler */
	struct device_node *power_seq_handler;
	/* Power up sequence information */
	struct lwis_device_power_sequence_list *power_up_sequence;
	/* Power down sequence information */
	struct lwis_device_power_sequence_list *power_down_sequence;
	/* Suspend sequence information */
	struct lwis_device_power_sequence_list *suspend_sequence;
	/* Resume sequence information */
	struct lwis_device_power_sequence_list *resume_sequence;
	/* GPIOs list */
	struct list_head gpios_list;
	/* GPIO interrupts list */
	struct lwis_gpios_info irq_gpios_info;
	/* Is power up to suspend mode */
	bool power_up_to_suspend;

	/* Power management hibernation state of the device */
	int pm_hibernation;

	/* Is device read only */
	bool is_read_only;
	/* Adjust thread priority */
	u32 transaction_thread_priority;

	/* LWIS allocator block manager */
	struct lwis_allocator_block_mgr *block_mgr;
	spinlock_t allocator_lock;

	/* Worker thread */
	struct kthread_worker transaction_worker;
	struct task_struct *transaction_worker_thread;
	/* Limit on number of transactions to be processed at a time */
	int transaction_process_limit;
};

/*
 *  struct lwis_client
 *  This struct applies to each client that uses a LWIS device, i.e. each
 *  application that calls open() on a /dev/lwis* device.
 */
struct lwis_client {
	struct mutex lock;
	struct lwis_device *lwis_dev;
	/* Hash table of events controlled by userspace in this client */
	DECLARE_HASHTABLE(event_states, EVENT_HASH_BITS);
	/* Queue of pending events to be consumed by userspace */
	struct list_head event_queue;
	size_t event_queue_size;
	struct list_head error_event_queue;
	size_t error_event_queue_size;
	/* Spinlock used to synchronize access to event states and queue */
	spinlock_t event_lock;
	/* Event wait queue for waking up userspace */
	wait_queue_head_t event_wait_queue;
	/* Hash table of allocated buffers keyed by file descriptor. */
	DECLARE_HASHTABLE(allocated_buffers, BUFFER_HASH_BITS);
	/* Hash table of enrolled buffers keyed by dvaddr */
	DECLARE_HASHTABLE(enrolled_buffers, BUFFER_HASH_BITS);
	/* Hash table of transactions keyed by trigger event ID */
	DECLARE_HASHTABLE(transaction_list, TRANSACTION_HASH_BITS);
	/* Spinlock used to synchronize access to transaction data structs */
	spinlock_t transaction_lock;
	/* List of transaction triggers */
	struct list_head transaction_process_queue;
	/* Transaction counter, which also provides transacton ID */
	int64_t transaction_counter;
	/* Hash table of pending transactions keyed by transaction id */
	DECLARE_HASHTABLE(pending_transactions, TRANSACTION_HASH_BITS);
	/* Hash table of hrtimer keyed by time out duration */
	DECLARE_HASHTABLE(timer_list, PERIODIC_IO_HASH_BITS);
	/* Work item */
	struct kthread_work transaction_work;
	/* Spinlock used to synchronize access to periodic io data structs */
	spinlock_t periodic_io_lock;
	/* Queue of all periodic_io pending processing */
	struct list_head periodic_io_process_queue;
	/* Periodic IO counter, which also provides periodic io ID */
	int64_t periodic_io_counter;
	/* Structure to store info to help debugging client data */
	struct lwis_client_debug_info debug_info;
	/* Each device has a linked list of clients */
	struct list_head node;
	/* Mark if the client called device enable */
	bool is_enabled;
	/* Work item to schedule managed LWIS Bus transfers */
	struct kthread_work io_bus_work;
	/* Indicates if the client has been issued a flush worker call */
	enum lwis_client_flush_state flush_state;
	/* Lock to guard client's flush state changes */
	spinlock_t flush_lock;
};

/*
 *  lwis_base_probe: Common probe function that will be used for all types
 *  of devices.
 */
int lwis_base_probe(struct lwis_device *lwis_dev);

/*
 *  lwis_base_unprobe: Cleanup a device instance
 */
void lwis_base_unprobe(struct lwis_device *unprobe_lwis_dev);

/*
 * Find LWIS device by id
 */
struct lwis_device *lwis_find_dev_by_id(int dev_id);

/*
 * Check i2c device is still in use:
 * Check if there is any other device using the same I2C bus.
 */
bool lwis_i2c_dev_is_in_use(struct lwis_device *lwis_dev);

/*
 * Power up a LWIS device, should be called when lwis_dev->enabled is 0
 * lwis_dev->interclient_lock should be held before this function.
 */
int lwis_dev_power_up_locked(struct lwis_device *lwis_dev);

/*
 * Power down a LWIS device, should be called when lwis_dev->enabled become 0
 * lwis_dev->interclient_lock should be held before this function.
 */
int lwis_dev_power_down_locked(struct lwis_device *lwis_dev);

/*
 * Process lwis device power sequence list
 */
int lwis_dev_process_power_sequence(struct lwis_device *lwis_dev,
				    struct lwis_device_power_sequence_list *list, bool set_active,
				    bool skip_error);

/*
 *  lwis_dev_power_seq_list_alloc:
 *  Allocate an instance of the lwis_device_power_sequence_info
 *  and initialize the data structures according to the number of
 *  lwis_device_power_sequence_info specified.
 */
struct lwis_device_power_sequence_list *lwis_dev_power_seq_list_alloc(int count);

/*
 *  lwis_dev_power_seq_list_free: Deallocate the
 *  wis_device_power_sequence_info structure.
 */
void lwis_dev_power_seq_list_free(struct lwis_device_power_sequence_list *list);

/*
 *  lwis_dev_power_seq_list_print:
 *  Print lwis_device_power_sequence_list content
 */
void lwis_dev_power_seq_list_print(struct lwis_device_power_sequence_list *list);

/*
 * lwis_device_info_dump:
 * Use the customized function handle to print information from each device registered in LWIS.
 */
void lwis_device_info_dump(const char *name, void (*func)(struct lwis_device *));

/*
 * lwis_save_register_io_info: Saves the register io info in a history buffer
 * for better debugability.
 */
void lwis_save_register_io_info(struct lwis_device *lwis_dev, struct lwis_io_entry *io_entry,
				size_t access_size);

/*
 * lwis_process_worker_queue:
 * Function to process the transaction process queue and
 * periodic io queue on the transaction thread
 */
void lwis_process_worker_queue(struct lwis_client *client);

/*
 * lwis_queue_device_worker:
 * Function to queue periodic or transaction work on the device
 * worker.
 */
void lwis_queue_device_worker(struct lwis_client *client);

/*
 * lwis_flush_device_worker:
 * Function to flush periodic or transaction work from the device
 * worker.
 */
void lwis_flush_device_worker(struct lwis_client *client);

#endif /* LWIS_DEVICE_H_ */
