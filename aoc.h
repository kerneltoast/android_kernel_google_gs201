/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */
/*
 * Google Whitechapel AoC Core Driver
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/sizes.h>
#include "aoc_ipc_core.h"
#include "uapi/aoc.h"

#ifdef __KERNEL__

struct aoc_service_dev;
typedef void (*aoc_service_dev_handler)(struct aoc_service_dev *d);

struct aoc_service_dev {
	struct device dev;
	wait_queue_head_t read_queue;
	wait_queue_head_t write_queue;

	aoc_service *service;
	void *ipc_base;
	aoc_service_dev_handler handler;
	void *prvdata;
	uint64_t suspend_rx_count;

	uint8_t mbox_index;
	uint8_t service_index;

	bool dead;
	bool wake_capable;
};

#define AOC_DEVICE(_d) container_of((_d), struct aoc_service_dev, dev)

phys_addr_t aoc_service_ring_base_phys_addr(struct aoc_service_dev *dev, aoc_direction dir,
					    size_t *out_size);
phys_addr_t aoc_get_heap_base_phys_addr(struct aoc_service_dev *dev, aoc_direction dir,
					    size_t *out_size);
ssize_t aoc_service_read(struct aoc_service_dev *dev, uint8_t *buffer,
			 size_t count, bool block);
ssize_t aoc_service_read_timeout(struct aoc_service_dev *dev, uint8_t *buffer,
				 size_t count, long timeout);
ssize_t aoc_service_write(struct aoc_service_dev *dev, const uint8_t *buffer,
			  size_t count, bool block);
ssize_t aoc_service_write_timeout(struct aoc_service_dev *dev, const uint8_t *buffer,
				  size_t count, long timeout);
int aoc_service_can_read(struct aoc_service_dev *dev);
int aoc_service_can_write(struct aoc_service_dev *dev);
void aoc_service_set_read_blocked(struct aoc_service_dev *dev);
void aoc_service_set_write_blocked(struct aoc_service_dev *dev);
wait_queue_head_t *aoc_service_get_read_queue(struct aoc_service_dev *dev);
wait_queue_head_t *aoc_service_get_write_queue(struct aoc_service_dev *dev);

/*
 * Returns true if data was flushed, false if no data was flushed
 */
bool aoc_service_flush_read_data(struct aoc_service_dev *dev);

bool aoc_online_state(struct aoc_service_dev *dev);

struct aoc_driver {
	struct device_driver drv;

	/* Array of service names to match against.  Last entry must be NULL */
	const char * const *service_names;
	int (*probe)(struct aoc_service_dev *dev);
	int (*remove)(struct aoc_service_dev *dev);
};
#define AOC_DRIVER(_d) container_of((_d), struct aoc_driver, drv)

int aoc_driver_register(struct aoc_driver *driver);
void aoc_driver_unregister(struct aoc_driver *driver);

typedef int (*aoc_map_handler)(u32 handle, phys_addr_t p, size_t size,
				bool mapped, void *ctx);
void aoc_set_map_handler(struct aoc_service_dev *dev, aoc_map_handler handler,
			 void *ctx);
void aoc_remove_map_handler(struct aoc_service_dev *dev);
void aoc_trigger_watchdog(const char *reason);

#define AOC_SERVICE_NAME_LENGTH 32

/* Rings should have the ring flag set, slots = 1, size = ring size
 * tx/rx stats for rings are measured in bytes, otherwise msg sends
 */
#define AOC_MAX_ENDPOINTS 80
#define AOC_ENDPOINT_NONE 0xffffffff

/* Offset from the beginning of the DRAM region for the firmware to be stored */
#define AOC_CHARDEV_NAME "aoc"

#define AOC_DOWNCALL_DOORBELL 12

#define AOC_GPIO_BASE_WC  0xB70000
#define AOC_GPIO_BASE_PRO 0xD70000

#define AOC_PCU_BASE_WC  0xB00000
#define AOC_PCU_BASE_PRO 0xA00000
#define AOC_PCU_DB_SET_OFFSET 0xD004
#define AOC_PCU_DB_CLR_OFFSET 0xD008
#define AOC_PCU_REVISION_OFFSET 0xF000
#define AOC_PCU_RESET_CONTROL_OFFSET 0x0
#define AOC_PCU_RESET_CONTROL_RESET_VALUE 0x0
#define AOC_PCU_WATCHDOG_CONTROL_OFFSET 0x3000
#define AOC_PCU_WATCHDOG_KEY_OFFSET 0x3004
#define AOC_PCU_WATCHDOG_VALUE_OFFSET 0x3008

#define AOC_PCU_WATCHDOG_KEY_UNLOCK 0xA55AA55A
#define AOC_PCU_WATCHDOG_CONTROL_KEY_ENABLED_MASK 0x4

#define AOC_BINARY_DRAM_BASE 0x98000000
#define AOC_BINARY_LOAD_ADDRESS 0x98000000
#define AOC_BINARY_DRAM_OFFSET (AOC_BINARY_LOAD_ADDRESS - AOC_BINARY_DRAM_BASE)

#define AOC_PARAMETER_MAGIC 0x0a0cda7a
enum AOC_FIRMWARE_INFORMATION {
	kAOCBoardID = 0x1001,
	kAOCBoardRevision = 0x1002,
	kAOCSRAMRepaired = 0x1003,
	kAOCASVTableVersion = 0x1004,
	kAOCCarveoutAddress = 0x1005,
	kAOCCarveoutSize = 0x1006,
	kAOCSensorDirectHeapAddress = 0x1007,
	kAOCSensorDirectHeapSize = 0x1008,
	kAOCForceVNOM = 0x1009,
	kAOCDisableMM = 0x100A,
	kAOCEnableUART = 0x100B,
	kAOCPlaybackHeapAddress = 0x100C,
	kAOCPlaybackHeapSize = 0x100D,
	kAOCCaptureHeapAddress = 0x100E,
	kAOCCaptureHeapSize = 0x100F,
	kAOCForceSpeakerUltrasonic = 0x1010,
};

#define module_aoc_driver(__aoc_driver)                                        \
	module_driver(__aoc_driver, aoc_driver_register, aoc_driver_unregister)

#endif /* __KERNEL__ */
