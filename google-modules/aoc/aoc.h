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

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/timer.h>
#include <soc/google/debug-snapshot.h>
#include "aoc_ipc_core.h"

/* TODO: Remove internal calls, or promote to "public" */
#include "aoc_ipc_core_internal.h"

#include "uapi/aoc.h"

#ifdef __KERNEL__

#define MAX_SENSOR_POWER_NUM 5
#define MAX_DMIC_POWER_NUM 4
#define AP_RESET_REASON_LENGTH 32
#define MAX_FIRMWARE_LENGTH 128

#define AOC_S2MPU_CTRL0 0x0
#define AOC_S2MPU_CTRL_PROTECTION_ENABLE_PER_VID_CLR 0x54
#define AOC_S2MPU_CTRL_PROTECTION_ENABLE_VID_MASK_ALL 0xFF

#define AOC_RESTART_DISABLED_RC (0xD15AB1ED)

#define SENSOR_DIRECT_HEAP_SIZE SZ_4M
#define PLAYBACK_HEAP_SIZE SZ_16K
#define CAPTURE_HEAP_SIZE SZ_64K

#define DT_PROPERTY_NOT_FOUND 0xffffffff

struct aoc_service_dev;
typedef void (*aoc_service_dev_handler)(struct aoc_service_dev *d);

enum AOC_FW_STATE {
	AOC_STATE_OFFLINE,
	AOC_STATE_FIRMWARE_LOADED,
	AOC_STATE_STARTING,
	AOC_STATE_ONLINE,
	AOC_STATE_SSR
};

struct mbox_slot {
	struct mbox_client client;
	struct mbox_chan *channel;
	void *prvdata;
	int index;
};

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

typedef int (*aoc_map_handler)(u32 handle, phys_addr_t p, size_t size,
				bool mapped, void *ctx);

struct aoc_prvdata {
	struct mbox_slot *mbox_channels;
	struct aoc_service_dev **services;

	unsigned long *read_blocked_mask;
	unsigned long *write_blocked_mask;

	struct work_struct online_work;
	struct resource dram_resource;
	aoc_map_handler map_handler;
	void *map_handler_ctx;

	struct delayed_work monitor_work;
	atomic_t aoc_process_active;

	struct device *dev;
	struct iommu_domain *domain;
	void *ipc_base;

	void *sram_virt;
	void *dram_virt;
	void *aoc_req_virt;
	size_t sram_size;
	size_t dram_size;
	size_t aoc_req_size;

	struct dma_heap *sensor_heap;
	struct dma_heap *audio_playback_heap;
	struct dma_heap *audio_capture_heap;
	phys_addr_t sensor_heap_base;
	phys_addr_t audio_playback_heap_base;
	phys_addr_t audio_capture_heap_base;

	int watchdog_irq;
	struct work_struct watchdog_work;
	bool first_fw_load;
	bool aoc_reset_done;
	bool ap_triggered_reset;
	bool force_release_aoc;
	char ap_reset_reason[AP_RESET_REASON_LENGTH];
	wait_queue_head_t aoc_reset_wait_queue;
	unsigned int acpm_async_id;
	int total_services;

	char firmware_name[MAX_FIRMWARE_LENGTH];
	char *firmware_version;

	struct cdev cdev;
	dev_t aoc_devt;
	struct class *_class;
	struct device *_device;

	u32 disable_monitor_mode;
	u32 enable_uart_tx;
	u32 force_voltage_nominal;
	u32 no_ap_resets;
	u32 force_speaker_ultrasonic;
	u32 volte_release_mif;

	u32 total_coredumps;
	u32 total_restarts;
	unsigned int iommu_nonsecure_irq;
	unsigned int iommu_secure_irq;

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	struct notifier_block itmon_nb;
#endif
	struct device *gsa_dev;
	bool protected_by_gsa;

	int sensor_power_count;
	const char *sensor_power_list[MAX_SENSOR_POWER_NUM];
	struct regulator *sensor_regulator[MAX_SENSOR_POWER_NUM];

	int dmic_power_count;
	const char *dmic_power_list[MAX_DMIC_POWER_NUM];
	struct regulator *dmic_regulator[MAX_DMIC_POWER_NUM];

	int reset_hysteresis_trigger_ms;
	u64 last_reset_time_ns;
	int reset_wait_time_index;

	u32 aoc_pcu_base;
	u32 aoc_gpio_base;
	u32 aoc_pcu_db_set_offset;
	u32 aoc_pcu_db_clr_offset;
	u32 aoc_cp_aperture_start_offset;
	u32 aoc_cp_aperture_end_offset;
	u32 aoc_clock_divider;
	u32 aoc_mbox_channels;

	u16 iommu_size;
	struct iommu_entry *iommu;
	bool iommu_configured;
	bool iommu_config_persistent;
};

struct aoc_module_parameters {
	bool *aoc_autoload_firmware;
	bool *aoc_disable_restart;
	bool *aoc_panic_on_req_timeout;
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

void aoc_set_map_handler(struct aoc_service_dev *dev, aoc_map_handler handler,
			 void *ctx);
void aoc_remove_map_handler(struct aoc_service_dev *dev);
void aoc_trigger_watchdog(const char *reason);

extern u32 gs_chipid_get_revision(void);
extern u32 gs_chipid_get_type(void);
extern u32 gs_chipid_get_product_id(void);

bool aoc_release_from_reset(struct aoc_prvdata *prvdata);

void *aoc_sram_translate(u32 offset);

void request_aoc_on(struct aoc_prvdata *p, bool status);
int wait_for_aoc_status(struct aoc_prvdata *p, bool status);

int aoc_watchdog_restart(struct aoc_prvdata *prvdata,
	struct aoc_module_parameters *aoc_module_params);

int platform_specific_probe(struct platform_device *pdev, struct aoc_prvdata *prvdata);

int start_firmware_load(struct device *dev);

void reset_sensor_power(struct aoc_prvdata *prvdata, bool is_init);

void aoc_configure_hardware(struct aoc_prvdata *prvdata);

void trigger_aoc_ramdump(struct aoc_prvdata *prvdata);

bool aoc_create_dma_buf_heaps(struct aoc_prvdata *prvdata);

phys_addr_t aoc_dram_translate_to_aoc(struct aoc_prvdata *p, phys_addr_t addr);

long aoc_unlocked_ioctl_handle_ion_fd(unsigned int cmd, unsigned long arg);

int configure_watchdog_interrupt(struct platform_device *pdev, struct aoc_prvdata *prvdata);

int configure_iommu_interrupts(struct device *dev, struct device_node *iommu_node,
		struct aoc_prvdata *prvdata);

void aoc_configure_ssmt(struct platform_device *pdev);

int aoc_num_services(void);

aoc_service *service_at_index(struct aoc_prvdata *prvdata,
					    unsigned int index);

struct aoc_service_dev *service_dev_at_index(struct aoc_prvdata *prvdata,
							unsigned int index);

bool validate_service(struct aoc_prvdata *prv, int i);

bool aoc_is_valid_dram_address(struct aoc_prvdata *prv, void *addr);

bool aoc_fw_ready(void);

u32 dt_property(struct device_node *node, const char *key);

void configure_crash_interrupts(struct aoc_prvdata *prvdata, bool enable);

void notify_timeout_aoc_status(void);

void trigger_aoc_ssr(bool ap_triggered_reset, char* reset_reason);

#define AOC_SERVICE_NAME_LENGTH 32

/* Rings should have the ring flag set, slots = 1, size = ring size
 * tx/rx stats for rings are measured in bytes, otherwise msg sends
 */
#define AOC_MAX_ENDPOINTS 200
#define AOC_ENDPOINT_NONE 0xffffffff

/* Offset from the beginning of the DRAM region for the firmware to be stored */
#define AOC_CHARDEV_NAME "aoc"

#define AOC_DOWNCALL_DOORBELL 12

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
	kAOCRandSeed = 0x1011,
	kAOCChipRevision = 0x1012,
	kAOCChipType =  0x1013,
	kAOCGnssType =  0x1014,
	kAOCVolteReleaseMif = 0x1015,
	kAOCChipProductId = 0x1016,
	kAOCWifiChip = 0x1017,
};

#define module_aoc_driver(__aoc_driver)                                        \
	module_driver(__aoc_driver, aoc_driver_register, aoc_driver_unregister)

#endif /* __KERNEL__ */
