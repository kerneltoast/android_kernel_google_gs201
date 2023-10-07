// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC Core Driver
 *
 * Copyright (c) 2019-2021 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "aoc: " fmt

#include "aoc.h"

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/dma-map-ops.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/glob.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_data/sscoredump.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <soc/google/acpm_ipc_ctrl.h>
#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pmu-if.h>

#include <linux/gsa/gsa_aoc.h>

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
#include <soc/google/exynos-itmon.h>
#endif

#include "ion_physical_heap.h"

#include "aoc_firmware.h"
#include "aoc_ipc_core.h"
#include "aoc_ramdump_regions.h"

/* TODO: Remove internal calls, or promote to "public" */
#include "aoc_ipc_core_internal.h"

/* This should not be required, as we expect only one of the two to be defined */
#if IS_ENABLED(CONFIG_SOC_GS201)
    #undef CONFIG_SOC_GS101
#endif

#if IS_ENABLED(CONFIG_SOC_GS201) && IS_ENABLED(CONFIG_SOC_GS101)
    #error "GS201 and GS101 are mutually exclusive"
#endif

#define MAX_FIRMWARE_LENGTH 128
#define AP_RESET_REASON_LENGTH 32
#define AOC_S2MPU_CTRL0 0x0

#define AOC_MAX_MINOR (1U)
#if IS_ENABLED(CONFIG_SOC_GS101)
	#define AOC_MBOX_CHANNELS 16 /* AP-A32 mbox */
#else
	#define AOC_MBOX_CHANNELS (16 * 3) /* AP-A32, AP-F1 and AP-P6 mbox */
#endif

#define AOC_FWDATA_ENTRIES 10
#define AOC_FWDATA_BOARDID_DFL  0x20202
#define AOC_FWDATA_BOARDREV_DFL 0x10000

#define SENSOR_DIRECT_HEAP_SIZE SZ_4M
#define PLAYBACK_HEAP_SIZE SZ_16K
#define CAPTURE_HEAP_SIZE SZ_64K

#define MAX_RESET_REASON_STRING_LEN 128UL

#define MAX_SENSOR_POWER_NUM 5

#if IS_ENABLED(CONFIG_SOC_GS201)
	#define AOC_PCU_BASE  AOC_PCU_BASE_PRO
	#define AOC_GPIO_BASE AOC_GPIO_BASE_PRO
	#define AOC_CP_APERTURE_START_OFFSET 0x7FDF80
	#define AOC_CP_APERTURE_END_OFFSET   0x7FFFFF
	#define AOC_CLOCK_DIVIDER 1
#elif IS_ENABLED(CONFIG_SOC_GS101)
	#define AOC_PCU_BASE  AOC_PCU_BASE_WC
	#define AOC_GPIO_BASE AOC_GPIO_BASE_WC
	#define AOC_CP_APERTURE_START_OFFSET 0x5FDF80
	#define AOC_CP_APERTURE_END_OFFSET   0x5FFFFF
	#define GPIO_INTERRUPT 93
	#define AOC_CLOCK_DIVIDER 6
#endif

#define MAX_SENSOR_POWER_NUM 5

#define RESET_WAIT_TIMES_NUM 3
#define RESET_WAIT_TIME_MS 3000
#define RESET_WAIT_TIME_INCREMENT_MS  2048

static DEFINE_MUTEX(aoc_service_lock);

enum AOC_FW_STATE {
	AOC_STATE_OFFLINE = 0,
	AOC_STATE_FIRMWARE_LOADED,
	AOC_STATE_STARTING,
	AOC_STATE_ONLINE
};
static enum AOC_FW_STATE aoc_state;

static struct platform_device *aoc_platform_device;

struct mbox_slot {
	struct mbox_client client;
	struct mbox_chan *channel;
	void *prvdata;
	int index;
};

struct aoc_prvdata {
	struct mbox_slot mbox_channels[AOC_MBOX_CHANNELS];
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
	void *aoc_s2mpu_virt;
	size_t sram_size;
	size_t dram_size;
	size_t aoc_req_size;
	u32 aoc_s2mpu_saved_value;

	struct dma_heap *sensor_heap;
	struct dma_heap *audio_playback_heap;
	struct dma_heap *audio_capture_heap;
	phys_addr_t sensor_heap_base;
	phys_addr_t audio_playback_heap_base;
	phys_addr_t audio_capture_heap_base;

	int watchdog_irq;
	struct work_struct watchdog_work;
	bool aoc_reset_done;
	bool ap_triggered_reset;
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

	u32 total_coredumps;
	u32 total_restarts;
	unsigned int sysmmu_nonsecure_irq;
	unsigned int sysmmu_secure_irq;

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	struct notifier_block itmon_nb;
#endif
	struct device *gsa_dev;
	bool protected_by_gsa;

	int sensor_power_count;
	const char *sensor_power_list[MAX_SENSOR_POWER_NUM];
	struct regulator *sensor_regulator[MAX_SENSOR_POWER_NUM];

	int reset_hysteresis_trigger_ms;
	u64 last_reset_time_ns;
	int reset_wait_time_index;
};

struct aoc_prvdata *aoc_prvdata_copy;

/* TODO: Reduce the global variables (move into a driver structure) */
/* Resources found from the device tree */
static struct resource *aoc_sram_resource;

struct sscd_info {
	char *name;
	struct sscd_segment segs[256];
	u16 seg_count;
};

static void trigger_aoc_ramdump(struct aoc_prvdata *prvdata);
static void sscd_release(struct device *dev);

static struct sscd_info sscd_info;
static struct sscd_platform_data sscd_pdata;
static struct platform_device sscd_dev = { .name = "aoc",
					   .driver_override = SSCD_NAME,
					   .id = -1,
					   .dev = {
						   .platform_data = &sscd_pdata,
						   .release = sscd_release,
					   } };

static void *aoc_sram_virt_mapping;
static void *aoc_dram_virt_mapping;

static int aoc_irq;

static struct aoc_control_block *aoc_control;

static int aoc_major;

static const char *default_firmware = "aoc.bin";
static bool aoc_autoload_firmware;
module_param(aoc_autoload_firmware, bool, 0644);
MODULE_PARM_DESC(aoc_autoload_firmware, "Automatically load firmware if true");

static int aoc_core_suspend(struct device *dev);
static int aoc_core_resume(struct device *dev);

const static struct dev_pm_ops aoc_core_pm_ops = {
	.suspend = aoc_core_suspend,
	.resume = aoc_core_resume,
};

static int aoc_bus_match(struct device *dev, struct device_driver *drv);
static int aoc_bus_probe(struct device *dev);
static int aoc_bus_remove(struct device *dev);

static struct bus_type aoc_bus_type = {
	.name = "aoc",
	.match = aoc_bus_match,
	.probe = aoc_bus_probe,
	.remove = aoc_bus_remove,
};

struct aoc_client {
	int client_id;
	int endpoint;
};

static bool aoc_fpga_reset(struct aoc_prvdata *prvdata);
static bool write_reset_trampoline(u32 addr);
static bool aoc_a32_reset(void);
static bool configure_sensor_regulator(struct aoc_prvdata *prvdata, bool enable);
static int aoc_watchdog_restart(struct aoc_prvdata *prvdata);
static void acpm_aoc_reset_callback(unsigned int *cmd, unsigned int size);

static int start_firmware_load(struct device *dev);
static void aoc_take_offline(struct aoc_prvdata *prvdata);
static void signal_aoc(struct mbox_chan *channel);
static void reset_sensor_power(struct aoc_prvdata *prvdata, bool is_init);

static void aoc_process_services(struct aoc_prvdata *prvdata, int offset);

static irqreturn_t watchdog_int_handler(int irq, void *dev);
static void aoc_watchdog(struct work_struct *work);

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
static int aoc_itmon_notifier(struct notifier_block *nb, unsigned long action,
			      void *nb_data)
{
	struct aoc_prvdata *prvdata;
	struct itmon_notifier *itmon_info = nb_data;

	prvdata = container_of(nb, struct aoc_prvdata, itmon_nb);
	if (itmon_info->port && (strncmp(itmon_info->port, "AOC", sizeof("AOC") - 1) == 0))
		return NOTIFY_STOP;

	if (itmon_info->target_addr == 0) {
		dev_err(prvdata->dev,
			"Possible repro of b/174577569, please upload a bugreport and /data/vendor/ssrdump to that bug\n");
		return NOTIFY_STOP;
	}

	if ((itmon_info->target_addr >= aoc_sram_resource->start + AOC_CP_APERTURE_START_OFFSET) &&
	    (itmon_info->target_addr <= aoc_sram_resource->start + AOC_CP_APERTURE_END_OFFSET)) {
		dev_err(prvdata->dev,
			"Valid memory access triggered ITMON error. Please file a bug with bugreport and contents of /data/vendor/ssrdump\n");
		return NOTIFY_STOP;
	}

	return NOTIFY_OK;
}
#endif

static inline void *aoc_sram_translate(u32 offset)
{
	BUG_ON(aoc_sram_virt_mapping == NULL);
	if (offset > resource_size(aoc_sram_resource))
		return NULL;

	return aoc_sram_virt_mapping + offset;
}

static inline void *aoc_dram_translate(struct aoc_prvdata *p, u32 offset)
{
	BUG_ON(p->dram_virt == NULL);
	if (offset > p->dram_size)
		return NULL;

	return p->dram_virt + offset;
}

static bool aoc_is_valid_dram_address(struct aoc_prvdata *prv, void *addr)
{
	ptrdiff_t offset;

	if (addr < prv->dram_virt)
		return false;

	offset = addr - prv->dram_virt;
	return (offset < prv->dram_size);
}

static inline phys_addr_t aoc_dram_translate_to_aoc(struct aoc_prvdata *p,
					    phys_addr_t addr)
{
	phys_addr_t phys_start = p->dram_resource.start;
	phys_addr_t phys_end = phys_start + resource_size(&p->dram_resource);
	u32 offset;

	if (addr < phys_start || addr >= phys_end)
		return 0;

	offset = addr - phys_start;
	return AOC_BINARY_DRAM_BASE + offset;
}

static inline bool aoc_fw_ready(void)
{
	return aoc_control != NULL && aoc_control->magic == AOC_MAGIC;
}

static inline int aoc_num_services(void)
{
	return aoc_fw_ready() ? le32_to_cpu(aoc_control->services) : 0;
}

static inline aoc_service *service_at_index(struct aoc_prvdata *prvdata,
					    unsigned index)
{
	if (!aoc_fw_ready() || index > aoc_num_services())
		return NULL;

	return (((uint8_t *)prvdata->ipc_base) + aoc_control->services_offset +
		(le32_to_cpu(aoc_control->service_size) * index));
}

static inline struct aoc_service_dev *service_dev_at_index(struct aoc_prvdata *prvdata, unsigned index)
{
	if (!aoc_fw_ready() || index > aoc_num_services() || aoc_state != AOC_STATE_ONLINE)
		return NULL;

	return prvdata->services[index];
}

static bool validate_service(struct aoc_prvdata *prv, int i)
{
	struct aoc_ipc_service_header *hdr = service_at_index(prv, i);
	struct device *dev = prv->dev;

	if (!aoc_is_valid_dram_address(prv, hdr)) {
		dev_err(dev, "service %d is not in DRAM region\n", i);
		return false;
	}

	if (hdr->regions[0].slots == 0 && hdr->regions[1].slots == 0) {
		dev_err(dev, "service %d is not readable or writable\n", i);

		return false;
	}

	if (aoc_service_is_ring(hdr) &&
	    (hdr->regions[0].slots > 1 || hdr->regions[1].slots > 1)) {
		dev_err(dev, "service %d has invalid ring slot configuration\n",
			i);

		return false;
	}

	return true;
}

static int driver_matches_service_by_name(struct device_driver *drv, void *name)
{
	struct aoc_driver *aoc_drv = AOC_DRIVER(drv);
	const char *service_name = name;
	const char *const *driver_names = aoc_drv->service_names;

	while (driver_names && *driver_names) {
		if (glob_match(*driver_names, service_name) == true)
			return 1;

		driver_names++;
	}

	return 0;
}

static bool has_name_matching_driver(const char *service_name)
{
	return bus_for_each_drv(&aoc_bus_type, NULL, (char *)service_name,
				driver_matches_service_by_name) != 0;
}

static bool service_names_are_valid(struct aoc_prvdata *prv)
{
	int services, i, j;
	const char *name;

	services = aoc_num_services();
	if (services == 0)
		return false;

	/* All names have a valid length */
	for (i = 0; i < services; i++) {
		size_t name_len;
		name = aoc_service_name(service_at_index(prv, i));

		if (!name) {
			dev_err(prv->dev,
				"failed to retrieve service name for service %d\n",
				i);
			return false;
		}

		name_len = strnlen(name, AOC_SERVICE_NAME_LENGTH);
		if (name_len == 0 || name_len == AOC_SERVICE_NAME_LENGTH) {
			dev_err(prv->dev,
				"service %d has a name that is too long\n", i);
			return false;
		}

		dev_dbg(prv->dev, "validated service %d name %s\n", i, name);
	}

	/* No duplicate names */
	for (i = 0; i < services; i++) {
		char name1[AOC_SERVICE_NAME_LENGTH],
			name2[AOC_SERVICE_NAME_LENGTH];
		name = aoc_service_name(service_at_index(prv, i));
		if (!name) {
			dev_err(prv->dev,
				"failed to retrieve service name for service %d\n",
				i);
			return false;
		}

		memcpy_fromio(name1, name, sizeof(name1));

		for (j = i + 1; j < services; j++) {
			name = aoc_service_name(service_at_index(prv, j));
			if (!name) {
				dev_err(prv->dev,
					"failed to retrieve service name for service %d\n",
					j);
				return false;
			}
			memcpy_fromio(name2, name, sizeof(name2));

			if (strncmp(name1, name2, AOC_SERVICE_NAME_LENGTH) ==
			    0) {
				dev_err(prv->dev,
					"service %d and service %d have the same name\n",
					i, j);
				return false;
			}
		}
	}

	return true;
}

static void free_mailbox_channels(struct aoc_prvdata *prv);

static int allocate_mailbox_channels(struct aoc_prvdata *prv)
{
	struct device *dev = prv->dev;
	struct mbox_slot *slot;
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(prv->mbox_channels); i++) {
		slot = &prv->mbox_channels[i];
		slot->channel = mbox_request_channel(&slot->client, i);
		if (IS_ERR(slot->channel)) {
			dev_err(dev, "failed to find mailbox interface %d : %ld\n", i,
				PTR_ERR(slot->channel));
			slot->channel = NULL;
			rc = -EIO;
			goto err_mbox_req;
		}
	}

err_mbox_req:
	if (rc != 0)
		free_mailbox_channels(prv);

	return rc;
}

static void free_mailbox_channels(struct aoc_prvdata *prv)
{
	struct mbox_slot *slot;
	int i;

	for (i = 0; i < ARRAY_SIZE(prv->mbox_channels); i++) {
		slot = &prv->mbox_channels[i];
		if (slot->channel) {
			mbox_free_channel(slot->channel);
			slot->channel = NULL;
		}
	}
}

static void aoc_mbox_rx_callback(struct mbox_client *cl, void *mssg)
{
	struct mbox_slot *slot = container_of(cl, struct mbox_slot, client);
	struct aoc_prvdata *prvdata = slot->prvdata;

	switch (aoc_state) {
	case AOC_STATE_FIRMWARE_LOADED:
		if (aoc_fw_ready()) {
			aoc_state = AOC_STATE_STARTING;
			schedule_work(&prvdata->online_work);
		}
		break;
	case AOC_STATE_ONLINE:
		aoc_process_services(prvdata, slot->index);
		break;
	default:
		break;
	}
}

static void aoc_mbox_tx_prepare(struct mbox_client *cl, void *mssg)
{
}

static void aoc_mbox_tx_done(struct mbox_client *cl, void *mssg, int r)
{
}

static void aoc_req_assert(struct aoc_prvdata *p, bool assert)
{
	iowrite32(!!assert, p->aoc_req_virt);
}

static int aoc_req_wait(struct aoc_prvdata *p, bool assert)
{
	unsigned long aoc_req_timeout;

	aoc_req_timeout = jiffies + (2 * HZ);
	while (time_before(jiffies, aoc_req_timeout)) {
		if (!!readl(p->aoc_req_virt + 0x40) == !!assert)
			return 0;
		msleep(100);
	}

	return -ETIMEDOUT;
}

extern int gs_chipid_get_ap_hw_tune_array(const u8 **array);

#if IS_ENABLED(CONFIG_SOC_GS101)
static bool aoc_sram_was_repaired(struct aoc_prvdata *prvdata)
{
	const u8 *array;
	struct device *dev = prvdata->dev;
	int ret;

	ret = gs_chipid_get_ap_hw_tune_array(&array);

	if (ret == -EPROBE_DEFER) {
		dev_err(dev, "Unable to determine SRAM repair state.  Leaving monitor mode disabled\n");
		return false;
	}

	if (ret != 32) {
		dev_err(dev, "Unexpected hw_tune_array size.  Leaving monitor mode disabled\n");
		return false;
	}

	/* Bit 65 says that AoC SRAM was repaired */
	return ((array[8] & 0x2) != 0);
}
#else
static inline bool aoc_sram_was_repaired(struct aoc_prvdata *prvdata) { return false; }
#endif

struct aoc_fw_data {
	u32 key;
	u32 value;
};

static u32 dt_property(struct device_node *node, const char *key)
{
	u32 ret;

	if (of_property_read_u32(node, key, &ret))
		return 0xffffffff;

	return ret;
}

static void aoc_pass_fw_information(void *base, const struct aoc_fw_data *fwd,
				    size_t num)
{
	u32 *data = base;
	int i;

	writel_relaxed(AOC_PARAMETER_MAGIC, data++);
	writel_relaxed(num, data++);
	writel_relaxed(12 + (num * (3 * sizeof(u32))), data++);

	for (i = 0; i < num; i++) {
		writel_relaxed(fwd[i].key, data++);
		writel_relaxed(sizeof(u32), data++);
		writel_relaxed(fwd[i].value, data++);
	}
}

static u32 aoc_board_config_parse(struct device_node *node, u32 *board_id, u32 *board_rev)
{
	const char *board_cfg;
	int err = 0;

	/* Read board config from device tree */
	err = of_property_read_string(node, "aoc-board-cfg", &board_cfg);
	if (err < 0) {
		pr_err("Unable to retrieve AoC board configuration, check DT");
		pr_info("Assuming R4/O6 board configuration");
		*board_id  = AOC_FWDATA_BOARDID_DFL;
		*board_rev = AOC_FWDATA_BOARDREV_DFL;
	  return err;
	}

	/* Read board id from device tree */
	err = of_property_read_u32(node, "aoc-board-id", board_id);
	if (err < 0) {
		pr_err("Unable to retrieve AoC board id, check DT");
		pr_info("Assuming R4/O6 board configuration");
		*board_id  = AOC_FWDATA_BOARDID_DFL;
		*board_rev = AOC_FWDATA_BOARDREV_DFL;
		return err;
	}

	/* Read board revision from device tree */
	err = of_property_read_u32(node, "aoc-board-rev", board_rev);
	if (err < 0) {
		pr_err("Unable to retrieve AoC board revision, check DT");
		pr_info("Assuming R4/O6 board configuration");
		*board_id  = AOC_FWDATA_BOARDID_DFL;
		*board_rev = AOC_FWDATA_BOARDREV_DFL;
		return err;
	}

	pr_info("AoC Platform: %s", board_cfg);

	return err;
}

static int aoc_fw_authenticate(struct aoc_prvdata *prvdata,
			       const struct firmware *fw) {

	int rc;
	dma_addr_t header_dma_addr;
	void *header_vaddr;

	/* Allocate coherent memory for the image header */
	header_vaddr = dma_alloc_coherent(prvdata->gsa_dev, AOC_AUTH_HEADER_SIZE,
					  &header_dma_addr, GFP_KERNEL);
	if (!header_vaddr) {
		dev_err(prvdata->dev, "Failed to allocate coherent memory for header\n");
		rc = -ENOMEM;
		goto err_alloc;
	}

	memcpy(header_vaddr, fw->data, AOC_AUTH_HEADER_SIZE);

	rc = gsa_load_aoc_fw_image(prvdata->gsa_dev, header_dma_addr,
				   prvdata->dram_resource.start + AOC_BINARY_DRAM_OFFSET);
	if (rc) {
		dev_err(prvdata->dev, "GSA authentication failed: %d\n", rc);
		goto err_auth;
	}

err_auth:
err_alloc:
	dma_free_coherent(prvdata->gsa_dev, AOC_AUTH_HEADER_SIZE, header_vaddr, header_dma_addr);
	return rc;
}

static void aoc_fw_callback(const struct firmware *fw, void *ctx)
{
	struct device *dev = ctx;
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	u32 sram_was_repaired = aoc_sram_was_repaired(prvdata);
	u32 carveout_base = prvdata->dram_resource.start;
	u32 carveout_size = prvdata->dram_size;
	u32 dt_force_vnom = dt_property(prvdata->dev->of_node, "force-vnom");
	u32 force_vnom = ((dt_force_vnom != 0) || (prvdata->force_voltage_nominal != 0)) ? 1 : 0;
	u32 disable_mm = prvdata->disable_monitor_mode;
	u32 enable_uart = prvdata->enable_uart_tx;
	u32 force_speaker_ultrasonic = prvdata->force_speaker_ultrasonic;
	u32 board_id  = AOC_FWDATA_BOARDID_DFL;
	u32 board_rev = AOC_FWDATA_BOARDREV_DFL;
	phys_addr_t sensor_heap = aoc_dram_translate_to_aoc(prvdata, prvdata->sensor_heap_base);
	phys_addr_t playback_heap = aoc_dram_translate_to_aoc(prvdata, prvdata->audio_playback_heap_base);
	phys_addr_t capture_heap = aoc_dram_translate_to_aoc(prvdata, prvdata->audio_capture_heap_base);
	unsigned int i;
	bool fw_signed;

	struct aoc_fw_data fw_data[] = {
		{ .key = kAOCBoardID, .value = board_id },
		{ .key = kAOCBoardRevision, .value = board_rev },
		{ .key = kAOCSRAMRepaired, .value = sram_was_repaired },
		{ .key = kAOCCarveoutAddress, .value = carveout_base},
		{ .key = kAOCCarveoutSize, .value = carveout_size},
		{ .key = kAOCSensorDirectHeapAddress, .value = sensor_heap},
		{ .key = kAOCSensorDirectHeapSize, .value = SENSOR_DIRECT_HEAP_SIZE },
		{ .key = kAOCPlaybackHeapAddress, .value = playback_heap},
		{ .key = kAOCPlaybackHeapSize, .value = PLAYBACK_HEAP_SIZE },
		{ .key = kAOCCaptureHeapAddress, .value = capture_heap},
		{ .key = kAOCCaptureHeapSize, .value = CAPTURE_HEAP_SIZE },
		{ .key = kAOCForceVNOM, .value = force_vnom },
		{ .key = kAOCDisableMM, .value = disable_mm },
		{ .key = kAOCEnableUART, .value = enable_uart },
		{ .key = kAOCForceSpeakerUltrasonic, .value = force_speaker_ultrasonic }
	};
	const char *version;
	u32 fw_data_entries = ARRAY_SIZE(fw_data);
	u32 ipc_offset, bootloader_offset;

	aoc_board_config_parse(prvdata->dev->of_node, &board_id, &board_rev);

	if (!fw) {
		dev_err(dev, "failed to load firmware image\n");
		return;
	}

	for (i = 0; i < fw_data_entries; i++) {
		if (fw_data[i].key == kAOCBoardID)
			fw_data[i].value = board_id;
		else if (fw_data[i].key == kAOCBoardRevision)
			fw_data[i].value = board_rev;
	}

	aoc_req_assert(prvdata, true);

	if (!fw->data) {
		dev_err(dev, "firmware image contains no data\n");
		goto free_fw;
	}

	if (!_aoc_fw_is_valid(fw)) {
		dev_err(dev, "firmware validation failed\n");
		goto free_fw;
	}

	ipc_offset = _aoc_fw_ipc_offset(fw);
	bootloader_offset = _aoc_fw_bootloader_offset(fw);
	version = _aoc_fw_version(fw);

	prvdata->firmware_version = devm_kasprintf(dev, GFP_KERNEL, "%s", version);

	pr_notice("successfully loaded firmware version %s type %s",
		  version ? version : "unknown",
		  _aoc_fw_is_release(fw) ? "release" : "development");

	if (prvdata->disable_monitor_mode)
		dev_err(dev, "Monitor Mode will be disabled.  Power will be impacted\n");

	if (prvdata->enable_uart_tx)
		dev_err(dev, "Enabling logging on UART. This will affect system timing\n");

	if (prvdata->force_voltage_nominal)
		dev_err(dev, "Forcing VDD_AOC to VNOM on this device. Power will be impacted\n");
	else
		dev_info(dev, "AoC using default DVFS on this device.\n");

	if (prvdata->no_ap_resets)
		dev_err(dev, "Resets by AP via sysfs are disabled\n");

	if (prvdata->force_speaker_ultrasonic)
		dev_err(dev, "Forcefully enabling Speaker Ultrasonic pipeline\n");

	if (!_aoc_fw_is_compatible(fw)) {
		dev_err(dev, "firmware and drivers are incompatible\n");
		goto free_fw;
	}

	fw_signed = _aoc_fw_is_signed(fw);
	prvdata->protected_by_gsa = fw_signed;

	dev_info(dev, "Loading %s aoc image\n", fw_signed ? "signed" : "unsigned");

	aoc_control = aoc_dram_translate(prvdata, ipc_offset);

	aoc_fpga_reset(prvdata);

	_aoc_fw_commit(fw, aoc_dram_virt_mapping + AOC_BINARY_DRAM_OFFSET);

	if (fw_signed) {
		int rc = aoc_fw_authenticate(prvdata, fw);
		if (rc) {
			dev_err(dev, "GSA: FW authentication failed: %d\n", rc);
			goto free_fw;
		}
	} else {
		write_reset_trampoline(AOC_BINARY_LOAD_ADDRESS + bootloader_offset);
	}

	aoc_pass_fw_information(aoc_dram_translate(prvdata, ipc_offset),
			fw_data, ARRAY_SIZE(fw_data));

	aoc_state = AOC_STATE_FIRMWARE_LOADED;

	dev_info(dev, "disabling SICD for 2 sec for aoc boot\n");
	disable_power_mode(0, POWERMODE_TYPE_SYSTEM);
	prvdata->ipc_base = aoc_dram_translate(prvdata, ipc_offset);

	/* start AOC */
	if (fw_signed) {
		int rc = gsa_send_aoc_cmd(prvdata->gsa_dev, GSA_AOC_START);
		if (rc < 0) {
			dev_err(dev, "GSA: Failed to start AOC: %d\n", rc);
			goto free_fw;
		}
	} else {
		aoc_a32_reset();
	}

	enable_irq(prvdata->watchdog_irq);

	/* Monitor if there is callback from aoc after 5sec */
	cancel_delayed_work_sync(&prvdata->monitor_work);
	schedule_delayed_work(&prvdata->monitor_work,
			msecs_to_jiffies(5 * 1000));

	msleep(2000);
	dev_info(dev, "re-enabling SICD\n");
	enable_power_mode(0, POWERMODE_TYPE_SYSTEM);

free_fw:
	release_firmware(fw);
}

phys_addr_t aoc_service_ring_base_phys_addr(struct aoc_service_dev *dev, aoc_direction dir,
					    size_t *out_size)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;
	void *ring_base;

	if (!dev)
		return -EINVAL;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);

	service = service_at_index(prvdata, dev->service_index);

	ring_base = aoc_service_ring_base(service, prvdata->ipc_base, dir);

	pr_debug("aoc DRAM starts at (virt): %pK, (phys):%llx, ring base (virt): %pK",
		 aoc_dram_virt_mapping, prvdata->dram_resource.start, ring_base);

	if (out_size)
		*out_size = aoc_service_ring_size(service, dir);

	return ring_base - aoc_dram_virt_mapping + prvdata->dram_resource.start;
}
EXPORT_SYMBOL_GPL(aoc_service_ring_base_phys_addr);

phys_addr_t aoc_get_heap_base_phys_addr(struct aoc_service_dev *dev, aoc_direction dir,
					    size_t *out_size)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;
	phys_addr_t audio_heap_base;

	if (!dev)
		return -EINVAL;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);

	service = service_at_index(prvdata, dev->service_index);

	if (out_size)
		*out_size = aoc_service_ring_size(service, dir);

	if (dir == AOC_DOWN)
		audio_heap_base = prvdata->audio_playback_heap_base;
	else
		audio_heap_base = prvdata->audio_capture_heap_base;

	pr_debug("Get heap address(phy):%llx\n", audio_heap_base);

	return audio_heap_base;
}
EXPORT_SYMBOL_GPL(aoc_get_heap_base_phys_addr);

bool aoc_service_flush_read_data(struct aoc_service_dev *dev)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;
	size_t slots;

	if (!dev)
		return false;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);

	service = service_at_index(prvdata, dev->service_index);

	slots = aoc_service_slots_available_to_read(service, AOC_UP);
	if (slots == 0)
		return false;

	aoc_service_advance_read_index(service, AOC_UP, slots);
	return true;
}
EXPORT_SYMBOL_GPL(aoc_service_flush_read_data);

ssize_t aoc_service_read(struct aoc_service_dev *dev, uint8_t *buffer,
			 size_t count, bool block)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;

	size_t msg_size;
	int service_number;
	int ret = 0;
	bool was_full;
	int interrupt = dev->mbox_index;

	if (!dev || !buffer || !count)
		return -EINVAL;

	if (dev->dead)
		return -ENODEV;

	if (aoc_state != AOC_STATE_ONLINE)
		return -EBUSY;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);

	service_number = dev->service_index;
	service = service_at_index(prvdata, dev->service_index);

	BUG_ON(!aoc_is_valid_dram_address(prvdata, service));

	if (aoc_service_message_slots(service, AOC_UP) == 0)
		return -EBADF;

	if (!aoc_service_can_read_message(service, AOC_UP)) {
		if (!block)
			return -EAGAIN;

		set_bit(service_number, prvdata->read_blocked_mask);
		ret = wait_event_interruptible(dev->read_queue,
			aoc_state != AOC_STATE_ONLINE || dev->dead ||
				aoc_service_can_read_message(service, AOC_UP));
		clear_bit(service_number, prvdata->read_blocked_mask);
	}

	if (dev->dead)
		return -ENODEV;

	if (aoc_state != AOC_STATE_ONLINE)
		return -ENODEV;

	/*
	 * The wait can fail if the AoC goes offline in the middle of a
	 * blocking read, so check again after the wait
	 */
	if (ret != 0)
		return -EAGAIN;

	if (!aoc_service_is_ring(service) &&
	    count < aoc_service_current_message_size(service, prvdata->ipc_base,
						     AOC_UP))
		return -EFBIG;

	msg_size = count;
	was_full = !aoc_service_can_write_message(service, AOC_UP);

	aoc_service_read_message(service, prvdata->ipc_base, AOC_UP, buffer,
				 &msg_size);

	/*
	 * If the service queue was full right before reading, signal AoC that
	 * there is now space available to write.
	 */
	if (was_full)
		signal_aoc(prvdata->mbox_channels[interrupt].channel);

	return msg_size;
}
EXPORT_SYMBOL_GPL(aoc_service_read);


bool aoc_online_state(struct aoc_service_dev *dev) {
	struct aoc_prvdata *prvdata;
	if (!dev)
		return false;

	prvdata = dev_get_drvdata(dev->dev.parent);
	if (!prvdata)
		return false;

	if (aoc_state != AOC_STATE_ONLINE || work_busy(&prvdata->watchdog_work))
		return false;
	return true;
}
EXPORT_SYMBOL_GPL(aoc_online_state);

ssize_t aoc_service_read_timeout(struct aoc_service_dev *dev, uint8_t *buffer,
				 size_t count, long timeout)
{
	struct aoc_prvdata *prvdata;
	aoc_service *service;

	size_t msg_size;
	int service_number;
	long ret = 1;

	if (!dev || !buffer || !count)
		return -EINVAL;

	if (dev->dead)
		return -ENODEV;

	prvdata = dev_get_drvdata(dev->dev.parent);
	if (!prvdata)
		return -ENODEV;

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE || work_busy(&prvdata->watchdog_work)) {
		ret = -EBUSY;
		goto err;
	}

	service_number = dev->service_index;
	service = service_at_index(prvdata, dev->service_index);

	if (!aoc_is_valid_dram_address(prvdata, service)) {
		WARN_ONCE(1, "aoc service %d has invalid DRAM region", service_number);
		ret = -ENODEV;
		goto err;
	}

	if (aoc_service_message_slots(service, AOC_UP) == 0) {
		ret = -EBADF;
		goto err;
	}

	if (!aoc_service_can_read_message(service, AOC_UP)) {
		set_bit(service_number, prvdata->read_blocked_mask);
		ret = wait_event_interruptible_timeout(
			dev->read_queue,
			aoc_state != AOC_STATE_ONLINE || dev->dead ||
				aoc_service_can_read_message(service, AOC_UP),
			timeout);
		clear_bit(service_number, prvdata->read_blocked_mask);
	}

	if (dev->dead || (aoc_state != AOC_STATE_ONLINE)) {
		ret = -ENODEV;
		goto err;
	}

	if (ret < 0)
		goto err;

	/* AoC timed out */
	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto err;
	}

	if (!aoc_service_is_ring(service) &&
	    count < aoc_service_current_message_size(service, prvdata->ipc_base,
						     AOC_UP)) {
		ret = -EFBIG;
		goto err;
	}

	msg_size = count;
	aoc_service_read_message(service, prvdata->ipc_base, AOC_UP, buffer,
				 &msg_size);

err:
	atomic_dec(&prvdata->aoc_process_active);

	if (ret < 0)
		return ret;

	return msg_size;
}
EXPORT_SYMBOL_GPL(aoc_service_read_timeout);

ssize_t aoc_service_write(struct aoc_service_dev *dev, const uint8_t *buffer,
			  size_t count, bool block)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;

	aoc_service *service;
	int service_number;
	int interrupt = dev->mbox_index;
	int ret = 0;

	if (!dev || !buffer || !count)
		return -EINVAL;

	if (dev->dead)
		return -ENODEV;

	if (aoc_state != AOC_STATE_ONLINE)
		return -ENODEV;

	if (interrupt >= AOC_MBOX_CHANNELS)
		return -EINVAL;

	BUG_ON(!dev->dev.parent);

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);

	service_number = dev->service_index;
	service = service_at_index(prvdata, service_number);

	BUG_ON(!aoc_is_valid_dram_address(prvdata, service));

	if (aoc_service_message_slots(service, AOC_DOWN) == 0)
		return -EBADF;

	if (count > aoc_service_message_size(service, AOC_DOWN))
		return -EFBIG;

	if (!aoc_service_can_write_message(service, AOC_DOWN)) {
		if (!block)
			return -EAGAIN;

		set_bit(service_number, prvdata->write_blocked_mask);
		ret = wait_event_interruptible(dev->write_queue,
			aoc_state != AOC_STATE_ONLINE || dev->dead ||
				aoc_service_can_write_message(service, AOC_DOWN));
		clear_bit(service_number, prvdata->write_blocked_mask);
	}

	if (dev->dead)
		return -ENODEV;

	if (aoc_state != AOC_STATE_ONLINE)
		return -ENODEV;

	/*
	 * The wait can fail if the AoC goes offline in the middle of a
	 * blocking write, so check again after the wait
	 */
	if (ret != 0)
		return -EAGAIN;

	ret = aoc_service_write_message(service, prvdata->ipc_base, AOC_DOWN,
					buffer, count);

	if (!aoc_service_is_ring(service) || aoc_ring_is_push(service))
		signal_aoc(prvdata->mbox_channels[interrupt].channel);

	return count;
}
EXPORT_SYMBOL_GPL(aoc_service_write);

ssize_t aoc_service_write_timeout(struct aoc_service_dev *dev, const uint8_t *buffer,
				  size_t count, long timeout)
{
	struct aoc_prvdata *prvdata;

	aoc_service *service;
	int service_number;
	int interrupt = dev->mbox_index;
	long ret = 1;

	if (!dev || !buffer || !count)
		return -EINVAL;

	if (dev->dead)
		return -ENODEV;

	prvdata = dev_get_drvdata(dev->dev.parent);
	if (!prvdata)
		return -ENODEV;

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE || work_busy(&prvdata->watchdog_work)) {
		ret = -EBUSY;
		goto err;
	}

	service_number = dev->service_index;
	service = service_at_index(prvdata, service_number);

	if (!aoc_is_valid_dram_address(prvdata, service)) {
		WARN_ONCE(1, "aoc service %d has invalid DRAM region", service_number);
		ret = -ENODEV;
		goto err;
	}

	if (aoc_service_message_slots(service, AOC_DOWN) == 0) {
		ret = -EBADF;
		goto err;
	}

	if (count > aoc_service_message_size(service, AOC_DOWN)) {
		ret = -EFBIG;
		goto err;
	}

	if (!aoc_service_can_write_message(service, AOC_DOWN)) {
		set_bit(service_number, prvdata->write_blocked_mask);
		ret = wait_event_interruptible_timeout(
			dev->write_queue,
			aoc_state != AOC_STATE_ONLINE || dev->dead ||
				aoc_service_can_write_message(service, AOC_DOWN),
			timeout);
		clear_bit(service_number, prvdata->write_blocked_mask);
	}

	if (dev->dead || (aoc_state != AOC_STATE_ONLINE)) {
		ret = -ENODEV;
		goto err;
	}

	if (ret < 0)
		goto err;

	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto err;
	}

	ret = aoc_service_write_message(service, prvdata->ipc_base, AOC_DOWN,
					buffer, count);

	if (!aoc_service_is_ring(service) || aoc_ring_is_push(service))
		signal_aoc(prvdata->mbox_channels[interrupt].channel);

err:
	atomic_dec(&prvdata->aoc_process_active);

	if (ret < 0)
		return ret;

	return count;
}
EXPORT_SYMBOL_GPL(aoc_service_write_timeout);

int aoc_service_can_read(struct aoc_service_dev *dev)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);
	service = service_at_index(prvdata, dev->service_index);

	if (aoc_service_message_slots(service, AOC_UP) == 0)
		return 0;

	return aoc_service_can_read_message(service, AOC_UP);
}
EXPORT_SYMBOL_GPL(aoc_service_can_read);

int aoc_service_can_write(struct aoc_service_dev *dev)
{
	const struct device *parent;
	struct aoc_prvdata *prvdata;
	aoc_service *service;

	parent = dev->dev.parent;
	prvdata = dev_get_drvdata(parent);
	service = service_at_index(prvdata, dev->service_index);

	if (aoc_service_message_slots(service, AOC_DOWN) == 0)
		return 0;

	return aoc_service_can_write_message(service, AOC_DOWN);
}
EXPORT_SYMBOL_GPL(aoc_service_can_write);

void aoc_service_set_read_blocked(struct aoc_service_dev *dev)
{
	int service_number;
	struct device *parent = dev->dev.parent;
	struct aoc_prvdata *prvdata = dev_get_drvdata(parent);

	service_number = dev->service_index;
	set_bit(service_number, prvdata->read_blocked_mask);
}
EXPORT_SYMBOL_GPL(aoc_service_set_read_blocked);

void aoc_service_set_write_blocked(struct aoc_service_dev *dev)
{
	int service_number;
	struct device *parent = dev->dev.parent;
	struct aoc_prvdata *prvdata = dev_get_drvdata(parent);

	service_number = dev->service_index;
	set_bit(service_number, prvdata->write_blocked_mask);
}
EXPORT_SYMBOL_GPL(aoc_service_set_write_blocked);

wait_queue_head_t *aoc_service_get_read_queue(struct aoc_service_dev *dev)
{
	return &dev->read_queue;
}
EXPORT_SYMBOL_GPL(aoc_service_get_read_queue);

wait_queue_head_t *aoc_service_get_write_queue(struct aoc_service_dev *dev)
{
	return &dev->write_queue;
}
EXPORT_SYMBOL_GPL(aoc_service_get_write_queue);

static bool write_reset_trampoline(u32 addr)
{
	u32 *reset;
	u32 instructions[] = {
        /* <start>: */
        /*  0: */  0xe59f004c,  /* ldr     r0, [pc, #76]   ; 54 <.PCU_SLC_MIF_REQ_ADDR> */
        /*  4: */  0xe59f104c,  /* ldr     r1, [pc, #76]   ; 58 <.PCU_SLC_MIF_REQ_VALUE> */
        /*  8: */  0xe5801000,  /* str     r1, [r0] */
        /*  c: */  0xe59f0048,  /* ldr     r0, [pc, #72]   ; 5c <.PCU_SLC_MIF_ACK_ADDR> */
        /* 10: */  0xe59f104c,  /* ldr     r1, [pc, #76]   ; 64 <.PCU_SLC_MIF_ACK_VALUE> */
        /* 14: */  0xe59f2044,  /* ldr     r2, [pc, #68]   ; 60 <.PCU_SLC_MIF_ACK_MASK> */

        /* <mif_ack_loop>: */
        /* 18: */  0xe5903000,  /* ldr     r3, [r0] */
        /* 1c: */  0xe0033002,  /* and     r3, r3, r2 */
        /* 20: */  0xe1530001,  /* cmp     r3, r1 */
        /* 24: */  0x1afffffb,  /* bne     18 <mif_ack_loop> */

        /* 28: */  0xe59f0038,  /* ldr     r0, [pc, #56]   ; 68 <.PCU_BLK_PWR_REQ_ADDR> */
        /* 2c: */  0xe59f1038,  /* ldr     r1, [pc, #56]   ; 6c <.PCU_BLK_PWR_REQ_VALUE> */
        /* 30: */  0xe5801000,  /* str     r1, [r0] */
        /* 34: */  0xe59f0034,  /* ldr     r0, [pc, #52]   ; 70 <.PCU_BLK_PWR_ACK_ADDR> */
        /* 38: */  0xe59f1038,  /* ldr     r1, [pc, #56]   ; 78 <.PCU_BLK_PWR_ACK_VALUE> */
        /* 3c: */  0xe59f2030,  /* ldr     r2, [pc, #48]   ; 74 <.PCU_BLK_PWR_ACK_MASK> */

        /* <blk_aoc_on_loop>: */
        /* 40: */  0xe5903000,  /* ldr     r3, [r0] */
        /* 44: */  0xe0033002,  /* and     r3, r3, r2 */
        /* 48: */  0xe1530001,  /* cmp     r3, r1 */
        /* 4c: */  0x1afffffb,  /* bne     40 <blk_aoc_on_loop> */
        /* 50: */  0xe59ff024,  /* ldr     pc, [pc, #36]   ; 7c <.BOOTLOADER_START_ADDR> */


        #if IS_ENABLED(CONFIG_SOC_GS201)
          /* .PCU_SLC_MIF_REQ_ADDR:  */  0xA08000,
          /* .PCU_SLC_MIF_REQ_VALUE: */  0x000003,  /* Set ACTIVE_REQUEST = 1, MIS_SLCn = 1 to request MIF access */
          /* .PCU_SLC_MIF_ACK_ADDR:  */  0xA08004,
          /* .PCU_SLC_MIF_ACK_MASK:  */  0x000002,  /* MIF_ACK field is bit 1 */
          /* .PCU_SLC_MIF_ACK_VALUE: */  0x000002,  /* MIF_ACK = ACK, 0x1 (<< 1) */

          /* .PCU_BLK_PWR_REQ_ADDR:  */  0xA0103C,
          /* .PCU_BLK_PWR_REQ_VALUE: */  0x000001,  /* POWER_REQUEST = On, 0x1 (<< 0) */
          /* .PCU_BLK_PWR_ACK_ADDR:  */  0xA0103C,
          /* .PCU_BLK_PWR_ACK_MASK:  */  0x00000C,  /* POWER_MODE field is bits 3:2 */
          /* .PCU_BLK_PWR_ACK_VALUE: */  0x000004,  /* POWER_MODE = On, 0x1 (<< 2) */
        #elif IS_ENABLED(CONFIG_SOC_GS101)
          /* .PCU_SLC_MIF_REQ_ADDR:  */  0xB0819C,
          /* .PCU_SLC_MIF_REQ_VALUE: */  0x000003,  /* Set ACTIVE_REQUEST = 1, MIS_SLCn = 1 to request MIF access */
          /* .PCU_SLC_MIF_ACK_ADDR:  */  0xB0819C,
          /* .PCU_SLC_MIF_ACK_MASK:  */  0x000002,  /* MIF_ACK field is bit 1 */
          /* .PCU_SLC_MIF_ACK_VALUE: */  0x000002,  /* MIF_ACK = ACK, 0x1 (<< 1) */

          /* .PCU_BLK_PWR_REQ_ADDR:  */  0xB02004,
          /* .PCU_BLK_PWR_REQ_VALUE: */  0x000004,  /* BLK_AOC = Initiate Wakeup Sequence, 0x1 (<< 2) */
          /* .PCU_BLK_PWR_ACK_ADDR:  */  0xB02000,
          /* .PCU_BLK_PWR_ACK_MASK:  */  0x000004,  /* BLK_AOC field is bit 2 */
          /* .PCU_BLK_PWR_ACK_VALUE: */  0x000004,  /* BLK_AOC = Active, 0x1 (<< 2) */
        #else
            #error "Unsupported silicon"
        #endif
        /* .BOOTLOADER_START_ADDR: */  addr,
	};

	pr_notice("writing reset trampoline to addr %#x\n", addr);

	reset = aoc_sram_translate(0);
	if (!reset)
		return false;

	memcpy_toio(reset, instructions, sizeof(instructions));

	return true;
}

static bool aoc_fpga_reset(struct aoc_prvdata *prvdata)
{
#ifdef AOC_JUNO
	u32 *reset = aoc_sram_translate(0x1000000);

	if (!reset)
		return false;

	aoc_take_offline(prvdata);

	/* Assert and deassert reset */
	iowrite32(0, reset);
	iowrite32(1, reset);
#endif

	return true;
}

static bool aoc_a32_reset(void)
{
	u32 pcu_value;
	void __iomem *pcu = aoc_sram_translate(AOC_PCU_BASE);

	if (!pcu)
		return false;

	pcu_value = ioread32(pcu);

	pcu_value |= 1;
	iowrite32(pcu_value, pcu);

	return true;
}

__attribute__((unused))
static int aoc_watchdog_restart(struct aoc_prvdata *prvdata)
{
	/* 4100 * 0.244 us * 100 = 100 ms */
	const int aoc_watchdog_value_ssr = 4100 * 100;
	const int aoc_reset_timeout_ms = 1000;
	const int aoc_reset_tries = 3;
	const u32 aoc_watchdog_control_ssr = 0x3F;
	const unsigned int custom_in_offset = 0x3AC4;
	const unsigned int custom_out_offset = 0x3AC0;
	int rc;
	void __iomem *pcu;
	unsigned int custom_in;
	unsigned int custom_out;
	int ret;
	bool aoc_reset_successful;
	int i;

	pcu = aoc_sram_translate(AOC_PCU_BASE);
	if (!pcu)
		return -ENODEV;

	dev_info(prvdata->dev, "asserting aoc_req\n");
	aoc_req_assert(prvdata, true);
	rc = aoc_req_wait(prvdata, true);
	if (rc) {
		dev_err(prvdata->dev, "timed out waiting for aoc_ack\n");
		return rc;
	}

	aoc_reset_successful = false;
	disable_irq_nosync(prvdata->sysmmu_nonsecure_irq);
	disable_irq_nosync(prvdata->sysmmu_secure_irq);
	for (i = 0; i < aoc_reset_tries; i++) {
		dev_info(prvdata->dev, "resetting aoc\n");
		writel(AOC_PCU_WATCHDOG_KEY_UNLOCK, pcu + AOC_PCU_WATCHDOG_KEY_OFFSET);
		if ((readl(pcu + AOC_PCU_WATCHDOG_CONTROL_OFFSET) &
				AOC_PCU_WATCHDOG_CONTROL_KEY_ENABLED_MASK) == 0) {
			dev_err(prvdata->dev, "unlock aoc watchdog failed\n");
		}
		writel(aoc_watchdog_value_ssr, pcu + AOC_PCU_WATCHDOG_VALUE_OFFSET);
		writel(aoc_watchdog_control_ssr, pcu + AOC_PCU_WATCHDOG_CONTROL_OFFSET);

		dev_info(prvdata->dev, "waiting for aoc reset to finish\n");
		if (wait_event_timeout(prvdata->aoc_reset_wait_queue, prvdata->aoc_reset_done,
				       aoc_reset_timeout_ms) == 0) {
			ret = exynos_pmu_read(custom_out_offset, &custom_out);
			dev_err(prvdata->dev,
				"AoC reset timeout custom_out=%d, ret=%d\n", custom_out, ret);
			ret = exynos_pmu_read(custom_in_offset, &custom_in);
			dev_err(prvdata->dev,
				"AoC reset timeout custom_in=%d, ret=%d\n", custom_in, ret);
			dev_err(prvdata->dev, "PCU_WATCHDOG_CONTROL = 0x%x\n",
				readl(pcu + AOC_PCU_WATCHDOG_CONTROL_OFFSET));
			dev_err(prvdata->dev, "PCU_WATCHDOG_VALUE = 0x%x\n",
				readl(pcu + AOC_PCU_WATCHDOG_VALUE_OFFSET));
		} else {
			aoc_reset_successful = true;
			break;
		}
	}
	enable_irq(prvdata->sysmmu_nonsecure_irq);
	enable_irq(prvdata->sysmmu_secure_irq);
	if (!aoc_reset_successful) {
		/* Trigger acpm ramdump since we timed out the aoc reset request */
		dbg_snapshot_emergency_reboot("AoC Restart timed out");
		return -ETIMEDOUT;
	}
	reset_sensor_power(prvdata, false);
	dev_info(prvdata->dev, "aoc reset finished\n");
	prvdata->aoc_reset_done = false;

	/*
	 * AOC_TZPC has been restored by ACPM, so we can access AOC_S2MPU.
	 * Restore AOC_S2MPU.
	 */
	writel(prvdata->aoc_s2mpu_saved_value, prvdata->aoc_s2mpu_virt + AOC_S2MPU_CTRL0);

	/* Restore SysMMU settings by briefly setting AoC to runtime active. Since SysMMU is a
	 * supplier to AoC, it will be set to runtime active as a side effect. */
	rc = pm_runtime_set_active(prvdata->dev);
	if (rc < 0) {
		dev_err(prvdata->dev, "sysmmu restore failed: pm_runtime_resume rc = %d\n", rc);
		return rc;
	}
	rc = pm_runtime_set_suspended(prvdata->dev);
	if (rc < 0) {
		dev_err(prvdata->dev, "sysmmu restore failed: pm_runtime_suspend rc = %d\n", rc);
		return rc;
	}

	rc = start_firmware_load(prvdata->dev);
	if (rc) {
		dev_err(prvdata->dev, "load aoc firmware failed: rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static void acpm_aoc_reset_callback(unsigned int *cmd, unsigned int size)
{
	struct aoc_prvdata *prvdata;

	if (!aoc_platform_device)
		return;

	prvdata = platform_get_drvdata(aoc_platform_device);
	pr_info("AOC prvdata pointer is: %p (expected: %p)", prvdata, aoc_prvdata_copy);
	prvdata->aoc_reset_done = true;
	wake_up(&prvdata->aoc_reset_wait_queue);
}

static ssize_t coredump_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", prvdata->total_coredumps);
}

static DEVICE_ATTR_RO(coredump_count);

static ssize_t restart_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", prvdata->total_restarts);
}

static DEVICE_ATTR_RO(restart_count);

static ssize_t revision_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	u32 fw_rev, hw_rev;

	if (!aoc_fw_ready())
		return scnprintf(buf, PAGE_SIZE, "Offline\n");

	fw_rev = le32_to_cpu(aoc_control->fw_version);
	hw_rev = le32_to_cpu(aoc_control->hw_version);
	return scnprintf(buf, PAGE_SIZE,
			 "FW Revision : %#x\nHW Revision : %#x\n", fw_rev,
			 hw_rev);
}

static DEVICE_ATTR_RO(revision);

static uint64_t clock_offset(void)
{
	u64 clock_offset;

	if (!aoc_fw_ready())
		return 0;

	memcpy_fromio(&clock_offset, &aoc_control->system_clock_offset,
		      sizeof(clock_offset));

	return le64_to_cpu(clock_offset);
}

static inline u64 sys_tick_to_aoc_tick(u64 sys_tick)
{
	return (sys_tick - clock_offset()) / AOC_CLOCK_DIVIDER;
}

static ssize_t aoc_clock_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	u64 counter;

	if (!aoc_fw_ready())
		return scnprintf(buf, PAGE_SIZE, "0\n");

	counter = arch_timer_read_counter();

	return scnprintf(buf, PAGE_SIZE, "%llu\n",
			 sys_tick_to_aoc_tick(counter));
}

static DEVICE_ATTR_RO(aoc_clock);

static ssize_t aoc_clock_and_kernel_boottime_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	u64 counter;
	ktime_t kboottime;

	if (!aoc_fw_ready())
		return scnprintf(buf, PAGE_SIZE, "0 0\n");

	counter = arch_timer_read_counter();
	kboottime = ktime_get_boottime();

	return scnprintf(buf, PAGE_SIZE, "%llu %llu\n",
			 sys_tick_to_aoc_tick(counter), (u64)kboottime);
}

static DEVICE_ATTR_RO(aoc_clock_and_kernel_boottime);

static ssize_t clock_offset_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	if (!aoc_fw_ready())
		return scnprintf(buf, PAGE_SIZE, "0\n");

	return scnprintf(buf, PAGE_SIZE, "%lld\n", clock_offset());
}

static DEVICE_ATTR_RO(clock_offset);

static ssize_t services_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	int services = aoc_num_services();
	int ret = 0;
	int i;

	ret += scnprintf(buf, PAGE_SIZE, "Services : %d\n", services);
	for (i = 0; i < services && ret < (PAGE_SIZE - 1); i++) {
		aoc_service *s = service_at_index(prvdata, i);
		struct aoc_ipc_service_header *hdr =
			(struct aoc_ipc_service_header *)s;

		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d : \"%s\" mbox %d\n",
				 i, aoc_service_name(s), aoc_service_irq_index(s));
		if (hdr->regions[0].slots > 0) {
			ret += scnprintf(
				buf + ret, PAGE_SIZE - ret,
				" Up Size:%ux%uB Tx:%u Rx:%u\n",
				hdr->regions[0].slots, hdr->regions[0].size,
				hdr->regions[0].tx, hdr->regions[0].rx);
		}

		if (hdr->regions[1].slots > 0) {
			ret += scnprintf(
				buf + ret, PAGE_SIZE - ret,
				" Down Size:%ux%uB Tx:%u Rx:%u\n",
				hdr->regions[1].slots, hdr->regions[1].size,
				hdr->regions[1].tx, hdr->regions[1].rx);
		}
	}

	return ret;
}

static DEVICE_ATTR_RO(services);

static int start_firmware_load(struct device *dev)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);

	dev_notice(dev, "attempting to load firmware \"%s\"\n",
		   prvdata->firmware_name);
	return request_firmware_nowait(THIS_MODULE, true,
				       prvdata->firmware_name, dev, GFP_KERNEL,
				       dev, aoc_fw_callback);
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s", prvdata->firmware_name);
}

static ssize_t firmware_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	char buffer[MAX_FIRMWARE_LENGTH];
	char *trimmed = NULL;

	if (strscpy(buffer, buf, sizeof(buffer)) <= 0)
		return -E2BIG;

	if (strchr(buffer, '/') != NULL) {
		dev_err(dev, "firmware path must not contain '/'\n");
		return -EINVAL;
	}

	/* Strip whitespace (including \n) */
	trimmed = strim(buffer);

	strscpy(prvdata->firmware_name, trimmed,
		sizeof(prvdata->firmware_name));
	start_firmware_load(dev);

	return count;
}

static DEVICE_ATTR_RW(firmware);

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	char reason_str[MAX_RESET_REASON_STRING_LEN + 1];
	size_t reason_str_len = min(MAX_RESET_REASON_STRING_LEN, count);

	if (aoc_state != AOC_STATE_ONLINE || work_busy(&prvdata->watchdog_work)) {
		dev_err(dev, "Reset requested while AoC is not online");
		return -ENODEV;
	}

	strscpy(reason_str, buf, reason_str_len);
	reason_str[reason_str_len] = '\0';
	dev_err(dev, "Reset requested from userspace, reason: %s", reason_str);

	if (prvdata->no_ap_resets) {
		dev_err(dev, "Reset request rejected, option disabled via persist options");
	} else {
		disable_irq_nosync(prvdata->watchdog_irq);
		strlcpy(prvdata->ap_reset_reason, reason_str, AP_RESET_REASON_LENGTH);
		prvdata->ap_triggered_reset = true;
		schedule_work(&prvdata->watchdog_work);
	}
	return count;
}

static DEVICE_ATTR_WO(reset);

static ssize_t sensor_power_enable_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 10, &val) == 0) {
		dev_info(prvdata->dev,"sensor_power_enable %d", val);
		configure_sensor_regulator(prvdata, !!val);
	}
	return count;
}

static DEVICE_ATTR_WO(sensor_power_enable);

static struct attribute *aoc_attrs[] = {
	&dev_attr_firmware.attr,
	&dev_attr_revision.attr,
	&dev_attr_services.attr,
	&dev_attr_coredump_count.attr,
	&dev_attr_restart_count.attr,
	&dev_attr_clock_offset.attr,
	&dev_attr_aoc_clock.attr,
	&dev_attr_aoc_clock_and_kernel_boottime.attr,
	&dev_attr_reset.attr,
	&dev_attr_sensor_power_enable.attr,
	NULL
};

ATTRIBUTE_GROUPS(aoc);

static int aoc_platform_probe(struct platform_device *dev);
static int aoc_platform_remove(struct platform_device *dev);
static void aoc_platform_shutdown(struct platform_device *dev);

static const struct of_device_id aoc_match[] = {
	{
		.compatible = "google,aoc",
	},
	{},
};

static struct platform_driver aoc_driver = {
	.probe = aoc_platform_probe,
	.remove = aoc_platform_remove,
	.shutdown = aoc_platform_shutdown,
	.driver = {
			.name = "aoc",
			.owner = THIS_MODULE,
			.pm = &aoc_core_pm_ops,
			.of_match_table = of_match_ptr(aoc_match),
		},
};

static int aoc_bus_match(struct device *dev, struct device_driver *drv)
{
	struct aoc_driver *driver = AOC_DRIVER(drv);

	const char *device_name = dev_name(dev);
	bool driver_matches_by_name = (driver->service_names != NULL);

	pr_debug("bus match dev:%s drv:%s\n", device_name, drv->name);

	/*
	 * If the driver matches by name, only call probe if the name matches.
	 *
	 * If there is a specific driver matching this service, do not allow a
	 * generic driver to claim the service
	 */
	if (!driver_matches_by_name && has_name_matching_driver(device_name)) {
		pr_debug("ignoring generic driver for service %s\n",
			 device_name);
		return 0;
	}

	/* Drivers with a name only match services with that name */
	if (driver_matches_by_name &&
	    !driver_matches_service_by_name(drv, (char *)device_name)) {
		return 0;
	}

	return 1;
}

static int aoc_bus_probe(struct device *dev)
{
	struct aoc_service_dev *the_dev = AOC_DEVICE(dev);
	struct aoc_driver *driver = AOC_DRIVER(dev->driver);

	pr_debug("bus probe dev:%s\n", dev_name(dev));
	if (!driver->probe)
		return -ENODEV;

	return driver->probe(the_dev);
}

static int aoc_bus_remove(struct device *dev)
{
	struct aoc_service_dev *aoc_dev = AOC_DEVICE(dev);
	struct aoc_driver *drv = AOC_DRIVER(dev->driver);
	int ret = -EINVAL;

	pr_notice("bus remove %s\n", dev_name(dev));

	if (drv->remove)
		ret = drv->remove(aoc_dev);

	return ret;
}

int aoc_driver_register(struct aoc_driver *driver)
{
	driver->drv.bus = &aoc_bus_type;
	return driver_register(&driver->drv);
}
EXPORT_SYMBOL_GPL(aoc_driver_register);

void aoc_driver_unregister(struct aoc_driver *driver)
{
	driver_unregister(&driver->drv);
}
EXPORT_SYMBOL_GPL(aoc_driver_unregister);

static void aoc_clear_gpio_interrupt(void)
{
#if defined(GPIO_INTERRUPT) && !defined(AOC_JUNO)
	int reg = GPIO_INTERRUPT, val;
	u32 *gpio_register =
		aoc_sram_translate(AOC_GPIO_BASE + ((reg / 32) * 12));

	val = ioread32(gpio_register);
	val &= ~(1 << (reg % 32));
	iowrite32(val, gpio_register);
#endif
}

static void aoc_configure_interrupt(void)
{
	aoc_clear_gpio_interrupt();
}

static int aoc_remove_device(struct device *dev, void *ctx)
{
	struct aoc_service_dev *the_dev = AOC_DEVICE(dev);

	/*
	 * Once dead is set to true, function calls using this AoC device will return error.
	 * Clients may still hold a refcount on the AoC device, so freeing is delayed.
	 */
	the_dev->dead = true;

	// Allow any pending reads and writes to finish before removing devices
	wake_up(&the_dev->read_queue);
	wake_up(&the_dev->write_queue);

	device_unregister(dev);

	return 0;
}

/* Service devices are freed after offline is complete */
static void aoc_device_release(struct device *dev)
{
	struct aoc_service_dev *the_dev = AOC_DEVICE(dev);

	pr_debug("%s %s\n", __func__, dev_name(dev));

	kfree(the_dev);
}

static struct aoc_service_dev *create_service_device(struct aoc_prvdata *prvdata, int index)
{
	struct device *parent = prvdata->dev;
	char service_name[32];
	const char *name;
	aoc_service *s;
	struct aoc_service_dev *dev;

	s = service_at_index(prvdata, index);
	if (!s)
		return NULL;

	dev = kzalloc(sizeof(struct aoc_service_dev), GFP_KERNEL);
	prvdata->services[index] = dev;

	name = aoc_service_name(s);
	if (!name)
		return NULL;

	memcpy_fromio(service_name, name, sizeof(service_name));

	dev_set_name(&dev->dev, "%s", service_name);
	dev->dev.parent = parent;
	dev->dev.bus = &aoc_bus_type;
	dev->dev.release = aoc_device_release;

	dev->service_index = index;
	dev->mbox_index = aoc_service_irq_index(s);
	dev->service = s;
	dev->ipc_base = prvdata->ipc_base;
	dev->dead = false;

	if (aoc_service_is_queue(s))
		dev->wake_capable = true;

	init_waitqueue_head(&dev->read_queue);
	init_waitqueue_head(&dev->write_queue);

	return dev;
}

static void trigger_aoc_ramdump(struct aoc_prvdata *prvdata)
{
	struct mbox_chan *channel = prvdata->mbox_channels[15].channel;
	static const uint32_t command[] = { 0, 0, 0, 0, 0x0deada0c, 0, 0, 0 };

	dev_notice(prvdata->dev, "Attempting to force AoC coredump\n");

	mbox_send_message(channel, (void *)&command);
}

static void signal_aoc(struct mbox_chan *channel)
{
#ifdef AOC_JUNO
	(void)channel;

	u32 mask = (1 << AOC_DOWNCALL_DOORBELL);

	/* The signal is called as directly after writing a message to shared
	 * memory, so make sure all pending writes are flushed before actually
	 * sending the signal
	 */
	wmb();
	iowrite32(mask,
		  aoc_sram_translate(AOC_PCU_BASE + AOC_PCU_DB_SET_OFFSET));
#else
	mbox_send_message(channel, NULL);
#endif
}

static int aoc_iommu_fault_handler(struct iommu_fault *fault, void *token)
{
	struct device *dev = token;

	dev_err(dev, "aoc iommu fault: fault->type = %u\n", fault->type);
	dev_err(dev, "fault->event: reason = %u, flags = %#010x, addr = %#010llx\n",
		fault->event.reason, fault->event.flags, fault->event.addr);
	dev_err(dev, "fault->prm: flags = %#010x, addr = %#010llx\n",
		fault->prm.flags, fault->prm.addr);

	/* Tell the IOMMU driver that the fault is non-fatal. */
	return -EAGAIN;
}

#define SSMT_BYPASS_VALUE	0x80000000U
#define SSMT_NS_READ_PID(n)	(0x4000 + 4 * (n))
#define SSMT_NS_WRITE_PID(n)	(0x4200 + 4 * (n))

#if IS_ENABLED(CONFIG_SOC_GS101)
static void aoc_configure_ssmt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int stream_id;

	void __iomem *ssmt_base = devm_platform_ioremap_resource_byname(pdev, "ssmt_aoc");

	if (IS_ERR(ssmt_base)) {
		dev_err(dev, "ssmt_aoc base address failure: %ld\n", PTR_ERR(ssmt_base));
		return;
	}

	/* Configure registers NS_READ_PID_<n>, NS_WRITE_PID_<n> for each stream id */
	for (stream_id = 0; stream_id <= 32; stream_id++) {
		/* Skip over stream id 31 */
		if (stream_id == 31)
			continue;
		writel_relaxed(SSMT_BYPASS_VALUE, ssmt_base + SSMT_NS_READ_PID(stream_id));
		writel_relaxed(SSMT_BYPASS_VALUE, ssmt_base + SSMT_NS_WRITE_PID(stream_id));
	}

	devm_iounmap(dev, ssmt_base);
}
#else
static inline void aoc_configure_ssmt( struct platform_device *pdev
    __attribute__((unused))) { }
#endif

static void aoc_configure_sysmmu(struct aoc_prvdata *p)
{
#ifndef AOC_JUNO
	struct iommu_domain *domain = p->domain;
	struct device *dev = p->dev;
	int rc;

	rc = iommu_register_device_fault_handler(dev, aoc_iommu_fault_handler, dev);
	if (rc)
		dev_err(dev, "iommu_register_device_fault_handler failed: rc = %d\n", rc);

	/* Map in the AoC carveout */
	if (iommu_map(domain, 0x98000000, p->dram_resource.start, p->dram_size,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping carveout failed\n");

#if IS_ENABLED(CONFIG_SOC_GS201)
	/* Use a 1MB mapping instead of individual mailboxes for now */
	/* TODO: Turn the mailbox address ranges into dtb entries */
	if (iommu_map(domain, 0x9E000000, 0x18200000, SZ_2M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping mailboxes failed\n");

	/* Map in GSA mailbox */
	if (iommu_map(domain, 0x9E200000, 0x17C00000, SZ_1M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping gsa mailbox failed\n");

	/* Map in modem registers */
	if (iommu_map(domain, 0x9E300000, 0x40000000, SZ_1M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping modem failed\n");

	/* Map in BLK_TPU */
	/* if (iommu_map(domain, 0x9E600000, 0x1CE00000, SZ_2M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping mailboxes failed\n"); */

	/* Map in the xhci_dma carveout */
	if (iommu_map(domain, 0x9B000000, 0x97000000, SZ_4M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping xhci_dma carveout failed\n");

	/* Map in USB for low power audio */
	if (iommu_map(domain, 0x9E500000, 0x11200000, SZ_1M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping usb failed\n");
#else
	/* Map in the xhci_dma carveout */
	if (iommu_map(domain, 0x9B000000, 0x97000000, SZ_4M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping xhci_dma carveout failed\n");

	/* Use a 1MB mapping instead of individual mailboxes for now */
	/* TODO: Turn the mailbox address ranges into dtb entries */
	if (iommu_map(domain, 0x9E000000, 0x17600000, SZ_1M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping mailboxes failed\n");

	/* Map in GSA mailbox */
	if (iommu_map(domain, 0x9E100000, 0x17C00000, SZ_1M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping gsa mailbox failed\n");

	/* Map in USB for low power audio */
	if (iommu_map(domain, 0x9E200000, 0x11100000, SZ_1M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping usb failed\n");

	/* Map in modem registers */
	if (iommu_map(domain, 0x9E300000, 0x40000000, SZ_1M,
		      IOMMU_READ | IOMMU_WRITE))
		dev_err(dev, "mapping modem failed\n");
#endif
#endif
}

static void aoc_clear_sysmmu(struct aoc_prvdata *p)
{
#ifndef AOC_JUNO
	struct iommu_domain *domain = p->domain;

	/* Memory carveout */
	iommu_unmap(domain, 0x98000000, p->dram_size);
	iommu_unmap(domain, 0x9B000000, SZ_4M);

	/* Device registers */
	iommu_unmap(domain, 0x9E000000, SZ_1M);
	iommu_unmap(domain, 0x9E100000, SZ_1M);
	iommu_unmap(domain, 0x9E200000, SZ_1M);
	iommu_unmap(domain, 0x9E300000, SZ_1M);
#endif
}

static void aoc_monitor_online(struct work_struct *work)
{
	struct aoc_prvdata *prvdata =
		container_of(work, struct aoc_prvdata, monitor_work.work);
	int restart_rc;


	mutex_lock(&aoc_service_lock);
	if (aoc_state == AOC_STATE_FIRMWARE_LOADED) {
		dev_err(prvdata->dev, "aoc init no respond, try restart\n");

		disable_irq_nosync(prvdata->watchdog_irq);
		aoc_take_offline(prvdata);
		restart_rc = aoc_watchdog_restart(prvdata);
		if (restart_rc)
			dev_info(prvdata->dev,
				"aoc restart failed: rc = %d\n", restart_rc);
		else
			dev_info(prvdata->dev,
				"aoc restart succeeded\n");
	}
	mutex_unlock(&aoc_service_lock);
}

static void aoc_did_become_online(struct work_struct *work)
{
	struct aoc_prvdata *prvdata =
		container_of(work, struct aoc_prvdata, online_work);
	struct device *dev = prvdata->dev;
	int i, s;

	cancel_delayed_work_sync(&prvdata->monitor_work);

	mutex_lock(&aoc_service_lock);

	s = aoc_num_services();

	aoc_req_assert(prvdata, false);

	pr_notice("firmware version %s did become online with %d services\n",
		  prvdata->firmware_version ? prvdata->firmware_version : "0",
		  aoc_num_services());

	if (s > AOC_MAX_ENDPOINTS) {
		dev_err(dev, "Firmware supports too many (%d) services\n", s);
		goto err;
	}

	if (!service_names_are_valid(prvdata)) {
		pr_err("invalid service names found.  Ignoring\n");
		goto err;
	}

	for (i = 0; i < s; i++) {
		if (!validate_service(prvdata, i)) {
			pr_err("service %d invalid\n", i);
			goto err;
		}
	}

	prvdata->services = devm_kcalloc(prvdata->dev, s, sizeof(struct aoc_service_dev *), GFP_KERNEL);
	if (!prvdata->services) {
		dev_err(prvdata->dev, "failed to allocate service array\n");
		goto err;
	}

	prvdata->total_services = s;

	if (prvdata->read_blocked_mask == NULL) {
		prvdata->read_blocked_mask = devm_kcalloc(prvdata->dev, BITS_TO_LONGS(s),
							  sizeof(unsigned long), GFP_KERNEL);
		if (!prvdata->read_blocked_mask)
			goto err;
	}

	if (prvdata->write_blocked_mask == NULL) {
		prvdata->write_blocked_mask = devm_kcalloc(prvdata->dev, BITS_TO_LONGS(s),
							  sizeof(unsigned long), GFP_KERNEL);
		if (!prvdata->write_blocked_mask)
			goto err;
	}

	for (i = 0; i < s; i++) {
		create_service_device(prvdata, i);
	}

	aoc_state = AOC_STATE_ONLINE;

	for (i = 0; i < s; i++)
		device_register(&prvdata->services[i]->dev);

err:
	mutex_unlock(&aoc_service_lock);
}

static bool configure_sensor_regulator(struct aoc_prvdata *prvdata, bool enable)
{
	bool check_enabled;
	int i;
	if (enable) {
		check_enabled = true;
		for (i = 0; i < prvdata->sensor_power_count; i++) {
			if (!prvdata->sensor_regulator[i] ||
					regulator_is_enabled(prvdata->sensor_regulator[i])) {
				continue;
			}

			if (regulator_enable(prvdata->sensor_regulator[i])) {
				pr_warn("encountered error on enabling %s.",
					prvdata->sensor_power_list[i]);
			}
			check_enabled &= regulator_is_enabled(prvdata->sensor_regulator[i]);
		}
	} else {
		check_enabled = false;
		for (i = prvdata->sensor_power_count - 1; i >= 0; i--) {
			if (!prvdata->sensor_regulator[i] ||
					!regulator_is_enabled(prvdata->sensor_regulator[i])) {
				continue;
			}

			if (regulator_disable(prvdata->sensor_regulator[i])) {
				pr_warn("encountered error on disabling %s.",
					prvdata->sensor_power_list[i]);
			}
			check_enabled |= regulator_is_enabled(prvdata->sensor_regulator[i]);
		}
	}

	return (check_enabled == enable);
}

static void reset_sensor_power(struct aoc_prvdata *prvdata, bool is_init)
{
	const int max_retry = 5;
	int count;
	bool success;

	if (prvdata->sensor_power_count == 0) {
		return;
	}

	if (!is_init) {
		count = 0;
		success = false;
		while (!success && count < max_retry) {
			success = configure_sensor_regulator(prvdata, false);
			count++;
		}
		if (!success) {
			pr_err("failed to disable sensor power after %d retry.", max_retry);
		} else {
			pr_info("sensor power is disabled.");
		}

		msleep(150);
	}

	count = 0;
	success = false;
	while (!success && count < max_retry) {
		success = configure_sensor_regulator(prvdata, true);
		count++;
	}
	if (!success) {
		pr_err("failed to enable sensor power after %d retry.", max_retry);
	} else {
		pr_info("sensor power is enabled.");
	}
}

static void aoc_take_offline(struct aoc_prvdata *prvdata)
{
	int rc;

	/* check if devices/services are ready */
	if (aoc_state == AOC_STATE_ONLINE) {
		pr_notice("taking aoc offline\n");
		aoc_state = AOC_STATE_OFFLINE;

		/* wait until aoc_process or service write/read finish */
		while (!!atomic_read(&prvdata->aoc_process_active));

		bus_for_each_dev(&aoc_bus_type, NULL, NULL, aoc_remove_device);

		if (aoc_control)
			aoc_control->magic = 0;

		if (prvdata->services) {
			devm_kfree(prvdata->dev, prvdata->services);
			prvdata->services = NULL;
			prvdata->total_services = 0;
		}

		/* wakeup AOC before calling GSA */
		aoc_req_assert(prvdata, true);
		rc = aoc_req_wait(prvdata, true);
		if (rc)
			dev_err(prvdata->dev, "timed out waiting for aoc_ack\n");
	}

	if(prvdata->protected_by_gsa) {
		/* TODO(b/275463650): GSA_AOC_SHUTDOWN needs to be 4, but the current
		 * header defines as 2.  Change this to enum when the header is updated.
		 */
		rc = gsa_send_aoc_cmd(prvdata->gsa_dev, 4);
		/* rc is the new state of AOC unless it's negative,
		 * in which case it's an error code
		 */
		if(rc != GSA_AOC_STATE_LOADED) {
			if(rc >= 0) {
				dev_err(prvdata->dev,
					"GSA shutdown command returned unexpected state: %d\n", rc);
			} else {
				dev_err(prvdata->dev,
					"GSA shutdown command returned error: %d\n", rc);
			}
		}

		rc = gsa_unload_aoc_fw_image(prvdata->gsa_dev);
		if (rc)
			dev_err(prvdata->dev, "GSA unload firmware failed: %d\n", rc);
	}
}

static void aoc_process_services(struct aoc_prvdata *prvdata, int offset)
{
	struct aoc_service_dev *service_dev;
	aoc_service *service;
	int services;
	int i;

	atomic_inc(&prvdata->aoc_process_active);

	if (aoc_state != AOC_STATE_ONLINE || work_busy(&prvdata->watchdog_work))
		goto exit;

	services = aoc_num_services();
	for (i = 0; i < services; i++) {
		service_dev = service_dev_at_index(prvdata, i);
		if (!service_dev)
			goto exit;

		service = service_dev->service;
		if (service_dev->mbox_index != offset)
			continue;

		if (service_dev->handler) {
			service_dev->handler(service_dev);
		} else {
			if (test_bit(i, prvdata->read_blocked_mask) &&
			    aoc_service_can_read_message(service, AOC_UP))
				wake_up(&service_dev->read_queue);

			if (test_bit(i, prvdata->write_blocked_mask) &&
			    aoc_service_can_write_message(service, AOC_DOWN))
				wake_up(&service_dev->write_queue);
		}
	}
exit:
	atomic_dec(&prvdata->aoc_process_active);
}

void aoc_set_map_handler(struct aoc_service_dev *dev, aoc_map_handler handler,
			 void *ctx)
{
	struct device *parent = dev->dev.parent;
	struct aoc_prvdata *prvdata = dev_get_drvdata(parent);

	prvdata->map_handler = handler;
	prvdata->map_handler_ctx = ctx;
}
EXPORT_SYMBOL_GPL(aoc_set_map_handler);

void aoc_remove_map_handler(struct aoc_service_dev *dev)
{
	struct device *parent = dev->dev.parent;
	struct aoc_prvdata *prvdata = dev_get_drvdata(parent);

	prvdata->map_handler = NULL;
	prvdata->map_handler_ctx = NULL;
}
EXPORT_SYMBOL_GPL(aoc_remove_map_handler);

static void aoc_pheap_alloc_cb(struct samsung_dma_buffer *buffer, void *ctx)
{
	struct device *dev = ctx;
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	struct sg_table *sg = &buffer->sg_table;
	phys_addr_t phys;
	size_t size;

	if (sg->nents != 1) {
		dev_warn(dev, "Unable to map sg_table with %d ents\n",
			 sg->nents);
		return;
	}

	phys = sg_phys(&sg->sgl[0]);
	phys = aoc_dram_translate_to_aoc(prvdata, phys);
	size = sg->sgl[0].length;

	mutex_lock(&aoc_service_lock);
	if (prvdata->map_handler) {
		prvdata->map_handler((u64)buffer->priv, phys, size, true,
				     prvdata->map_handler_ctx);
	}
	mutex_unlock(&aoc_service_lock);
}

static void aoc_pheap_free_cb(struct samsung_dma_buffer *buffer, void *ctx)
{
	struct device *dev = ctx;
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	struct sg_table *sg = &buffer->sg_table;
	phys_addr_t phys;
	size_t size;

	if (sg->nents != 1) {
		dev_warn(dev, "Unable to map sg_table with %d ents\n",
			 sg->nents);
		return;
	}

	phys = sg_phys(&sg->sgl[0]);
	phys = aoc_dram_translate_to_aoc(prvdata, phys);
	size = sg->sgl[0].length;

	mutex_lock(&aoc_service_lock);
	if (prvdata->map_handler) {
		prvdata->map_handler((u64)buffer->priv, phys, size, false,
				     prvdata->map_handler_ctx);
	}
	mutex_unlock(&aoc_service_lock);
}

#ifdef AOC_JUNO
static irqreturn_t aoc_int_handler(int irq, void *dev)
{
	aoc_clear_gpio_interrupt();

	/* Transitioning from offline to online */
	if (aoc_state == AOC_STATE_FIRMWARE_LOADED) {
		if (aoc_fw_ready())
			aoc_state = AOC_STATE_STARTING;
			schedule_work(&aoc_online_work);
		}
	} else if (aoc_state == AOC_STATE_ONLINE) {
		aoc_process_services(dev_get_drvdata(dev), 0);
	}

	return IRQ_HANDLED;
}
#else
static irqreturn_t watchdog_int_handler(int irq, void *dev)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);

	/* AP shouldn't access AoC registers to clear the IRQ. */
	/* Mask the IRQ until the IRQ gets cleared by AoC reset during SSR. */
	disable_irq_nosync(irq);
	schedule_work(&prvdata->watchdog_work);

	return IRQ_HANDLED;
}

static struct aoc_section_header *find_ramdump_section(struct aoc_ramdump_header
						*ramdump_header, int section_type)
{
	int i;

	for (i = 0; i < ramdump_header->num_sections; i++)
		if (ramdump_header->sections[i].type == section_type)
			return &ramdump_header->sections[i];

	return NULL;
}

static void aoc_watchdog(struct work_struct *work)
{
	struct aoc_prvdata *prvdata =
		container_of(work, struct aoc_prvdata, watchdog_work);

	struct aoc_ramdump_header *ramdump_header =
		(struct aoc_ramdump_header *)((unsigned long)prvdata->dram_virt +
					      RAMDUMP_HEADER_OFFSET);
	struct wakeup_source *ws =
		wakeup_source_register(prvdata->dev, dev_name(prvdata->dev));
	unsigned long ramdump_timeout;
	unsigned long carveout_paddr_from_aoc;
	unsigned long carveout_vaddr_from_aoc;
	size_t i;
	size_t num_pages;
	struct page **dram_pages = NULL;
	void *dram_cached = NULL;
	int sscd_retries = 20;
	const int sscd_retry_ms = 1000;
	int sscd_rc;
	char crash_info[RAMDUMP_SECTION_CRASH_INFO_SIZE];
	int restart_rc;
	bool ap_reset = false;
	struct aoc_section_header *crash_info_section =
		find_ramdump_section(ramdump_header, SECTION_TYPE_CRASH_INFO);

	prvdata->total_restarts++;

	/* Initialize crash_info[0] to identify if it has changed later in the function. */
	crash_info[0] = 0;

	if (prvdata->ap_triggered_reset) {
		if ((ktime_get_real_ns() - prvdata->last_reset_time_ns) / 1000000
			<= prvdata->reset_hysteresis_trigger_ms) {
			/* If the watchdog was triggered recently, busy wait to
			 * avoid overlapping resets.
			 */
			dev_err(prvdata->dev, "Triggered hysteresis for AP reset, waiting %d ms",
				RESET_WAIT_TIME_MS +
				prvdata->reset_wait_time_index * RESET_WAIT_TIME_INCREMENT_MS);
			msleep(RESET_WAIT_TIME_MS +
				prvdata->reset_wait_time_index * RESET_WAIT_TIME_INCREMENT_MS);
			if (prvdata->reset_wait_time_index < RESET_WAIT_TIMES_NUM)
				prvdata->reset_wait_time_index++;
		} else {
			prvdata->reset_wait_time_index = 0;
		}
	}

	prvdata->last_reset_time_ns = ktime_get_real_ns();

	sscd_info.name = "aoc";
	sscd_info.seg_count = 0;

	dev_err(prvdata->dev, "aoc watchdog triggered, generating coredump\n");
	dev_err(prvdata->dev, "holding %s wakelock for 10 sec\n", ws->name);
	pm_wakeup_ws_event(ws, 10000, true);

	if (!sscd_pdata.sscd_report) {
		dev_err(prvdata->dev, "aoc coredump failed: no sscd driver\n");
		goto err_coredump;
	}

	if (prvdata->ap_triggered_reset) {
		dev_info(prvdata->dev, "AP triggered reset, reason: [%s]",
			prvdata->ap_reset_reason);
		prvdata->ap_triggered_reset = false;
		ap_reset = true;
		trigger_aoc_ramdump(prvdata);
	}

	ramdump_timeout = jiffies + (5 * HZ);
	while (time_before(jiffies, ramdump_timeout)) {
		if (ramdump_header->valid)
			break;
		msleep(100);
	}

	if (!ramdump_header->valid) {
		dev_err(prvdata->dev, "aoc coredump timed out, coredump only contains DRAM\n");
		if (crash_info_section) {
			const char *crash_reason = (const char *)ramdump_header +
				crash_info_section->offset;
			bool crash_reason_valid = (strnlen(crash_reason,
				sizeof(crash_info)) != 0);

			snprintf(crash_info, sizeof(crash_info),
				"AoC watchdog : %s (incomplete %u:%u)",
				crash_reason_valid ? crash_reason : "unknown reason",
				ramdump_header->breadcrumbs[0], ramdump_header->breadcrumbs[1]);
		} else {
			dev_err(prvdata->dev, "could not find crash info section in aoc coredump header");
			snprintf(crash_info, sizeof(crash_info),
				"AoC watchdog : unknown reason (incomplete %u:%u)",
				ramdump_header->breadcrumbs[0], ramdump_header->breadcrumbs[1]);
		}
	}

	if (ramdump_header->valid && memcmp(ramdump_header, RAMDUMP_MAGIC, sizeof(RAMDUMP_MAGIC))) {
		dev_err(prvdata->dev,
			"aoc coredump failed: invalid magic (corruption or incompatible firmware?)\n");
		strscpy(crash_info, "AoC Watchdog : coredump corrupt",
			sizeof(crash_info));
	}

	num_pages = DIV_ROUND_UP(prvdata->dram_size, PAGE_SIZE);
	dram_pages = vmalloc(num_pages * sizeof(*dram_pages));
	if (!dram_pages) {
		dev_err(prvdata->dev,
			"aoc coredump failed: alloc dram_pages failed\n");
		goto err_vmalloc;
	}
	for (i = 0; i < num_pages; i++)
		dram_pages[i] = phys_to_page(prvdata->dram_resource.start +
					     (i * PAGE_SIZE));
	dram_cached = vmap(dram_pages, num_pages, VM_MAP, PAGE_KERNEL_RO);
	if (!dram_cached) {
		dev_err(prvdata->dev,
			"aoc coredump failed: vmap dram_pages failed\n");
		goto err_vmap;
	}

	if (ramdump_header->valid) {
		if (crash_info_section && crash_info_section->flags & RAMDUMP_FLAG_VALID) {
			const char *crash_reason = (const char *)ramdump_header +
				crash_info_section->offset;
			dev_info(prvdata->dev, "aoc coredump has valid coredump header, crash reason [%s]",
				crash_reason);
			strscpy(crash_info, crash_reason, sizeof(crash_info));
		} else {
			dev_info(prvdata->dev, "aoc coredump has valid coredump header, but invalid crash reason");
			strscpy(crash_info, "AoC Watchdog : invalid crash info",
				sizeof(crash_info));
		}
	}

	if (ap_reset) {
		/* Prefer the user specified reason */
		scnprintf(crash_info, sizeof(crash_info), "AP Reset: %s", prvdata->ap_reset_reason);
	}

	if (crash_info[0] == 0)
		strscpy(crash_info, "AoC Watchdog: empty crash info string", sizeof(crash_info));

	dev_info(prvdata->dev, "aoc crash info: [%s]", crash_info);

	/* TODO(siqilin): Get paddr and vaddr base from firmware instead */
	carveout_paddr_from_aoc = 0x98000000;
	carveout_vaddr_from_aoc = 0x78000000;
	/* Entire AoC DRAM carveout, coredump is stored within the carveout */
	sscd_info.segs[0].addr = dram_cached;
	sscd_info.segs[0].size = prvdata->dram_size;
	sscd_info.segs[0].paddr = (void *)carveout_paddr_from_aoc;
	sscd_info.segs[0].vaddr = (void *)carveout_vaddr_from_aoc;
	sscd_info.seg_count = 1;

	/*
	 * sscd_report() returns -EAGAIN if there are no readers to consume a
	 * coredump. Retry sscd_report() with a sleep to handle the race condition
	 * where AoC crashes before the userspace daemon starts running.
	 */
	for (i = 0; i <= sscd_retries; i++) {
		sscd_rc = sscd_pdata.sscd_report(&sscd_dev, sscd_info.segs,
						 sscd_info.seg_count,
						 SSCD_FLAGS_ELFARM64HDR,
						 crash_info);
		if (sscd_rc != -EAGAIN)
			break;

		msleep(sscd_retry_ms);
	}

	if (sscd_rc == 0) {
		prvdata->total_coredumps++;
		dev_info(prvdata->dev, "aoc coredump done\n");
	} else {
		dev_err(prvdata->dev, "aoc coredump failed: sscd_rc = %d\n", sscd_rc);
	}

	if (dram_cached)
		vunmap(dram_cached);
err_vmap:
	vfree(dram_pages);
err_vmalloc:
err_coredump:
	/* make sure there is no AoC startup work active */
	cancel_work_sync(&prvdata->online_work);

	mutex_lock(&aoc_service_lock);
	aoc_take_offline(prvdata);
	restart_rc = aoc_watchdog_restart(prvdata);
	if (restart_rc)
		dev_info(prvdata->dev, "aoc subsystem restart failed: rc = %d\n", restart_rc);
	else
		dev_info(prvdata->dev, "aoc subsystem restart succeeded\n");

	mutex_unlock(&aoc_service_lock);
}

void aoc_trigger_watchdog(const char *reason)
{
	struct aoc_prvdata *prvdata;

	if (!aoc_platform_device)
		return;

	prvdata = platform_get_drvdata(aoc_platform_device);
	if (!prvdata)
		return;

	if (work_busy(&prvdata->watchdog_work))
		return;

	reset_store(prvdata->dev, NULL, reason, strlen(reason));
}
EXPORT_SYMBOL_GPL(aoc_trigger_watchdog);
#endif

static struct dma_heap *aoc_create_dma_buf_heap(struct aoc_prvdata *prvdata, const char *name,
						phys_addr_t base, size_t size)
{
	struct device *dev = prvdata->dev;
	size_t align = SZ_16K;
	struct dma_heap *heap;

	heap = ion_physical_heap_create(base, size, align, name, aoc_pheap_alloc_cb,
					aoc_pheap_free_cb, dev);
	if (IS_ERR(heap))
		dev_err(dev, "heap \"%s\" creation failure: %ld\n", name, PTR_ERR(heap));

	return heap;
}

static bool aoc_create_dma_buf_heaps(struct aoc_prvdata *prvdata)
{
	phys_addr_t base = prvdata->dram_resource.start + resource_size(&prvdata->dram_resource);

	base -= SENSOR_DIRECT_HEAP_SIZE;
	prvdata->sensor_heap = aoc_create_dma_buf_heap(prvdata, "sensor_direct_heap",
						       base, SENSOR_DIRECT_HEAP_SIZE);
	prvdata->sensor_heap_base = base;
	if (IS_ERR(prvdata->sensor_heap))
		return false;

	base -= PLAYBACK_HEAP_SIZE;
	prvdata->audio_playback_heap = aoc_create_dma_buf_heap(prvdata, "aaudio_playback_heap",
							       base, PLAYBACK_HEAP_SIZE);
	prvdata->audio_playback_heap_base = base;
	if (IS_ERR(prvdata->audio_playback_heap))
		return false;

	base -= CAPTURE_HEAP_SIZE;
	prvdata->audio_capture_heap = aoc_create_dma_buf_heap(prvdata, "aaudio_capture_heap",
							      base, CAPTURE_HEAP_SIZE);
	prvdata->audio_capture_heap_base = base;
	if (IS_ERR(prvdata->audio_capture_heap))
		return false;

	return true;
}

static int aoc_open(struct inode *inode, struct file *file)
{
	struct aoc_prvdata *prvdata = container_of(inode->i_cdev,
					struct aoc_prvdata, cdev);

	file->private_data = prvdata;
	return 0;
}

static long aoc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dma_buf *dmabuf;
	struct aoc_prvdata *prvdata = file->private_data;
	struct samsung_dma_buffer *dma_heap_buf;
	long ret = -EINVAL;

	switch (cmd) {
	case AOC_IOCTL_ION_FD_TO_HANDLE:
	{
		struct aoc_ion_handle handle;

		BUILD_BUG_ON(sizeof(struct aoc_ion_handle) !=
			     _IOC_SIZE(AOC_IOCTL_ION_FD_TO_HANDLE));

		if (copy_from_user(&handle, (struct aoc_ion_handle *)arg, _IOC_SIZE(cmd)))
			break;

		dmabuf = dma_buf_get(handle.fd);
		if (IS_ERR(dmabuf)) {
			pr_err("fd is not an ion buffer\n");
			ret = PTR_ERR(dmabuf);
			break;
		}

		dma_heap_buf = dmabuf->priv;
		handle.handle = (u64)dma_heap_buf->priv;

		dma_buf_put(dmabuf);

		if (!copy_to_user((struct aoc_ion_handle *)arg, &handle, _IOC_SIZE(cmd)))
			ret = 0;
	}
	break;

	case AOC_IOCTL_DISABLE_MM:
	{
		u32 disable_mm;

		BUILD_BUG_ON(sizeof(disable_mm) != _IOC_SIZE(AOC_IOCTL_DISABLE_MM));

		if (copy_from_user(&disable_mm, (u32 *)arg, _IOC_SIZE(cmd)))
			break;

		prvdata->disable_monitor_mode = disable_mm;
		if (prvdata->disable_monitor_mode != 0)
			pr_info("AoC Monitor Mode disabled\n");

		ret = 0;
	}
	break;

	case AOC_IOCTL_DISABLE_AP_RESETS:
	{
		u32 disable_ap_resets;

		BUILD_BUG_ON(sizeof(disable_ap_resets) != _IOC_SIZE(AOC_IOCTL_DISABLE_AP_RESETS));

		if (copy_from_user(&disable_ap_resets, (u32 *)arg, _IOC_SIZE(cmd)))
			break;

		prvdata->no_ap_resets = disable_ap_resets;
		if (prvdata->no_ap_resets != 0)
			pr_info("AoC AP side resets disabled\n");

		ret = 0;
	}
	break;

	case AOC_IOCTL_FORCE_VNOM:
	{
		u32 force_vnom;

		BUILD_BUG_ON(sizeof(force_vnom) != _IOC_SIZE(AOC_IOCTL_FORCE_VNOM));

		if (copy_from_user(&force_vnom, (u32 *)arg, _IOC_SIZE(cmd)))
			break;

		prvdata->force_voltage_nominal = force_vnom;
		if (prvdata->force_voltage_nominal != 0)
			pr_info("AoC Force Nominal Voltage enabled\n");

		ret = 0;
	}
	break;

	case AOC_IOCTL_ENABLE_UART_TX:
	{
		u32 enable_uart;

		BUILD_BUG_ON(sizeof(enable_uart) != _IOC_SIZE(AOC_IOCTL_ENABLE_UART_TX));

		if (copy_from_user(&enable_uart, (u32 *)arg, _IOC_SIZE(cmd)))
			break;

		prvdata->enable_uart_tx = enable_uart;
		if (prvdata->enable_uart_tx != 0)
			pr_info("AoC UART Logging Enabled\n");

		ret = 0;
	}
	break;

	case AOC_IOCTL_FORCE_SPEAKER_ULTRASONIC:
	{
		u32 force_sprk_ultrasonic;

		BUILD_BUG_ON(sizeof(force_sprk_ultrasonic) != _IOC_SIZE(AOC_IOCTL_FORCE_SPEAKER_ULTRASONIC));

		if (copy_from_user(&force_sprk_ultrasonic, (u32 *)arg, _IOC_SIZE(cmd)))
			break;

		prvdata->force_speaker_ultrasonic = force_sprk_ultrasonic;
		if (prvdata->force_speaker_ultrasonic != 0)
			pr_info("AoC Forcefully enabling Speaker Ultrasonic pipeline\n");

		ret = 0;
	}
	break;

	case AOC_IS_ONLINE:
		{
			int online = (aoc_state == AOC_STATE_ONLINE);
			if (!copy_to_user((int *)arg, &online, _IOC_SIZE(cmd)))
				ret = 0;
		}
	break;

	default:
		/* ioctl(2) The specified request does not apply to the kind of object
		 * that the file descriptor fd references
		 */
		pr_err("Received IOCTL with invalid ID (%d) returning ENOTTY", cmd);
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static int aoc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations aoc_fops = {
	.open = aoc_open,
	.release = aoc_release,
	.unlocked_ioctl = aoc_unlocked_ioctl,

	.owner = THIS_MODULE,
};

static char *aoc_devnode(struct device *dev, umode_t *mode)
{
	if (!mode || !dev)
		return NULL;

	if (MAJOR(dev->devt) == aoc_major)
		*mode = 0666;

	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}

static int init_chardev(struct aoc_prvdata *prvdata)
{
	int rc;

	cdev_init(&prvdata->cdev, &aoc_fops);
	prvdata->cdev.owner = THIS_MODULE;
	rc = alloc_chrdev_region(&prvdata->aoc_devt, 0, AOC_MAX_MINOR, AOC_CHARDEV_NAME);
	if (rc != 0) {
		pr_err("Failed to alloc chrdev region\n");
		goto err;
	}

	rc = cdev_add(&prvdata->cdev, prvdata->aoc_devt, AOC_MAX_MINOR);
	if (rc) {
		pr_err("Failed to register chrdev\n");
		goto err_cdev_add;
	}

	aoc_major = MAJOR(prvdata->aoc_devt);

	prvdata->_class = class_create(THIS_MODULE, AOC_CHARDEV_NAME);
	if (!prvdata->_class) {
		pr_err("failed to create aoc_class\n");
		rc = -ENXIO;
		goto err_class_create;
	}

	prvdata->_class->devnode = aoc_devnode;

	prvdata->_device = device_create(prvdata->_class, NULL,
					 MKDEV(aoc_major, 0),
					 NULL, AOC_CHARDEV_NAME);
	if (!prvdata->_device) {
		pr_err("failed to create aoc_device\n");
		rc = -ENXIO;
		goto err_device_create;
	}

	return rc;

err_device_create:
	class_destroy(prvdata->_class);
err_class_create:
	cdev_del(&prvdata->cdev);
err_cdev_add:
	unregister_chrdev_region(prvdata->aoc_devt, 1);
err:
	return rc;
}

static void deinit_chardev(struct aoc_prvdata *prvdata)
{
	if (!prvdata)
		return;

	device_destroy(prvdata->_class, prvdata->aoc_devt);
	class_destroy(prvdata->_class);
	cdev_del(&prvdata->cdev);

	unregister_chrdev_region(prvdata->aoc_devt, AOC_MAX_MINOR);
}

static void aoc_cleanup_resources(struct platform_device *pdev)
{
	struct aoc_prvdata *prvdata = platform_get_drvdata(pdev);

	pr_notice("cleaning up resources\n");

	if (prvdata) {
		aoc_take_offline(prvdata);
		free_mailbox_channels(prvdata);

		if (prvdata->domain) {
			aoc_clear_sysmmu(prvdata);
			prvdata->domain = NULL;
		}

#ifdef AOC_JUNO
		free_irq(aoc_irq, prvdata->dev);
		aoc_irq = -1;
#endif
	}

}

static void release_gsa_device(void *prv)
{
	struct aoc_prvdata *prvdata = prv;

	put_device(prvdata->gsa_dev);
}

static int find_gsa_device(struct aoc_prvdata *prvdata)
{
	struct device_node *np;
	struct platform_device *gsa_pdev;

	np = of_parse_phandle(prvdata->dev->of_node, "gsa-device", 0);
	if (!np) {
		dev_err(prvdata->dev,
			"gsa-device phandle not found in AOC device tree node\n");
		return -ENODEV;
	}
	gsa_pdev = of_find_device_by_node(np);
	of_node_put(np);

	if (!gsa_pdev) {
		dev_err(prvdata->dev,
			"gsa-device phandle doesn't refer to a device\n");
		return -ENODEV;
	}
	prvdata->gsa_dev = &gsa_pdev->dev;
	return devm_add_action_or_reset(prvdata->dev, release_gsa_device,
					prvdata);
}

static int aoc_core_suspend(struct device *dev)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	size_t total_services = aoc_num_services();
	int i = 0;

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE || work_busy(&prvdata->watchdog_work))
		goto exit;

	for (i = 0; i < total_services; i++) {
		struct aoc_service_dev *s = service_dev_at_index(prvdata, i);

		if (s && s->wake_capable)
			s->suspend_rx_count = aoc_service_slots_available_to_read(s->service,
										  AOC_UP);
	}

exit:
	atomic_dec(&prvdata->aoc_process_active);
	return 0;
}

static int aoc_core_resume(struct device *dev)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	size_t total_services = aoc_num_services();
	int i = 0;

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE || work_busy(&prvdata->watchdog_work))
		goto exit;

	for (i = 0; i < total_services; i++) {
		struct aoc_service_dev *s = service_dev_at_index(prvdata, i);

		if (s && s->wake_capable) {
			size_t available = aoc_service_slots_available_to_read(s->service, AOC_UP);

			if (available != s->suspend_rx_count)
				dev_notice(dev, "Service \"%s\" has %zu messages to read on wake\n",
					   dev_name(&s->dev), available);
		}
	}

exit:
	atomic_dec(&prvdata->aoc_process_active);
	return 0;
}

static int aoc_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aoc_prvdata *prvdata = NULL;
	struct device_node *aoc_node, *mem_node, *sysmmu_node;
	struct resource *rsrc;
	unsigned int acpm_async_size;
	int ret;
	int rc;
	int i;

	if (aoc_platform_device != NULL) {
		dev_err(dev,
			"already matched the AoC to another platform device");
		rc = -EEXIST;
		goto err_platform_not_null;
	}

	aoc_node = dev->of_node;
	mem_node = of_parse_phandle(aoc_node, "memory-region", 0);

	prvdata = devm_kzalloc(dev, sizeof(*prvdata), GFP_KERNEL);
	if (!prvdata) {
		rc = -ENOMEM;
		goto err_failed_prvdata_alloc;
	}
	aoc_prvdata_copy = prvdata;

	prvdata->dev = dev;
	prvdata->disable_monitor_mode = 0;
	prvdata->enable_uart_tx = 0;
	prvdata->force_voltage_nominal = 0;
	prvdata->no_ap_resets = 0;
	prvdata->reset_hysteresis_trigger_ms = 10000;
	prvdata->last_reset_time_ns = ktime_get_real_ns();
	prvdata->reset_wait_time_index = 0;

	rc = find_gsa_device(prvdata);
	if (rc) {
		dev_err(dev, "Failed to initialize gsa device: %d\n", rc);
	}

	ret = init_chardev(prvdata);
	if (ret) {
		dev_err(dev, "Failed to initialize chardev: %d\n", ret);
		rc = -ENOMEM;
		goto err_chardev;
	}

	if (!mem_node) {
		dev_err(dev,
			"failed to find reserve-memory in the device tree\n");
		rc = -EINVAL;
		goto err_memnode;
	}

	aoc_sram_resource =
		platform_get_resource_byname(pdev, IORESOURCE_MEM, "blk_aoc");

	ret = of_address_to_resource(mem_node, 0, &prvdata->dram_resource);
	of_node_put(mem_node);

	if (!aoc_sram_resource || ret != 0) {
		dev_err(dev,
			"failed to get memory resources for device sram %pR dram %pR\n",
			aoc_sram_resource, &prvdata->dram_resource);
		rc = -ENOMEM;
		goto err_mem_resources;
	}

#ifdef AOC_JUNO
	aoc_irq = platform_get_irq(pdev, 0);
	if (aoc_irq < 1) {
		dev_err(dev, "failed to configure aoc interrupt\n");
		rc = aoc_irq;
		goto err_get_irq;
	}
#else
	for (i = 0; i < ARRAY_SIZE(prvdata->mbox_channels); i++) {
		prvdata->mbox_channels[i].client.dev = dev;
		prvdata->mbox_channels[i].client.tx_block = false;
		prvdata->mbox_channels[i].client.tx_tout = 100; /* 100ms timeout for tx */
		prvdata->mbox_channels[i].client.knows_txdone = false;
		prvdata->mbox_channels[i].client.rx_callback = aoc_mbox_rx_callback;
		prvdata->mbox_channels[i].client.tx_done = aoc_mbox_tx_done;
		prvdata->mbox_channels[i].client.tx_prepare = aoc_mbox_tx_prepare;

		prvdata->mbox_channels[i].prvdata = prvdata;
		prvdata->mbox_channels[i].index = i;
	}


	strscpy(prvdata->firmware_name, default_firmware,
		sizeof(prvdata->firmware_name));

	platform_set_drvdata(pdev, prvdata);

	ret = allocate_mailbox_channels(prvdata);
	if (ret) {
		dev_err(dev, "failed to allocate mailbox channels %d\n", ret);
		rc = -ENOMEM;
		goto err_mem_resources;
	}

	init_waitqueue_head(&prvdata->aoc_reset_wait_queue);
	INIT_WORK(&prvdata->watchdog_work, aoc_watchdog);

	prvdata->watchdog_irq = platform_get_irq_byname(pdev, "watchdog");
	if (prvdata->watchdog_irq < 0) {
		dev_err(dev, "failed to find watchdog irq\n");
		rc = -EIO;
		goto err_watchdog_irq_get;
	}

	irq_set_status_flags(prvdata->watchdog_irq, IRQ_NOAUTOEN);
	ret = devm_request_irq(dev, prvdata->watchdog_irq, watchdog_int_handler,
			       IRQF_TRIGGER_HIGH, dev_name(dev), dev);
	if (ret != 0) {
		dev_err(dev, "failed to register watchdog irq handler: %d\n",
			ret);
		rc = -EIO;
		goto err_watchdog_irq_req;
	}

	sysmmu_node = of_parse_phandle(aoc_node, "iommus", 0);
	if (!sysmmu_node) {
		dev_err(dev, "failed to find sysmmu device tree node\n");
		rc = -ENODEV;
		goto err_watchdog_sysmmu_irq;
	}
	ret = of_irq_get(sysmmu_node, 0);
	if (ret < 0) {
		dev_err(dev, "failed to find sysmmu non-secure irq: %d\n", ret);
		rc = ret;
		goto err_watchdog_sysmmu_irq;
	}
	prvdata->sysmmu_nonsecure_irq = ret;
	ret = of_irq_get(sysmmu_node, 1);
	if (ret < 0) {
		dev_err(dev, "failed to find sysmmu secure irq: %d\n", ret);
		rc = ret;
		goto err_watchdog_sysmmu_irq;
	}
	prvdata->sysmmu_secure_irq = ret;
	of_node_put(sysmmu_node);
#endif

	pr_notice("found aoc with interrupt:%d sram:%pR dram:%pR\n", aoc_irq,
		  aoc_sram_resource, &prvdata->dram_resource);
	aoc_platform_device = pdev;

	aoc_sram_virt_mapping = devm_ioremap_resource(dev, aoc_sram_resource);

	prvdata->dram_size = resource_size(&prvdata->dram_resource);
	if (!devm_request_mem_region(dev, prvdata->dram_resource.start, prvdata->dram_size, dev_name(dev))) {
		dev_err(dev, "Failed to claim dram resource %pR\n", &prvdata->dram_resource);
		rc = -EIO;
		goto err_sram_dram_map;
	}

	aoc_dram_virt_mapping = devm_ioremap_wc(dev, prvdata->dram_resource.start, prvdata->dram_size);

	/* Change to devm_platform_ioremap_resource_byname when available */
	rsrc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "aoc_req");
	if (rsrc) {
		prvdata->aoc_req_virt = devm_ioremap_resource(dev, rsrc);
		prvdata->aoc_req_size = resource_size(rsrc);

		if (IS_ERR(prvdata->aoc_req_virt)) {
			dev_err(dev, "failed to map aoc_req region at %pR\n",
				rsrc);
			prvdata->aoc_req_virt = NULL;
			prvdata->aoc_req_size = 0;
		} else {
			dev_dbg(dev, "found aoc_req at %pR\n", rsrc);
		}
	}

	prvdata->sram_virt = aoc_sram_virt_mapping;
	prvdata->sram_size = resource_size(aoc_sram_resource);

	prvdata->dram_virt = aoc_dram_virt_mapping;

	if (IS_ERR(aoc_sram_virt_mapping) || IS_ERR(aoc_dram_virt_mapping)) {
		rc = -ENOMEM;
		goto err_sram_dram_map;
	}

#ifndef AOC_JUNO
	prvdata->aoc_s2mpu_virt = devm_platform_ioremap_resource_byname(pdev, "aoc_s2mpu");
	if (IS_ERR(prvdata->aoc_s2mpu_virt)) {
		dev_err(dev, "failed to map aoc_s2mpu: rc = %ld\n",
			PTR_ERR(prvdata->aoc_s2mpu_virt));
		rc = PTR_ERR(prvdata->aoc_s2mpu_virt);
		goto err_s2mpu_map;
	}
	prvdata->aoc_s2mpu_saved_value = ioread32(prvdata->aoc_s2mpu_virt + AOC_S2MPU_CTRL0);

	pm_runtime_set_active(dev);
	/* Leave AoC in suspended state. Otherwise, AoC SysMMU is set to active which results in the
	 * SysMMU driver trying to access SysMMU SFRs during device suspend/resume operations. The
	 * latter is problematic if AoC is in monitor mode and BLK_AOC is off. */
	pm_runtime_set_suspended(dev);

	prvdata->domain = iommu_get_domain_for_dev(dev);
	if (!prvdata->domain) {
		pr_err("failed to find iommu domain\n");
		rc = -EIO;
		goto err_find_iommu;
	}

	aoc_configure_ssmt(pdev);

	aoc_configure_sysmmu(prvdata);

	if (!aoc_create_dma_buf_heaps(prvdata)) {
		pr_err("Unable to create dma_buf heaps\n");
		aoc_cleanup_resources(pdev);
		return -ENOMEM;
	}
#endif

	prvdata->sensor_power_count = of_property_count_strings(dev->of_node, "sensor_power_list");
	if (prvdata->sensor_power_count > MAX_SENSOR_POWER_NUM) {
		pr_warn("sensor power count %i is larger than available number.",
			prvdata->sensor_power_count);
		prvdata->sensor_power_count = MAX_SENSOR_POWER_NUM;
	} else if (prvdata->sensor_power_count < 0) {
		pr_err("unsupported sensor power list, err = %i.", prvdata->sensor_power_count);
		prvdata->sensor_power_count = 0;
	}

	ret = of_property_read_string_array(dev->of_node, "sensor_power_list",
					    (const char**)&prvdata->sensor_power_list,
					    prvdata->sensor_power_count);

	for (i = 0; i < prvdata->sensor_power_count; i++) {
		prvdata->sensor_regulator[i] =
				devm_regulator_get_exclusive(dev, prvdata->sensor_power_list[i]);
		if (IS_ERR_OR_NULL(prvdata->sensor_regulator[i])) {
			prvdata->sensor_regulator[i] = NULL;
			pr_err("failed to get %s regulator.", prvdata->sensor_power_list[i]);
		}
	}

	reset_sensor_power(prvdata, true);

	/* Default to 6MB if we are not loading the firmware (i.e. trace32) */
	aoc_control = aoc_dram_translate(prvdata, 6 * SZ_1M);

	INIT_WORK(&prvdata->online_work, aoc_did_become_online);

	INIT_DELAYED_WORK(&prvdata->monitor_work, aoc_monitor_online);

	aoc_configure_interrupt();

#ifdef AOC_JUNO
	ret = request_irq(aoc_irq, aoc_int_handler, IRQF_TRIGGER_HIGH, "aoc",
			  prvdata->_device);
	if (ret != 0) {
		pr_err("failed to register interrupt handler : %d\n", ret);

		rc = -ENXIO;
		goto err_aoc_irq_req;
	}
#endif

	ret = acpm_ipc_request_channel(aoc_node, acpm_aoc_reset_callback,
				       &prvdata->acpm_async_id, &acpm_async_size);
	if (ret < 0) {
		dev_err(dev, "failed to register acpm aoc reset callback\n");
		rc = -EIO;
		/* goto err_acmp_reset; */
	}

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	prvdata->itmon_nb.notifier_call = aoc_itmon_notifier;
	itmon_notifier_chain_register(&prvdata->itmon_nb);
#endif

	if (aoc_autoload_firmware) {
		ret = start_firmware_load(dev);
		if (ret != 0)
			pr_err("failed to start firmware download: %d\n", ret);
	}

	ret = sysfs_create_groups(&dev->kobj, aoc_groups);

	pr_debug("platform_probe matched\n");

	return 0;

/* err_acmp_reset: */
#ifdef AOC_JUNO
err_aoc_irq_req:
#endif
#ifndef AOC_JUNO
err_find_iommu:
err_s2mpu_map:
#endif
err_sram_dram_map:

#ifndef AOC_JUNO
err_watchdog_sysmmu_irq:
err_watchdog_irq_req:
err_watchdog_irq_get:
#else
err_get_irq:
#endif
err_mem_resources:
	aoc_cleanup_resources(pdev);
err_memnode:
	deinit_chardev(prvdata);
err_chardev:
	kfree(prvdata);
err_failed_prvdata_alloc:
err_platform_not_null:
	return rc;
}

static int aoc_platform_remove(struct platform_device *pdev)
{
	struct aoc_prvdata *prvdata;
	int i;

	pr_debug("platform_remove\n");

	prvdata = platform_get_drvdata(pdev);
	acpm_ipc_release_channel(pdev->dev.of_node, prvdata->acpm_async_id);
	for (i = 0; i < prvdata->sensor_power_count; i++) {
		if (prvdata->sensor_regulator[i]) {
			regulator_put(prvdata->sensor_regulator[i]);
		}
	}
	sysfs_remove_groups(&pdev->dev.kobj, aoc_groups);

	aoc_cleanup_resources(pdev);
	deinit_chardev(prvdata);
	platform_set_drvdata(pdev, NULL);
	aoc_platform_device = NULL;

	return 0;
}

static void sscd_release(struct device *dev)
{
}

static void aoc_platform_shutdown(struct platform_device *pdev)
{
	struct aoc_prvdata *prvdata = platform_get_drvdata(pdev);

	disable_irq_nosync(prvdata->watchdog_irq);
	aoc_take_offline(prvdata);
}

/* Module methods */
static int __init aoc_init(void)
{
	pr_debug("system driver init\n");

	if (bus_register(&aoc_bus_type) != 0) {
		pr_err("failed to register AoC bus\n");
		goto err_aoc_bus;
	}

	if (platform_driver_register(&aoc_driver) != 0) {
		pr_err("failed to register platform driver\n");
		goto err_aoc_driver;
	}

	if (platform_device_register(&sscd_dev) != 0) {
		pr_err("failed to register AoC coredump device\n");
		goto err_aoc_coredump;
	}

	return 0;

err_aoc_coredump:
	platform_driver_unregister(&aoc_driver);
err_aoc_driver:
	bus_unregister(&aoc_bus_type);
err_aoc_bus:
	return -ENODEV;
}

static void __exit aoc_exit(void)
{
	pr_debug("system driver exit\n");

	platform_driver_unregister(&aoc_driver);

	bus_unregister(&aoc_bus_type);
}

module_init(aoc_init);
module_exit(aoc_exit);

MODULE_LICENSE("GPL v2");
