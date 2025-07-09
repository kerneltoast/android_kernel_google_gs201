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
#include <linux/dma-map-ops.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/glob.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_data/sscoredump.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
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
#include "ion_physical_heap.h"

#include "aoc_firmware.h"
#include "aoc_ramdump_regions.h"

#define AOC_MAX_MINOR (1U)

#define AOC_FWDATA_ENTRIES 10
#define AOC_FWDATA_BOARDID_DFL  0x20202
#define AOC_FWDATA_BOARDREV_DFL 0x10000

#define MAX_RESET_REASON_STRING_LEN 128UL

#define RESET_WAIT_TIMES_NUM 3
#define RESET_WAIT_TIME_MS 3000
#define RESET_WAIT_TIME_INCREMENT_MS  2048

DEFINE_MUTEX(aoc_service_lock);

enum AOC_FW_STATE aoc_state;

struct platform_device *aoc_platform_device;

/* TODO: Reduce the global variables (move into a driver structure) */
/* Resources found from the device tree */
struct resource *aoc_sram_resource;

struct sscd_info {
	char *name;
	struct sscd_segment segs[256];
	u16 seg_count;
};

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

struct aoc_control_block *aoc_control;

static int aoc_major;

static const char *default_firmware = "aoc.bin";
static bool aoc_autoload_firmware = false;
module_param(aoc_autoload_firmware, bool, 0644);
MODULE_PARM_DESC(aoc_autoload_firmware, "Automatically load firmware if true");

static bool aoc_disable_restart = false;
module_param(aoc_disable_restart, bool, 0644);
MODULE_PARM_DESC(aoc_disable_restart, "Prevent AoC from restarting after crashing.");

static bool aoc_panic_on_req_timeout = true;
module_param(aoc_panic_on_req_timeout, bool, 0644);
MODULE_PARM_DESC(aoc_panic_on_req_timeout, "Enable kernel panic when aoc_req times out.");

static struct aoc_module_parameters aoc_module_params = {
	.aoc_autoload_firmware = &aoc_autoload_firmware,
	.aoc_disable_restart = &aoc_disable_restart,
	.aoc_panic_on_req_timeout = &aoc_panic_on_req_timeout,
};

static int aoc_core_suspend(struct device *dev);
static int aoc_core_resume(struct device *dev);

const static struct dev_pm_ops aoc_core_pm_ops = {
	.suspend = aoc_core_suspend,
	.resume = aoc_core_resume,
};

static int aoc_bus_match(struct device *dev, struct device_driver *drv);
static int aoc_bus_probe(struct device *dev);
static void aoc_bus_remove(struct device *dev);

static void aoc_configure_iommu_fault_handler(struct aoc_prvdata *p);
static void aoc_configure_iommu(struct aoc_prvdata *p, const struct firmware *fw);

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

static bool write_reset_trampoline(const struct firmware *fw);
static bool configure_dmic_regulator(struct aoc_prvdata *prvdata, bool enable);
static bool configure_sensor_regulator(struct aoc_prvdata *prvdata, bool enable);

static void aoc_take_offline(struct aoc_prvdata *prvdata);

static void aoc_process_services(struct aoc_prvdata *prvdata, int offset);

static void aoc_watchdog(struct work_struct *work);

void *aoc_sram_translate(u32 offset)
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

bool aoc_is_valid_dram_address(struct aoc_prvdata *prv, void *addr)
{
	ptrdiff_t offset;

	if (addr < prv->dram_virt)
		return false;

	offset = addr - prv->dram_virt;
	return (offset < prv->dram_size);
}

phys_addr_t aoc_dram_translate_to_aoc(struct aoc_prvdata *p,
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

bool aoc_fw_ready(void)
{
	return aoc_control != NULL && aoc_control->magic == AOC_MAGIC;
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

	for (i = 0; i < prv->aoc_mbox_channels; i++) {
		slot = &prv->mbox_channels[i];
		slot->channel = mbox_request_channel(&slot->client, i);
		if (IS_ERR(slot->channel)) {
			rc = PTR_ERR(slot->channel);
			dev_err(dev, "failed to find mailbox interface %d : %d\n", i, rc);
			slot->channel = NULL;
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

	for (i = 0; i < prv->aoc_mbox_channels; i++) {
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

extern int gs_chipid_get_ap_hw_tune_array(const u8 **array);

static inline bool aoc_sram_was_repaired(struct aoc_prvdata *prvdata) { return false; }

struct aoc_fw_data {
	u32 key;
	u32 value;
};

u32 dt_property(struct device_node *node, const char *key)
{
	u32 ret;

	if (of_property_read_u32(node, key, &ret))
		return DT_PROPERTY_NOT_FOUND;

	return ret;
}
EXPORT_SYMBOL_GPL(dt_property);

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
	static bool first_load_prevented = false;
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
	u32 volte_release_mif = prvdata->volte_release_mif;
	u32 board_id  = AOC_FWDATA_BOARDID_DFL;
	u32 board_rev = AOC_FWDATA_BOARDREV_DFL;
	u32 rand_seed = get_random_u32();
	u32 chip_revision = gs_chipid_get_revision();
	u32 chip_type = gs_chipid_get_type();
	u32 chip_product_id = gs_chipid_get_product_id();
	u32 dt_gnss_type = dt_property(prvdata->dev->of_node, "gnss-type");
	u32 gnss_type = dt_gnss_type == DT_PROPERTY_NOT_FOUND ? 0 : dt_gnss_type;
	u32 dt_wifi_chip = dt_property(prvdata->dev->of_node, "wifi-chip");
	u32 wifi_chip = dt_wifi_chip == DT_PROPERTY_NOT_FOUND ? 0 : dt_wifi_chip;
	bool dt_prevent_aoc_load = (dt_property(prvdata->dev->of_node, "prevent-fw-load")==1);
	phys_addr_t sensor_heap = aoc_dram_translate_to_aoc(prvdata, prvdata->sensor_heap_base);
	phys_addr_t playback_heap = aoc_dram_translate_to_aoc(prvdata, prvdata->audio_playback_heap_base);
	phys_addr_t capture_heap = aoc_dram_translate_to_aoc(prvdata, prvdata->audio_capture_heap_base);
	unsigned int i;
	bool fw_signed, gsa_enabled;

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
		{ .key = kAOCForceSpeakerUltrasonic, .value = force_speaker_ultrasonic },
		{ .key = kAOCRandSeed, .value = rand_seed },
		{ .key = kAOCChipRevision, .value = chip_revision },
		{ .key = kAOCChipType, .value = chip_type },
		{ .key = kAOCGnssType, .value = gnss_type },
		{ .key = kAOCVolteReleaseMif, .value = volte_release_mif },
		{ .key = kAOCChipProductId, .value = chip_product_id },
		{ .key = kAOCWifiChip, .value = wifi_chip },
	};

	const char *version;
	u32 fw_data_entries = ARRAY_SIZE(fw_data);
	u32 ipc_offset;

	if ((dt_prevent_aoc_load) && (!first_load_prevented)) {
		dev_err(dev, "DTS settings prevented AoC firmware from being loaded\n");
		first_load_prevented = true;
		return;
	}

	aoc_board_config_parse(prvdata->dev->of_node, &board_id, &board_rev);

	if (!fw) {
		dev_err(dev, "Failed to load AoC firmware image\n");
		return;
	}

	if (prvdata->force_release_aoc) {
		dev_info(dev, "Force Reload Trigger: Free current loaded\n");
		goto free_fw;
	}

	for (i = 0; i < fw_data_entries; i++) {
		if (fw_data[i].key == kAOCBoardID)
			fw_data[i].value = board_id;
		else if (fw_data[i].key == kAOCBoardRevision)
			fw_data[i].value = board_rev;
	}

	request_aoc_on(prvdata, true);

	if (!fw->data) {
		dev_err(dev, "firmware image contains no data\n");
		goto free_fw;
	}

	if (!_aoc_fw_is_valid(fw)) {
		dev_err(dev, "firmware validation failed\n");
		goto free_fw;
	}

	ipc_offset = _aoc_fw_ipc_offset(fw);
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
	if (!fw_signed) {
		dev_err(dev, "Loading unsigned aoc image is unsupported\n");
		goto free_fw;
	}

	dev_info(dev, "Loading signed aoc image\n");

	prvdata->protected_by_gsa = fw_signed;

	aoc_control = aoc_dram_translate(prvdata, ipc_offset);

	{
		bool commit_rc = _aoc_fw_commit(fw, aoc_dram_virt_mapping + AOC_BINARY_DRAM_OFFSET);
		if (!commit_rc) {
			dev_err(dev, "FW commit failed!\n");
		}
	}

	gsa_enabled = of_property_read_bool(prvdata->dev->of_node, "gsa-enabled");

	if (gsa_enabled) {
		int rc;

		aoc_configure_iommu_fault_handler(prvdata);
		rc = aoc_fw_authenticate(prvdata, fw);
		if (rc) {
			dev_err(dev, "GSA: FW authentication failed: %d\n", rc);
			goto free_fw;
		}
	} else {
		aoc_configure_iommu(prvdata, fw);
		write_reset_trampoline(fw);
	}

	aoc_pass_fw_information(aoc_dram_translate(prvdata, ipc_offset),
			fw_data, ARRAY_SIZE(fw_data));

	aoc_state = AOC_STATE_FIRMWARE_LOADED;

	/* AOC needs DRAM while booting, so prevent AP from sleep. */
	dev_info(dev, "preventing AP from sleep for 2 sec for aoc boot\n");
	disable_power_mode(0, POWERMODE_TYPE_SYSTEM);
	prvdata->ipc_base = aoc_dram_translate(prvdata, ipc_offset);

	/* start AOC */
	if (gsa_enabled) {
		int rc = gsa_send_aoc_cmd(prvdata->gsa_dev, GSA_AOC_START);
		if (rc < 0) {
			dev_err(dev, "GSA: Failed to start AOC: %d\n", rc);
			goto free_fw;
		}
	} else {
		aoc_release_from_reset(prvdata);
	}

	configure_crash_interrupts(prvdata, true);

	/* Monitor if there is callback from aoc after 5sec */
	cancel_delayed_work_sync(&prvdata->monitor_work);
	schedule_delayed_work(&prvdata->monitor_work,
			msecs_to_jiffies(5 * 1000));

	msleep(2000);
	dev_info(dev, "re-enabling low power mode\n");
	enable_power_mode(0, POWERMODE_TYPE_SYSTEM);

	release_firmware(fw);
	return;

free_fw:
	/* Change aoc_state to offline due to abnormal firmware */
	aoc_state = AOC_STATE_OFFLINE;
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

	pr_debug("Get heap address(phy):%pap\n", &audio_heap_base);

	return audio_heap_base;
}
EXPORT_SYMBOL_GPL(aoc_get_heap_base_phys_addr);

static bool write_reset_trampoline(const struct firmware *fw)
{
	u32 *reset, bl_size;
	u32 *bootloader;

	reset = aoc_sram_translate(0);
	if (!reset)
		return false;

	bl_size = _aoc_fw_bl_size(fw);
	bootloader = _aoc_fw_bl(fw);

	pr_notice("writing reset trampoline to addr %#x\n",
		bootloader[bl_size / sizeof(u32) - 1]);
	memcpy_toio(reset, bootloader, bl_size);

	return true;
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
	struct aoc_prvdata *prvdata = platform_get_drvdata(aoc_platform_device);

	return (sys_tick - clock_offset()) / prvdata->aoc_clock_divider;
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

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE)
		goto exit;

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
exit:
	atomic_dec(&prvdata->aoc_process_active);

	return ret;
}

static DEVICE_ATTR_RO(services);

int start_firmware_load(struct device *dev)
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

	if (aoc_state != AOC_STATE_ONLINE) {
		dev_err(dev, "Reset requested while AoC is not online");
		return -ENODEV;
	}

	strscpy(reason_str, buf, sizeof(reason_str));
	dev_err(dev, "Reset requested from userspace, reason: %s", reason_str);

	if (prvdata->no_ap_resets) {
		dev_err(dev, "Reset request rejected, option disabled via persist options");
	} else {
		trigger_aoc_ssr(true, reason_str);
	}
	return count;
}

static DEVICE_ATTR_WO(reset);

static ssize_t force_reload_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);

	/* Force release current loaded AoC if watchdog already active */
	prvdata->force_release_aoc = true;
	while (work_busy(&prvdata->watchdog_work) || work_busy(&prvdata->monitor_work.work));
	prvdata->force_release_aoc = false;

	trigger_aoc_ssr(true, "Force Reload AoC");

	return count;
}
static DEVICE_ATTR_WO(force_reload);

static ssize_t dmic_power_enable_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 10, &val) == 0) {
		dev_info(prvdata->dev,"dmic_power_enable %d", val);
		configure_dmic_regulator(prvdata, !!val);
	}
	return count;
}
static DEVICE_ATTR_WO(dmic_power_enable);

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

static ssize_t notify_timeout_aoc_status_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return 0;
}

static DEVICE_ATTR_RO(notify_timeout_aoc_status);

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
	&dev_attr_force_reload.attr,
	&dev_attr_dmic_power_enable.attr,
	&dev_attr_notify_timeout_aoc_status.attr,
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
MODULE_DEVICE_TABLE(of, aoc_match);

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

static void aoc_bus_remove(struct device *dev)
{
	struct aoc_service_dev *aoc_dev = AOC_DEVICE(dev);
	struct aoc_driver *drv = AOC_DRIVER(dev->driver);

	pr_notice("bus remove %s\n", dev_name(dev));

	if (drv->remove)
		drv->remove(aoc_dev);
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

static int aoc_wakeup_queues(struct device *dev, void *ctx)
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

	return 0;
}

static int aoc_remove_device(struct device *dev, void *ctx)
{
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
	if (!dev)
		return NULL;
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

static void aoc_configure_iommu_fault_handler(struct aoc_prvdata *p)
{
	struct device *dev = p->dev;
	int rc = iommu_register_device_fault_handler(dev, aoc_iommu_fault_handler, dev);

	if (rc)
		dev_err(dev, "iommu_register_device_fault_handler failed: rc = %d\n", rc);
}

static void aoc_configure_iommu(struct aoc_prvdata *p, const struct firmware *fw)
{
	int rc;
	size_t i, j, cnt;
	struct iommu_entry *iommu;
	struct iommu_domain *domain = p->domain;
	struct device *dev = p->dev;
	u16 iommu_offset, iommu_size;

	if (p->iommu_configured && p->iommu_config_persistent) {
		dev_info(dev, "IOMMU already configured skipping\n");
		return;
	}

	aoc_configure_iommu_fault_handler(p);

	iommu_offset = _aoc_fw_iommu_offset(fw);
	iommu_size = _aoc_fw_iommu_size(fw);
	if (!_aoc_fw_is_valid_iommu_size(fw)) {
		dev_warn(dev, "Invalid iommu table (0x%x @ 0x%x)\n", iommu_size, iommu_offset);
		return;
	}
	cnt = iommu_size / sizeof(struct iommu_entry);
	iommu = _aoc_fw_iommu_entry(fw);

	p->iommu_size = iommu_size;
	p->iommu = devm_kzalloc(dev, iommu_size, GFP_KERNEL);
	if (!p->iommu)
		return;

	memcpy(p->iommu, iommu, iommu_size);

	for (i = 0; i < cnt; i++) {
		rc = iommu_map(domain, IOMMU_VADDR(iommu[i].value),
						IOMMU_PADDR(iommu[i].value),
						IOMMU_SIZE(iommu[i].value),
						IOMMU_READ | IOMMU_WRITE);
		if (rc < 0) {
			dev_err(
				dev,
				"Failed configuring iommu: [err=%d] [index:%zu, vaddr: 0x%llx, paddr: 0x%llx, size: 0x%llx]\n",
				rc, i, IOMMU_VADDR(iommu[i].value), IOMMU_PADDR(iommu[i].value),
				IOMMU_SIZE(iommu[i].value));
			for (j = 0; j < i; j++) {
				rc = iommu_unmap(domain, IOMMU_VADDR(iommu[j].value),
						IOMMU_SIZE(iommu[j].value));
				if (rc < 0)
					dev_err(dev, "Failed unmapping iommu: %d\n", rc);
			}
			return;
		}
	}

	p->iommu_configured = true;
}

static void aoc_clear_iommu(struct aoc_prvdata *p)
{
	int rc;
	struct iommu_domain *domain = p->domain;
	size_t i, cnt;
	struct device *dev = p->dev;

	if (p->iommu != NULL) {
		cnt = p->iommu_size / sizeof(struct iommu_entry);
		for (i = 0; i < cnt; i++) {
			rc = iommu_unmap(domain, IOMMU_VADDR(p->iommu[i].value),
							IOMMU_SIZE(p->iommu[i].value));
			if (rc < 0)
				dev_err(dev, "Failed umapping iommu: %d\n", rc);
		}
	}
}

static void aoc_monitor_online(struct work_struct *work)
{
	struct aoc_prvdata *prvdata =
		container_of(work, struct aoc_prvdata, monitor_work.work);
	bool skip_reset = of_property_read_bool(prvdata->dev->of_node, "skip-monitor-online-reset");

	if (aoc_state == AOC_STATE_FIRMWARE_LOADED) {
		dev_err(prvdata->dev, "aoc init no respond, try restart\n");

		if (skip_reset)
			/* TODO: figure out if this still causes APC watchdogs on GS201 */
			return;

		trigger_aoc_ssr(true, "AOC detected not online");
	}
}

static void aoc_did_become_online(struct work_struct *work)
{
	struct aoc_prvdata *prvdata =
		container_of(work, struct aoc_prvdata, online_work);
	struct device *dev = prvdata->dev;
	int i, s, ret;

	cancel_delayed_work_sync(&prvdata->monitor_work);

	mutex_lock(&aoc_service_lock);

	s = aoc_num_services();

	request_aoc_on(prvdata, false);

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
		if (!create_service_device(prvdata, i)) {
			dev_err(prvdata->dev, "failed to create service device at index %d\n", i);
			goto err;
		}
	}

	aoc_state = AOC_STATE_ONLINE;

	for (i = 0; i < s; i++) {
		ret = device_register(&prvdata->services[i]->dev);
		if (ret)
			dev_err(dev, "failed to register service device %s err=%d\n",
				dev_name(&prvdata->services[i]->dev), ret);
	}

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

static bool configure_dmic_regulator(struct aoc_prvdata *prvdata, bool enable)
{
	bool check_enabled;
	int i;
	if (enable) {
		check_enabled = true;
		for (i = 0; i < prvdata->dmic_power_count; i++) {
			if (!prvdata->dmic_regulator[i] ||
					regulator_is_enabled(prvdata->dmic_regulator[i])) {
				continue;
			}

			if (regulator_enable(prvdata->dmic_regulator[i])) {
				pr_warn("encountered error on enabling %s.",
					prvdata->dmic_power_list[i]);
			}
			check_enabled &= regulator_is_enabled(prvdata->dmic_regulator[i]);
		}
	} else {
		check_enabled = false;

		for (i = prvdata->dmic_power_count - 1; i >= 0; i--) {
			if (!prvdata->dmic_regulator[i] ||
					!regulator_is_enabled(prvdata->dmic_regulator[i])) {
				continue;
			}

			if (regulator_disable(prvdata->dmic_regulator[i])) {
				pr_warn(" encountered error on disabling %s.",
					prvdata->dmic_power_list[i]);
			}
			check_enabled |= regulator_is_enabled(prvdata->dmic_regulator[i]);
		}
	}

	return (check_enabled == enable);
}

static void aoc_parse_dmic_power(struct aoc_prvdata *prvdata, struct device_node *node)
{
	int i;
	prvdata->dmic_power_count = of_property_count_strings(node, "dmic_power_list");
	if (prvdata->dmic_power_count > MAX_DMIC_POWER_NUM) {
		pr_warn("dmic power count %i is larger than available number.",
			prvdata->dmic_power_count);
		prvdata->dmic_power_count = MAX_DMIC_POWER_NUM;
	} else if (prvdata->dmic_power_count < 0) {
		pr_err("unsupported dmic power list, err = %i.", prvdata->dmic_power_count);
		prvdata->dmic_power_count = 0;
		return;
	}

	of_property_read_string_array(node, "dmic_power_list",
				(const char **)&prvdata->dmic_power_list,
				prvdata->dmic_power_count);

	for (i = 0; i < prvdata->dmic_power_count; i++) {
		prvdata->dmic_regulator[i] =
			devm_regulator_get_exclusive(prvdata->dev, prvdata->dmic_power_list[i]);
		if (IS_ERR_OR_NULL(prvdata->dmic_regulator[i])) {
			prvdata->dmic_regulator[i] = NULL;
			pr_err("failed to get %s regulator.", prvdata->dmic_power_list[i]);
		}
	}
}

void reset_sensor_power(struct aoc_prvdata *prvdata, bool is_init)
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
	if (aoc_state == AOC_STATE_ONLINE || aoc_state == AOC_STATE_SSR) {
		pr_notice("taking aoc offline\n");
		aoc_state = AOC_STATE_OFFLINE;

		/* wait until aoc_process or service write/read finish */
		while (!!atomic_read(&prvdata->aoc_process_active)) {
			bus_for_each_dev(&aoc_bus_type, NULL, NULL, aoc_wakeup_queues);
		}

		bus_for_each_dev(&aoc_bus_type, NULL, NULL, aoc_remove_device);

		if (aoc_control)
			aoc_control->magic = 0;

		if (prvdata->services) {
			devm_kfree(prvdata->dev, prvdata->services);
			prvdata->services = NULL;
			prvdata->total_services = 0;
		}

		/* wakeup AOC before calling GSA */
		request_aoc_on(prvdata, true);
		rc = wait_for_aoc_status(prvdata, true);
		if (rc) {
			dev_err(prvdata->dev, "timed out waiting for aoc_ack\n");
			if (prvdata->protected_by_gsa)
				dev_err(prvdata->dev, "skipping GSA commands");
			notify_timeout_aoc_status();
			return;
		}
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

	if (aoc_state != AOC_STATE_ONLINE)
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

void notify_timeout_aoc_status(void)
{
	if (aoc_platform_device == NULL) {
		pr_err("AOC platform device is undefined, can't notify aocd\n");
		return;
	}
	sysfs_notify(&aoc_platform_device->dev.kobj, NULL,
		"notify_timeout_aoc_status");
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

void trigger_aoc_ssr(bool ap_triggered_reset, char *reset_reason) {
	struct aoc_prvdata *prvdata = platform_get_drvdata(aoc_platform_device);
	if (!mutex_trylock(&aoc_service_lock)) {
		return;
	} else {
		bool aoc_in_ssr = aoc_state == AOC_STATE_SSR;
		mutex_unlock(&aoc_service_lock);
		if (aoc_in_ssr) {
			dev_err(prvdata->dev, "Reset request rejected, AOC already in SSR\n");
		} else {
			configure_crash_interrupts(prvdata, false);
			if (ap_triggered_reset) {
				strlcpy(prvdata->ap_reset_reason, reset_reason,
					AP_RESET_REASON_LENGTH);
				prvdata->ap_triggered_reset = true;
			}
			schedule_work(&prvdata->watchdog_work);
		}
	}
}

static void prepend_fw_builder_to_crash_string(struct aoc_prvdata *prvdata, char *crash_info,
									size_t max_len) {
	char *fw_builder;
	size_t prefix_size;

	if (glob_match("*[0-9]-*[a-zA-Z]", prvdata->firmware_version)) {
		fw_builder = strchrnul(prvdata->firmware_version, '-');
		fw_builder = strchrnul(fw_builder, '-');
		fw_builder++;
	} else {
		fw_builder = "Unknown";
	}

	prefix_size = strlen(fw_builder) + 3;
	memmove(crash_info + prefix_size, crash_info, max_len - prefix_size);
	crash_info[0] = '[';
	strcpy(&crash_info[1], fw_builder);
	crash_info[prefix_size - 2] = ']';
	crash_info[prefix_size - 1] = ' ';
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
	bool skip_carveout_map = of_property_read_bool(prvdata->dev->of_node, "skip-carveout-map");
	size_t num_pages;
	struct page **dram_pages = NULL;
	void *dram_cached = NULL;
	int sscd_retries = 20;
	const int sscd_retry_ms = 1000;
	int sscd_rc;
	char crash_info[RAMDUMP_SECTION_CRASH_INFO_SIZE];
	int restart_rc;
	bool ap_triggered_reset, valid_magic;
	struct aoc_section_header *crash_info_section;

	/* If we're already in SSR state, do nothing. */
	mutex_lock(&aoc_service_lock);
	if (aoc_state == AOC_STATE_SSR) {
		mutex_unlock(&aoc_service_lock);
		return;
	} else {
		aoc_state = AOC_STATE_SSR;
		mutex_unlock(&aoc_service_lock);
	}

	prvdata->total_restarts++;

	ap_triggered_reset = prvdata->ap_triggered_reset;
	prvdata->ap_triggered_reset = false;

	/* Initialize crash_info[0] to identify if it has changed later in the function. */
	crash_info[0] = 0;

	if (ap_triggered_reset) {
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

	if (ap_triggered_reset) {
		dev_info(prvdata->dev, "AP triggered reset, reason: [%s]",
			prvdata->ap_reset_reason);
		trigger_aoc_ramdump(prvdata);
	}

	ramdump_timeout = jiffies + (5 * HZ);
	while (time_before(jiffies, ramdump_timeout)) {
		valid_magic = memcmp(ramdump_header, RAMDUMP_MAGIC, sizeof(RAMDUMP_MAGIC)) == 0;
		if (ramdump_header->valid == 1 && valid_magic)
			break;
		msleep(100);
	}

	crash_info_section = &ramdump_header->sections[RAMDUMP_SECTION_CRASH_INFO_INDEX];
	if (crash_info_section->type != SECTION_TYPE_CRASH_INFO)
		crash_info_section = NULL;

	if (!(ramdump_header->valid == 1) || !valid_magic) {
		if (!(ramdump_header->valid == 1))
			dev_info(prvdata->dev, "aoc coredump timed out, coredump only contains DRAM\n");
		if (!valid_magic)
			dev_info(prvdata->dev, "aoc coredump has invalid magic\n");

		if (crash_info_section) {
			const char *crash_reason = (const char *)ramdump_header +
				crash_info_section->offset;
			bool crash_reason_valid = crash_reason < (char *)prvdata->dram_virt +
				prvdata->dram_size && crash_reason[0] != 0;

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

	if (ramdump_header->valid == 1 && valid_magic) {
		if (crash_info_section && crash_info_section->flags & RAMDUMP_FLAG_VALID) {
			const char *crash_reason = (const char *)ramdump_header +
				crash_info_section->offset;
			dev_info(prvdata->dev,
				"aoc coredump has valid coredump header, crash reason [%s]", crash_reason);
			strscpy(crash_info, crash_reason, sizeof(crash_info));
		} else {
			dev_info(prvdata->dev,
				"aoc coredump has valid coredump header, but invalid crash reason");
			strscpy(crash_info, "AoC Watchdog : invalid crash info",
				sizeof(crash_info));
		}
	}

	if (!skip_carveout_map) {
		/* In some cases, we don't map AoC carveout as cached due to b/240786634 */
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
		sscd_info.segs[0].addr = dram_cached;
	} else {
		sscd_info.segs[0].addr = prvdata->dram_virt;
	}

	if (ap_triggered_reset) {
		/* Prefer the user specified reason */
		scnprintf(crash_info, sizeof(crash_info), "AP Reset: %s", prvdata->ap_reset_reason);
	}

	if (crash_info[0] == 0)
		strscpy(crash_info, "AoC Watchdog: empty crash info string", sizeof(crash_info));

	prepend_fw_builder_to_crash_string(prvdata, crash_info,
			RAMDUMP_SECTION_CRASH_INFO_SIZE);

	dev_info(prvdata->dev, "aoc crash info: [%s]", crash_info);

	/* TODO(siqilin): Get paddr and vaddr base from firmware instead */
	carveout_paddr_from_aoc = 0x98000000;
	carveout_vaddr_from_aoc = 0x78000000;

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
	if (dram_pages)
		vfree(dram_pages);
err_vmalloc:
err_coredump:
	/* make sure there is no AoC startup work active */
	cancel_work_sync(&prvdata->online_work);

	mutex_lock(&aoc_service_lock);
	aoc_take_offline(prvdata);
	restart_rc = aoc_watchdog_restart(prvdata, &aoc_module_params);
	if (restart_rc == AOC_RESTART_DISABLED_RC) {
		dev_info(prvdata->dev, "aoc subsystem restart is disabled\n");
	} else if (restart_rc) {
		dev_info(prvdata->dev, "aoc subsystem restart failed: rc = %d\n", restart_rc);
	} else {
		dev_info(prvdata->dev, "aoc subsystem restart succeeded\n");
	}

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

static int aoc_open(struct inode *inode, struct file *file)
{
	struct aoc_prvdata *prvdata = container_of(inode->i_cdev,
					struct aoc_prvdata, cdev);

	file->private_data = prvdata;
	return 0;
}

static long aoc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct aoc_prvdata *prvdata = file->private_data;
	long ret = -EINVAL;

	switch (cmd) {
	case AOC_IOCTL_ION_FD_TO_HANDLE:
	{
		ret = aoc_unlocked_ioctl_handle_ion_fd(cmd, arg);
		if (ret == -EINVAL) {
			pr_err("invalid argument\n");
		}
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

	case AOC_IOCTL_VOLTE_RELEASE_MIF:
	{
		u32 volte_release_mif;

		BUILD_BUG_ON(sizeof(volte_release_mif) != _IOC_SIZE(AOC_IOCTL_VOLTE_RELEASE_MIF));

		if (copy_from_user(&volte_release_mif, (u32 *)arg, _IOC_SIZE(cmd)))
			break;

		prvdata->volte_release_mif = volte_release_mif;
		if (prvdata->volte_release_mif != 0)
			pr_info("AoC setting release Mif on Volte\n");

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
			aoc_clear_iommu(prvdata);
			prvdata->domain = NULL;
		}
	}

}

static void release_gsa_device(void *prv)
{
	struct device *gsa_device = (struct device *)prv;
	put_device(gsa_device);
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
					&gsa_pdev->dev);
}

static int aoc_core_suspend(struct device *dev)
{
	struct aoc_prvdata *prvdata = dev_get_drvdata(dev);
	size_t total_services = aoc_num_services();
	int i = 0;

	atomic_inc(&prvdata->aoc_process_active);
	if (aoc_state != AOC_STATE_ONLINE)
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
	if (aoc_state != AOC_STATE_ONLINE)
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

static void aoc_pheap_map(struct samsung_dma_buffer *buffer, void *ctx, bool should_map)
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
		prvdata->map_handler((u64)buffer->priv, phys, size, should_map,
				     prvdata->map_handler_ctx);
	}
	mutex_unlock(&aoc_service_lock);
}

static void aoc_pheap_alloc_cb(struct samsung_dma_buffer *buffer, void *ctx)
{
	aoc_pheap_map(buffer, ctx, true);
}

static void aoc_pheap_free_cb(struct samsung_dma_buffer *buffer, void *ctx)
{
	aoc_pheap_map(buffer, ctx, false);
}

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

bool aoc_create_dma_buf_heaps(struct aoc_prvdata *prvdata)
{
	phys_addr_t base = prvdata->dram_resource.start + resource_size(&prvdata->dram_resource);

	if (resource_size(&prvdata->dram_resource) < SENSOR_DIRECT_HEAP_SIZE +
			PLAYBACK_HEAP_SIZE + CAPTURE_HEAP_SIZE)
		return false;

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

/* Returns true if `base` is located within the aoc dram carveout */
static bool is_aoc_dma_buf(struct aoc_prvdata *prvdata, phys_addr_t base) {
	phys_addr_t dram_carveout_start;
	phys_addr_t dram_carveout_end;

	dram_carveout_start = prvdata->dram_resource.start;
	dram_carveout_end = dram_carveout_start + resource_size(&prvdata->dram_resource);
	return (base <= dram_carveout_end && base >= dram_carveout_start);
}

long aoc_unlocked_ioctl_handle_ion_fd(unsigned int cmd, unsigned long arg)
{
	struct aoc_ion_handle handle;
	struct dma_buf *dmabuf;
	struct samsung_dma_buffer *dma_heap_buf;

	struct ion_physical_heap *phys_heap;
	phys_addr_t base;
	long ret = -EINVAL;
	struct aoc_prvdata *prvdata = platform_get_drvdata(aoc_platform_device);

	BUILD_BUG_ON(sizeof(struct aoc_ion_handle) !=
				_IOC_SIZE(AOC_IOCTL_ION_FD_TO_HANDLE));

	if (copy_from_user(&handle, (struct aoc_ion_handle *)arg, _IOC_SIZE(cmd)))
		return ret;

	dmabuf = dma_buf_get(handle.fd);
	if (IS_ERR(dmabuf))
		 return -EINVAL;

	dma_heap_buf = dmabuf->priv;
	handle.handle = (u64)dma_heap_buf->priv;

	/*
	 * Ensure base is in aoc dram carveout. Ensures that the dmabuf
	 * is created and maintained by AoC.
	 */
	base = 0;
	if (dma_heap_buf->heap->priv) {
		phys_heap = dma_heap_buf->heap->priv;
		base = phys_heap->base;
	}

	if (!(is_aoc_dma_buf(prvdata, base)))
		return ret;

	dma_buf_put(dmabuf);

	if (!copy_to_user((struct aoc_ion_handle *)arg, &handle, _IOC_SIZE(cmd)))
		ret = 0;

	return ret;
}

static int platform_probe_parse_dt(struct device *dev, struct device_node *aoc_node)
{
	struct aoc_prvdata *prvdata = platform_get_drvdata(aoc_platform_device);

	prvdata->aoc_pcu_base = dt_property(aoc_node, "pcu-base");
	if (prvdata->aoc_pcu_base == DT_PROPERTY_NOT_FOUND) {
		dev_err(dev, "AOC DT missing property pcu-base");
		return -EINVAL;
	}
	prvdata->aoc_gpio_base = dt_property(aoc_node, "gpio-base");
	if (prvdata->aoc_gpio_base == DT_PROPERTY_NOT_FOUND) {
		dev_err(dev, "AOC DT missing property gpio-base");
		return -EINVAL;
	}
	prvdata->aoc_pcu_db_set_offset = dt_property(aoc_node, "pcu-db-set-offset");
	if (prvdata->aoc_pcu_db_set_offset == DT_PROPERTY_NOT_FOUND) {
		dev_err(dev, "AOC DT missing property pcu-db-set-offset");
		return -EINVAL;
	}
	prvdata->aoc_pcu_db_clr_offset = dt_property(aoc_node, "pcu-db-clr-offset");
	if (prvdata->aoc_pcu_db_clr_offset == DT_PROPERTY_NOT_FOUND) {
		dev_err(dev, "AOC DT missing property pcu-db-clr-offset");
		return -EINVAL;
	}
	prvdata->aoc_cp_aperture_start_offset = dt_property(aoc_node,
								"cp-aperture-start-offset");
	if (prvdata->aoc_cp_aperture_start_offset == DT_PROPERTY_NOT_FOUND) {
		dev_err(dev, "AOC DT missing property cp-aperture-start-offset");
		return -EINVAL;
	}
	prvdata->aoc_cp_aperture_end_offset = dt_property(aoc_node,
								"cp-aperture-end-offset");
	if (prvdata->aoc_cp_aperture_end_offset == DT_PROPERTY_NOT_FOUND) {
		dev_err(dev, "AOC DT missing property cp-aperture-end-offset");
		return -EINVAL;
	}
	prvdata->aoc_clock_divider = dt_property(aoc_node, "clock-divider");
	if (prvdata->aoc_clock_divider == DT_PROPERTY_NOT_FOUND) {
		dev_err(dev, "AOC DT missing property clock-divider");
		return -EINVAL;
	}
	prvdata->aoc_mbox_channels =  dt_property(aoc_node, "mbox-channels");
	if (prvdata->aoc_mbox_channels == DT_PROPERTY_NOT_FOUND) {
		dev_err(dev, "AOC DT missing property mbox-channels");
		return -EINVAL;
	}
	prvdata->iommu_config_persistent = of_property_read_bool(aoc_node,
									"iommu-config-persistent");

	return 0;
}

static int aoc_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aoc_prvdata *prvdata = NULL;
	struct device_node *aoc_node, *mem_node, *iommu_node;
	struct resource *rsrc;
	int ret;
	int rc;
	int i;

	if (aoc_platform_device != NULL) {
		dev_err(dev,
			"already matched the AoC to another platform device");
		rc = -EEXIST;
		goto err_platform_not_null;
	}
	aoc_platform_device = pdev;

	aoc_node = dev->of_node;
	mem_node = of_parse_phandle(aoc_node, "memory-region", 0);

	prvdata = devm_kzalloc(dev, sizeof(*prvdata), GFP_KERNEL);
	if (!prvdata) {
		rc = -ENOMEM;
		goto err_failed_prvdata_alloc;
	}
	platform_set_drvdata(pdev, prvdata);

	if (platform_probe_parse_dt(dev, aoc_node) < 0) {
		rc = -EINVAL;
		goto err_invalid_dt;
	}

	prvdata->mbox_channels = devm_kzalloc(dev,
		sizeof(struct mbox_slot) * prvdata->aoc_mbox_channels, GFP_KERNEL);
	if (!prvdata->mbox_channels) {
		rc = -ENOMEM;
		goto err_failed_prvdata_alloc;
	}

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
		rc = -EINVAL;
		goto err_failed_prvdata_alloc;
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

	for (i = 0; i < prvdata->aoc_mbox_channels; i++) {
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

	rc = allocate_mailbox_channels(prvdata);
	if (rc) {
		dev_err(dev, "failed to allocate mailbox channels %d\n", rc);
		goto err_mem_resources;
	}

	init_waitqueue_head(&prvdata->aoc_reset_wait_queue);
	INIT_WORK(&prvdata->watchdog_work, aoc_watchdog);

	ret = configure_watchdog_interrupt(pdev, prvdata);
	if (ret < 0)
		goto err_watchdog_irq;

	iommu_node = of_parse_phandle(aoc_node, "iommus", 0);
	if (!iommu_node) {
		dev_err(dev, "failed to find iommu device tree node\n");
		rc = -ENODEV;
		goto err_watchdog_iommu_irq;
	}
	ret = configure_iommu_interrupts(dev, iommu_node, prvdata);
	if (ret < 0)
		goto err_watchdog_iommu_irq;
	of_node_put(iommu_node);

	pr_notice("found aoc with interrupt:%d sram:%pR dram:%pR\n", aoc_irq,
		  aoc_sram_resource, &prvdata->dram_resource);

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

	pm_runtime_set_active(dev);
	/* Leave AoC in suspended state. Otherwise, AoC IOMMU is set to active which results in the
	 * IOMMU driver trying to access IOMMU SFRs during device suspend/resume operations. The
	 * latter is problematic if AoC is in monitor mode and BLK_AOC is off. */

	pm_runtime_set_suspended(dev);

	prvdata->domain = iommu_get_domain_for_dev(dev);
	if (!prvdata->domain) {
		pr_err("failed to find iommu domain\n");
		rc = -EIO;
		goto err_find_iommu;
	}

	aoc_configure_ssmt(pdev);

	if (!aoc_create_dma_buf_heaps(prvdata)) {
		pr_err("Unable to create dma_buf heaps\n");
		aoc_cleanup_resources(pdev);
		return -ENOMEM;
	}

	prvdata->sensor_power_count = of_property_count_strings(aoc_node, "sensor_power_list");
	if (prvdata->sensor_power_count > MAX_SENSOR_POWER_NUM) {
		pr_warn("sensor power count %i is larger than available number.",
			prvdata->sensor_power_count);
		prvdata->sensor_power_count = MAX_SENSOR_POWER_NUM;
	} else if (prvdata->sensor_power_count < 0) {
		pr_err("unsupported sensor power list, err = %i.", prvdata->sensor_power_count);
		prvdata->sensor_power_count = 0;
	}

	ret = of_property_read_string_array(aoc_node, "sensor_power_list",
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

	aoc_parse_dmic_power(prvdata, aoc_node);
	configure_dmic_regulator(prvdata, true);

	/* Default to 6MB if we are not loading the firmware (i.e. trace32) */
	aoc_control = aoc_dram_translate(prvdata, 6 * SZ_1M);

	INIT_WORK(&prvdata->online_work, aoc_did_become_online);

	INIT_DELAYED_WORK(&prvdata->monitor_work, aoc_monitor_online);

	aoc_configure_hardware(prvdata);

	rc = platform_specific_probe(pdev, prvdata);

	if (aoc_autoload_firmware) {
		ret = start_firmware_load(dev);
		if (ret != 0)
			pr_err("failed to start firmware download: %d\n", ret);
	}

	ret = sysfs_create_groups(&dev->kobj, aoc_groups);

	pr_debug("platform_probe matched\n");

	return 0;

/* err_acmp_reset: */
err_find_iommu:
err_sram_dram_map:

err_watchdog_iommu_irq:
err_watchdog_irq:
err_mem_resources:
	aoc_cleanup_resources(pdev);
err_memnode:
	deinit_chardev(prvdata);
err_chardev:
err_failed_prvdata_alloc:
err_invalid_dt:
	aoc_platform_device = NULL;
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

	configure_crash_interrupts(prvdata, false);
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
MODULE_IMPORT_NS(DMA_BUF);
