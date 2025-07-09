// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU firmware management for mobile chipsets.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/gsa/gsa_tpu.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "edgetpu.h"
#include "edgetpu-config.h"
#include "edgetpu-firmware.h"
#include "edgetpu-firmware-util.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-mmu.h"
#include "edgetpu-mobile-platform.h"
#include "mobile-firmware.h"

#define SSMT_NS_READ_STREAM_VID_OFFSET(n) (0x1000u + (0x4u * (n)))
#define SSMT_NS_WRITE_STREAM_VID_OFFSET(n) (0x1200u + (0x4u * (n)))

#define SSMT_NS_READ_STREAM_VID_REG(base, n)                                   \
	((base) + SSMT_NS_READ_STREAM_VID_OFFSET(n))
#define SSMT_NS_WRITE_STREAM_VID_REG(base, n)                                  \
	((base) + SSMT_NS_WRITE_STREAM_VID_OFFSET(n))

static struct mobile_image_config *mobile_firmware_get_image_config(struct edgetpu_dev *etdev)
{
	return edgetpu_firmware_get_data(etdev->firmware);
}

/* Clear mapping #i */
static void clear_mapping(struct edgetpu_dev *etdev,
			  struct mobile_image_config *image_config, int i)
{
	tpu_addr_t tpu_addr;
	size_t size;

	tpu_addr = image_config->mappings[i].virt_address;
	size = CONFIG_TO_SIZE(image_config->mappings[i].image_config_value);
	edgetpu_mmu_remove_translation(etdev, tpu_addr, size, EDGETPU_CONTEXT_KCI);
}

static void mobile_firmware_clear_mappings(struct edgetpu_dev *etdev,
					   struct mobile_image_config *image_config)
{
	int i;

	for (i = 0; i < image_config->num_iommu_mapping; i++)
		clear_mapping(etdev, image_config, i);
}

static int mobile_firmware_setup_mappings(struct edgetpu_dev *etdev,
					  struct mobile_image_config *image_config)
{
	int i, ret;
	tpu_addr_t tpu_addr;
	size_t size;
	phys_addr_t phys_addr;

	for (i = 0; i < image_config->num_iommu_mapping; i++) {
		tpu_addr = image_config->mappings[i].virt_address;
		if (!tpu_addr) {
			etdev_warn(etdev, "Invalid firmware header\n");
			ret = -EIO;
			goto err;
		}
		size = CONFIG_TO_SIZE(image_config->mappings[i].image_config_value);
		phys_addr = image_config->mappings[i].image_config_value & ~(0xFFF);

		etdev_dbg(etdev, "Adding IOMMU mapping for firmware : %pad -> %pap", &tpu_addr,
			  &phys_addr);

		ret = edgetpu_mmu_add_translation(etdev, tpu_addr, phys_addr, size,
						  IOMMU_READ | IOMMU_WRITE, EDGETPU_CONTEXT_KCI);
		if (ret) {
			etdev_err(etdev,
				  "Unable to Map: %d tpu_addr: %pad phys_addr: %pap size: %#lx\n",
				  ret, &tpu_addr, &phys_addr, size);
			goto err;
		}
	}

	return 0;

err:
	while (i--)
		clear_mapping(etdev, image_config, i);
	return ret;
}

static void clear_ns_mapping(struct edgetpu_dev *etdev,
			     struct mobile_image_config *image_config, int i)
{
	tpu_addr_t tpu_addr;
	size_t size;

	tpu_addr = image_config->ns_iommu_mappings[i] & ~(0xFFF);
	size = CONFIG_TO_MBSIZE(image_config->ns_iommu_mappings[i]);
	edgetpu_mmu_remove_translation(etdev, tpu_addr, size, EDGETPU_CONTEXT_KCI);
}

static void mobile_firmware_clear_ns_mappings(struct edgetpu_dev *etdev,
					      struct mobile_image_config *image_config)
{
	int i;

	for (i = 0; i < image_config->num_ns_iommu_mappings; i++)
		clear_ns_mapping(etdev, image_config, i);
}

static int mobile_firmware_setup_ns_mappings(struct edgetpu_dev *etdev,
					     struct mobile_image_config *image_config)
{
	tpu_addr_t tpu_addr;
	size_t size = 0;
	int ret = 0, i;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	phys_addr_t phys_addr = etmdev->fw_ctx_paddr;

	for (i = 0; i < image_config->num_ns_iommu_mappings; i++)
		size += CONFIG_TO_MBSIZE(image_config->ns_iommu_mappings[i]);

	if (size > etmdev->fw_ctx_size) {
		etdev_err(etdev, "Insufficient firmware context memory");
		return -ENOSPC;
	}

	for (i = 0; i < image_config->num_ns_iommu_mappings; i++) {
		size = CONFIG_TO_MBSIZE(image_config->ns_iommu_mappings[i]);
		tpu_addr = image_config->ns_iommu_mappings[i] & ~(0xFFF);
		ret = edgetpu_mmu_add_translation(etdev, tpu_addr, phys_addr,
						  size, IOMMU_READ | IOMMU_WRITE,
						  EDGETPU_CONTEXT_KCI);
		if (ret)
			goto err;
		phys_addr += size;
	}

	return 0;

err:
	while (i--)
		clear_ns_mapping(etdev, image_config, i);
	return ret;
}

static int mobile_firmware_after_create(struct edgetpu_firmware *et_fw)
{
	/*
	 * Use firmware data to keep a copy of the image config in order
	 * to avoid re-doing IOMMU mapping on each firmware run
	 */
	struct mobile_image_config *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	edgetpu_firmware_set_data(et_fw, data);
	return 0;
}

static void mobile_firmware_before_destroy(struct edgetpu_firmware *et_fw)
{
	struct mobile_image_config *image_config;
	struct edgetpu_dev *etdev = et_fw->etdev;

	image_config = mobile_firmware_get_image_config(etdev);

	mobile_firmware_clear_ns_mappings(etdev, image_config);
	if (image_config->privilege_level == FW_PRIV_LEVEL_NS)
		mobile_firmware_clear_mappings(etdev, image_config);
	edgetpu_firmware_set_data(et_fw, NULL);
	kfree(image_config);
}

static int mobile_firmware_alloc_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);

	/* Allocate extra space the image header */
	size_t buffer_size =
		etmdev->fw_region_size + MOBILE_FW_HEADER_SIZE;

	fw_buf->vaddr = vmalloc(buffer_size);
	if (!fw_buf->vaddr) {
		etdev_err(etdev, "%s: failed to allocate buffer (%zu bytes)\n",
			  __func__, buffer_size);
		return -ENOMEM;
	}
	fw_buf->dma_addr = 0;
	fw_buf->alloc_size = buffer_size;
	fw_buf->used_size_align = 16;
	return 0;
}

static void mobile_firmware_free_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
	vfree(fw_buf->vaddr);
	fw_buf->vaddr = NULL;
	fw_buf->dma_addr = 0;
	fw_buf->alloc_size = 0;
	fw_buf->used_size_align = 0;
}

static void mobile_firmware_save_image_config(struct edgetpu_dev *etdev,
					      struct mobile_image_config *image_config)
{
	struct mobile_image_config *saved_image_config = mobile_firmware_get_image_config(etdev);

	memcpy(saved_image_config, image_config, sizeof(*saved_image_config));
}

static int mobile_firmware_gsa_authenticate(struct edgetpu_mobile_platform_dev *etmdev,
					    struct edgetpu_firmware_buffer *fw_buf,
					    struct mobile_image_config *image_config,
					    void *image_vaddr)
{
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	void *header_vaddr;
	dma_addr_t header_dma_addr;
	int tpu_state;
	int ret = 0;

	tpu_state = gsa_send_tpu_cmd(etmdev->gsa_dev, GSA_TPU_GET_STATE);

	if (tpu_state < GSA_TPU_STATE_INACTIVE) {
		etdev_err(etdev, "GSA failed to retrieve current status: %d\n", tpu_state);
		return tpu_state;
	}

	etdev_dbg(etdev, "GSA Reports TPU state: %d\n", tpu_state);

	if (tpu_state > GSA_TPU_STATE_INACTIVE) {
		ret = gsa_unload_tpu_fw_image(etmdev->gsa_dev);
		if (ret) {
			etdev_warn(etdev, "GSA release failed: %d\n", ret);
			return -EIO;
		}
	}

	/* Copy the firmware image to the target location, skipping the header */
	memcpy(image_vaddr, fw_buf->vaddr + MOBILE_FW_HEADER_SIZE,
	       fw_buf->used_size - MOBILE_FW_HEADER_SIZE);

	/* Allocate coherent memory for the image header */
	header_vaddr = dma_alloc_coherent(etmdev->gsa_dev,
					  MOBILE_FW_HEADER_SIZE,
					  &header_dma_addr, GFP_KERNEL);
	if (!header_vaddr) {
		etdev_err(etdev,
			  "Failed to allocate coherent memory for header\n");
		return -ENOMEM;
	}

	memcpy(header_vaddr, fw_buf->vaddr, MOBILE_FW_HEADER_SIZE);
	etdev_dbg(etdev, "Requesting GSA image load. meta = %pad payload = %pap", &header_dma_addr,
		  &etmdev->fw_region_paddr);

	ret = gsa_load_tpu_fw_image(etmdev->gsa_dev, header_dma_addr,
				    etmdev->fw_region_paddr);
	if (ret)
		etdev_err(etdev, "GSA authentication failed: %d\n", ret);

	dma_free_coherent(etmdev->gsa_dev, MOBILE_FW_HEADER_SIZE, header_vaddr, header_dma_addr);

	return ret;
}

static void mobile_firmware_setup_ssmt(struct edgetpu_dev *etdev)
{
	int i;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct mobile_image_config *image_config = mobile_firmware_get_image_config(etdev);

	/*
	 * This only works if the SSMT is set to client-driven mode, which only GSA can do.
	 * Skip if GSA is not available
	 */
	if (!etmdev->ssmt_base || !etmdev->gsa_dev)
		return;

	etdev_dbg(etdev, "Setting up SSMT for privilege level: %d\n",
		  image_config->privilege_level);

	/*
	 * Setup non-secure SCIDs, assume VID = SCID when running at TZ or GSA level,
	 * Reset the table to zeroes if running non-secure firmware, since the SSMT
	 * will be in clamped mode and we want all memory accesses to go to the
	 * default page table.
	 */
	for (i = 0; i < EDGETPU_NCONTEXTS; i++) {
		int val;

		if (image_config->privilege_level == FW_PRIV_LEVEL_NS)
			val = 0;
		else
			val = i;

		writel(val, SSMT_NS_READ_STREAM_VID_REG(etmdev->ssmt_base, i));
		writel(val, SSMT_NS_WRITE_STREAM_VID_REG(etmdev->ssmt_base, i));
	}
}

static int mobile_firmware_prepare_run(struct edgetpu_firmware *et_fw,
				       struct edgetpu_firmware_buffer *fw_buf)
{
	struct edgetpu_dev *etdev = et_fw->etdev;

	/* Reset KCI mailbox before starting f/w, don't process anything old.*/
	edgetpu_mailbox_reset(etdev->kci->mailbox);

	mobile_firmware_setup_ssmt(etdev);

	return edgetpu_mobile_firmware_reset_cpu(etdev, false);
}

static int mobile_firmware_restart(struct edgetpu_firmware *et_fw, bool force_reset)
{
	struct edgetpu_dev *etdev = et_fw->etdev;

	/*
	 * We are in a bad state, reset the CPU and hope the device recovers.
	 * Ignore failures in the reset assert request and proceed to reset release.
	 */
	if (force_reset)
		edgetpu_mobile_firmware_reset_cpu(etdev, true);

	mobile_firmware_setup_ssmt(etdev);

	return edgetpu_mobile_firmware_reset_cpu(etdev, false);
}

static int mobile_firmware_setup_buffer(struct edgetpu_firmware *et_fw,
					struct edgetpu_firmware_buffer *fw_buf)
{
	int ret = 0;
	void *image_vaddr;
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct mobile_image_config *image_config;
	struct mobile_image_config *last_image_config = mobile_firmware_get_image_config(etdev);
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	phys_addr_t image_start, image_end, carveout_start, carveout_end;
	bool image_config_changed;
	struct mobile_image_header *hdr;

	if (fw_buf->used_size < MOBILE_FW_HEADER_SIZE) {
		etdev_err(etdev, "Invalid buffer size: %zu < %d\n",
			  fw_buf->used_size, MOBILE_FW_HEADER_SIZE);
		return -EINVAL;
	}

	image_vaddr = memremap(etmdev->fw_region_paddr,
			       etmdev->fw_region_size, MEMREMAP_WC);
	if (!image_vaddr) {
		etdev_err(etdev, "memremap failed\n");
		return -ENOMEM;
	}

	hdr = (struct mobile_image_header *)fw_buf->vaddr;
	if (hdr->Magic != EDGETPU_MOBILE_FW_MAGIC)
		etdev_warn(etdev, "Invalid firmware header magic value %#08x\n",
			   hdr->Magic);

	/* fetch the firmware versions */
	image_config = fw_buf->vaddr + MOBILE_IMAGE_CONFIG_OFFSET;
	memcpy(&etdev->fw_version, &image_config->firmware_versions,
	       sizeof(etdev->fw_version));

	image_config_changed = memcmp(image_config, last_image_config, sizeof(*image_config));

	if (etmdev->gsa_dev) {
		ret = mobile_firmware_gsa_authenticate(etmdev, fw_buf, image_config, image_vaddr);
	} else if (image_config->privilege_level == FW_PRIV_LEVEL_NS) {
		etdev_dbg(etdev, "Loading unauthenticated non-secure firmware\n");
		/* Copy the firmware image to the target location, skipping the header */
		memcpy(image_vaddr, fw_buf->vaddr + MOBILE_FW_HEADER_SIZE,
		       fw_buf->used_size - MOBILE_FW_HEADER_SIZE);
	} else {
		etdev_err(etdev,
			  "Cannot load firmware at privilege level %d with no authentication\n",
			  image_config->privilege_level);
		ret = -EINVAL;
	}

	if (ret)
		goto out;

	image_start = (phys_addr_t)image_config->carveout_base;
	image_end = (phys_addr_t)(image_config->firmware_base + image_config->firmware_size - 1);
	carveout_start = etmdev->fw_region_paddr;
	carveout_end = carveout_start + etmdev->fw_region_size - 1;

	/* Image must fit within the carveout */
	if (image_start < carveout_start || image_end > carveout_end) {
		etdev_err(etdev, "Firmware image doesn't fit in carveout\n");
		etdev_err(etdev, "Image config: %pap - %pap\n", &image_start, &image_end);
		etdev_err(etdev, "Carveout: %pap - %pap\n", &carveout_start, &carveout_end);
		ret = -ERANGE;
		goto out;
	}

	if (!image_config_changed)
		goto out;

	/* clear last image mappings */
	if (last_image_config->privilege_level == FW_PRIV_LEVEL_NS)
		mobile_firmware_clear_mappings(etdev, last_image_config);

	if (image_config->privilege_level == FW_PRIV_LEVEL_NS)
		ret = mobile_firmware_setup_mappings(etdev, image_config);
	if (ret)
		goto out;
	mobile_firmware_clear_ns_mappings(etdev, last_image_config);
	ret = mobile_firmware_setup_ns_mappings(etdev, image_config);
	if (ret) {
		mobile_firmware_clear_mappings(etdev, image_config);
		goto out;
	}
	mobile_firmware_save_image_config(etdev, image_config);
out:
	memunmap(image_vaddr);
	return ret;
}

/* Load firmware for chips that use carveout memory for a single chip. */
int edgetpu_firmware_chip_load_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc, const char *name)
{
	int ret;
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct device *dev = etdev->dev;
	const struct firmware *fw;
	size_t aligned_size;

	ret = request_firmware(&fw, name, dev);
	if (ret) {
		etdev_dbg(etdev,
			  "%s: request '%s' failed: %d\n", __func__, name, ret);
		return ret;
	}

	aligned_size = ALIGN(fw->size, fw_desc->buf.used_size_align);
	if (aligned_size > fw_desc->buf.alloc_size) {
		etdev_dbg(etdev,
			   "%s: firmware buffer too small: alloc size=%#zx, required size=%#zx\n",
			  __func__, fw_desc->buf.alloc_size, aligned_size);
		ret = -ENOSPC;
		goto out_release_firmware;
	}

	memcpy(fw_desc->buf.vaddr, fw->data, fw->size);
	fw_desc->buf.used_size = aligned_size;
	/* May return NULL on out of memory, driver must handle properly */
	fw_desc->buf.name = kstrdup(name, GFP_KERNEL);

out_release_firmware:
	release_firmware(fw);
	return ret;
}

void edgetpu_firmware_chip_unload_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc)
{
	kfree(fw_desc->buf.name);
	fw_desc->buf.name = NULL;
	fw_desc->buf.used_size = 0;
}

int edgetpu_mobile_firmware_reset_cpu(struct edgetpu_dev *etdev, bool assert_reset)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct mobile_image_config *image_config = mobile_firmware_get_image_config(etdev);
	int ret = 0;

	if (image_config->privilege_level == FW_PRIV_LEVEL_NS) {
		int i;

		for (i = 0; i < EDGETPU_NUM_CORES; i++)
			edgetpu_dev_write_32_sync(etdev, EDGETPU_REG_RESET_CONTROL + i * 8,
						  assert_reset ? 1 : 0);
	}
	else if (etmdev->gsa_dev)
		ret = gsa_send_tpu_cmd(etmdev->gsa_dev,
				       assert_reset ? GSA_TPU_SHUTDOWN : GSA_TPU_START);
	else
		ret = -ENODEV;

	etdev_dbg(etdev, "%s CPU reset result = %d", assert_reset ? "assert" : "release", ret);

	if (ret < 0)
		return ret;

	return 0;
}

static const struct edgetpu_firmware_chip_data mobile_firmware_chip_data = {
	.default_firmware_name = EDGETPU_DEFAULT_FIRMWARE_NAME,
	.after_create = mobile_firmware_after_create,
	.before_destroy = mobile_firmware_before_destroy,
	.alloc_buffer = mobile_firmware_alloc_buffer,
	.free_buffer = mobile_firmware_free_buffer,
	.setup_buffer = mobile_firmware_setup_buffer,
	.prepare_run = mobile_firmware_prepare_run,
	.restart = mobile_firmware_restart,
};

int edgetpu_mobile_firmware_create(struct edgetpu_dev *etdev)
{
	return edgetpu_firmware_create(etdev, &mobile_firmware_chip_data);
}

void edgetpu_mobile_firmware_destroy(struct edgetpu_dev *etdev)
{
	edgetpu_firmware_destroy(etdev);
}

unsigned long edgetpu_chip_firmware_iova(struct edgetpu_dev *etdev)
{
	/*
	 * On mobile platforms, firmware address translation may happen in 1 or 2 stages:
	 * 1.- Instruction remap registers.
	 * 2.- IOMMU translation (when not running in GSA privilege).
	 * In either case, the address seen by the TPU's CPU core will remain constant, and
	 * equal to the macro below.
	 */
	return EDGETPU_INSTRUCTION_REMAP_BASE;
}
