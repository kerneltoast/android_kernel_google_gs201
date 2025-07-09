// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP firmware loading management.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <gcip/gcip-common-image-header.h>
#include <gcip/gcip-image-config.h>

#include "gxp-config.h"
#include "gxp-firmware-data.h"
#include "gxp-firmware-loader.h"
#include "gxp-firmware.h"
#include "gxp-internal.h"

#if GXP_HAS_MCU
#include "gxp-gsa.h"
#include "gxp-mcu-firmware.h"
#endif

#if GXP_HAS_MCU
static int gxp_firmware_loader_gsa_auth(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;
	int ret;
	uint core;
	dma_addr_t headers_dma_addr;
	void *header_vaddr;
	const u8 *data;
	struct gxp_mcu_firmware *mcu_fw = gxp_mcu_firmware_of(gxp);

	if (!mcu_fw->is_secure) {
		dev_warn(
			gxp->dev,
			"No need to do firmware authentication with non-secure privilege\n");
		return 0;
	}
	if (!gxp->gsa_dev) {
		dev_warn(
			gxp->dev,
			"No GSA device available, skipping firmware authentication\n");
		return 0;
	}
	/* Authenticate MCU firmware */
	header_vaddr = dma_alloc_coherent(gxp->gsa_dev, GCIP_FW_HEADER_SIZE,
					  &headers_dma_addr, GFP_KERNEL);
	if (!header_vaddr) {
		dev_err(gxp->dev,
			"Failed to allocate coherent memory for header\n");
		return -ENOMEM;
	}
	memcpy(header_vaddr, mgr->mcu_firmware->data, GCIP_FW_HEADER_SIZE);
	ret = gsa_load_dsp_fw_image(gxp->gsa_dev, headers_dma_addr,
				    mcu_fw->image_buf.paddr);
	if (ret) {
		dev_err(gxp->dev, "MCU fw GSA authentication fails");
		goto err_load_mcu_fw;
	}

	for (core = 0; core < GXP_NUM_CORES; core++) {
		data = mgr->core_firmware[core]->data;
		/* Authenticate core firmware */
		memcpy(header_vaddr, data, GCIP_FW_HEADER_SIZE);
		ret = gsa_load_dsp_fw_image(gxp->gsa_dev, headers_dma_addr,
					    gxp->fwbufs[core].paddr);
		if (ret) {
			dev_err(gxp->dev,
				"Core %u firmware authentication fails", core);
			goto err_load_core_fw;
		}
	}
	dma_free_coherent(gxp->gsa_dev, GCIP_FW_HEADER_SIZE, header_vaddr,
			  headers_dma_addr);
	return 0;
err_load_core_fw:
	gsa_unload_dsp_fw_image(gxp->gsa_dev);
err_load_mcu_fw:
	dma_free_coherent(gxp->gsa_dev, GCIP_FW_HEADER_SIZE, header_vaddr,
			  headers_dma_addr);
	return ret;
}

static void gxp_firmware_loader_gsa_unload(struct gxp_dev *gxp)
{
	struct gxp_mcu_firmware *mcu_fw = gxp_mcu_firmware_of(gxp);

	if (mcu_fw->is_secure)
		gsa_unload_dsp_fw_image(gxp->gsa_dev);
}
#endif /* GXP_HAS_MCU */

int gxp_firmware_loader_init(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;
	gxp->fw_loader_mgr = mgr;
	mutex_init(&mgr->lock);
	return 0;
}

void gxp_firmware_loader_destroy(struct gxp_dev *gxp)
{
	gxp_firmware_loader_unload(gxp);
}

void gxp_firmware_loader_set_core_fw_name(struct gxp_dev *gxp,
					  const char *fw_name)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;

	mutex_lock(&mgr->lock);

	kfree(mgr->core_firmware_name);
	mgr->core_firmware_name = NULL;

	if (fw_name)
		mgr->core_firmware_name = kstrdup(fw_name, GFP_KERNEL);

	mutex_unlock(&mgr->lock);
}

char *gxp_firmware_loader_get_core_fw_name(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;
	char *name;

	mutex_lock(&mgr->lock);
	if (mgr->core_firmware_name)
		name = kstrdup(mgr->core_firmware_name, GFP_KERNEL);
	else
		name = kstrdup(DSP_FIRMWARE_DEFAULT_PREFIX, GFP_KERNEL);
	mutex_unlock(&mgr->lock);
	return name;
}

/*
 * Fetches and records image config of the first core firmware.
 */
static void gxp_firmware_loader_get_core_image_config(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;
	struct gcip_common_image_header *hdr =
		(struct gcip_common_image_header *)mgr->core_firmware[0]->data;
	struct gcip_image_config *cfg;

	if (unlikely(mgr->core_firmware[0]->size < GCIP_FW_HEADER_SIZE))
		return;
	cfg = get_image_config_from_hdr(hdr);
	if (cfg)
		mgr->core_img_cfg = *cfg;
	else
		dev_warn(gxp->dev,
			 "Core 0 Firmware doesn't have a valid image config");
}

/*
 * Call this function when mgr->core_firmware have been populated.
 * This function sets is_loaded to true.
 *
 */
static void gxp_firmware_loader_has_loaded(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;

	lockdep_assert_held(&mgr->lock);
	gxp_firmware_loader_get_core_image_config(gxp);
	mgr->is_loaded = true;
}

static void gxp_firmware_loader_unload_core_firmware(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;
	uint core;

	lockdep_assert_held(&mgr->lock);
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (mgr->core_firmware[core]) {
			release_firmware(mgr->core_firmware[core]);
			mgr->core_firmware[core] = NULL;
		}
	}
	kfree(mgr->core_firmware_name);
	mgr->core_firmware_name = NULL;
}

#if GXP_HAS_MCU
static void gxp_firmware_loader_unload_mcu_firmware(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;

	lockdep_assert_held(&mgr->lock);
	if (!gxp_is_direct_mode(gxp)) {
		if (mgr->mcu_firmware) {
			gxp_mcu_firmware_unload(gxp, mgr->mcu_firmware);
			mgr->mcu_firmware = NULL;
		}
		kfree(mgr->mcu_firmware_name);
		mgr->mcu_firmware_name = NULL;
	}
}
#endif /* GXP_HAS_MCU */

static int gxp_firmware_loader_load_locked(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;
	int ret;

	lockdep_assert_held(&mgr->lock);
	ret = gxp_firmware_load_core_firmware(gxp, mgr->core_firmware_name,
					      mgr->core_firmware);
	if (ret)
		return ret;

#if GXP_HAS_MCU
	if (!gxp_is_direct_mode(gxp)) {
		ret = gxp_mcu_firmware_load(gxp, mgr->mcu_firmware_name,
					    &mgr->mcu_firmware);
		if (ret)
			goto err_unload_core;

		ret = gxp_firmware_loader_gsa_auth(gxp);
		if (ret)
			goto err_unload_mcu;
	}
#endif
	ret = gxp_firmware_rearrange_elf(gxp, mgr->core_firmware);
	if (ret)
		goto err_unload;
	gxp_firmware_loader_has_loaded(gxp);
	return 0;

err_unload:
#if GXP_HAS_MCU
	if (!gxp_is_direct_mode(gxp))
		gxp_firmware_loader_gsa_unload(gxp);
err_unload_mcu:
	if (!gxp_is_direct_mode(gxp))
		gxp_firmware_loader_unload_mcu_firmware(gxp);
err_unload_core:
#endif
	gxp_firmware_loader_unload_core_firmware(gxp);
	return ret;
}

int gxp_firmware_loader_load_if_needed(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;
	struct gxp_mapped_resource res = {};
	int ret = 0;

	mutex_lock(&mgr->lock);
	if (mgr->is_loaded)
		goto out_unlock;
	ret = gxp_firmware_loader_load_locked(gxp);
	if (ret)
		goto out_unlock;
	/*
	 * This should be done only after the firmware is successfully loaded because the size of
	 * system config region is required.
	 */
	gxp_firmware_get_cfg_resource(gxp, &mgr->core_img_cfg, IMAGE_CONFIG_SYS_CFG_REGION, &res);
	mutex_unlock(&mgr->lock);

	gxp_fw_data_populate_system_config(gxp, res.size);

	return 0;

out_unlock:
	mutex_unlock(&mgr->lock);
	return ret;
}

void gxp_firmware_loader_unload(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;

	mutex_lock(&mgr->lock);
	if (mgr->is_loaded) {
#if GXP_HAS_MCU
		gxp_firmware_loader_gsa_unload(gxp);
		gxp_firmware_loader_unload_mcu_firmware(gxp);
#endif
		gxp_firmware_loader_unload_core_firmware(gxp);
	}
	mgr->is_loaded = false;
	mutex_unlock(&mgr->lock);
}

#if GXP_HAS_MCU
void gxp_firmware_loader_set_mcu_fw_name(struct gxp_dev *gxp,
					 const char *fw_name)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;

	mutex_lock(&mgr->lock);
	mgr->mcu_firmware_name = kstrdup(fw_name, GFP_KERNEL);
	mutex_unlock(&mgr->lock);
}

char *gxp_firmware_loader_get_mcu_fw_name(struct gxp_dev *gxp)
{
	struct gxp_firmware_loader_manager *mgr = gxp->fw_loader_mgr;
	char *name;

	mutex_lock(&mgr->lock);
	if (mgr->mcu_firmware_name)
		name = kstrdup(mgr->mcu_firmware_name, GFP_KERNEL);
	else
		name = kstrdup(GXP_DEFAULT_MCU_FIRMWARE, GFP_KERNEL);
	mutex_unlock(&mgr->lock);
	return name;
}
#endif /* GXP_HAS_MCU */
