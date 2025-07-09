/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP firmware loader.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_FIRMWARE_H__
#define __GXP_FIRMWARE_H__

#include <linux/bitops.h>
#include <linux/sizes.h>

#include <gcip/gcip-image-config.h>

#include "gxp-config.h"
#include "gxp-internal.h"

#define Q7_ALIVE_MAGIC	0x55555555

#define CORE_CFG_REGION_SIZE (SZ_8K * GXP_NUM_CORES)
#define VD_CFG_REGION_SIZE (SZ_4K)

/*
 * Enum for specifying what information needs to be fetched from the image_config.IommuMapping
 * array.
 */
enum gxp_imgcfg_type {
	IMAGE_CONFIG_CORE_CFG_REGION,
	IMAGE_CONFIG_VD_CFG_REGION,
	IMAGE_CONFIG_SYS_CFG_REGION,
};

/*
 * Indexes same as image_config.IommuMappingIdx in the firmware side.
 *
 * This enum is defined in header file to be shared with unit tests. For functions not in
 * gxp-firmware.c, they should always use gxp_firmware_get_cfg_resource() instead.
 */
enum gxp_imgcfg_idx {
	/* Indexes for image_config with ConfigVersion = 2. */
	CORE_CFG_REGION_IDX,
	VD_CFG_REGION_IDX,
	SYS_CFG_REGION_IDX,

	/* Also indexes, but for image_config with ConfigVersion >= 3. */
	MCU_SHARED_REGION_IDX = 0,
	SYS_CFG_REGION_IDX_V3 = 1,
};

struct gxp_firmware_manager {
	/* Firmware status bitmap. Accessors must hold `vd_semaphore`. */
	u32 firmware_running;
};

enum aurora_msg {
	MSG_CORE_ALIVE,
	MSG_TOP_ACCESS_OK,
	MSG_BOOT_MODE,
	MSG_SCRATCHPAD_MAX,
};

/*
 * Initializes the core firmware loading/unloading subsystem. This includes
 * initializing the LPM and obtaining the memory regions needed to load the FW.
 * The function needs to be called once after a block power up event.
 */
int gxp_fw_init(struct gxp_dev *gxp);

/*
 * Tears down the firmware loading/unloading subsystem in preparation for a
 * block-level shutdown event. To be called once before a block shutdown.
 */
void gxp_fw_destroy(struct gxp_dev *gxp);

/*
 * Requests and loads core firmware into memories.
 * If the loaded firmware is ELF, rearranges it.
 *
 * Returns 0 on success, a negative errno on failure.
 */
int gxp_firmware_load_core_firmware(
	struct gxp_dev *gxp, char *name_prefix,
	const struct firmware *core_firmwares[GXP_NUM_CORES]);

/*
 * Rearranges firmware data if the firmware is ELF.
 *
 * Returns 0 on success, a negative errno on failure.
 */
int gxp_firmware_rearrange_elf(struct gxp_dev *gxp,
			       const struct firmware *firmwares[GXP_NUM_CORES]);

/*
 * All functions below, which manage the state of or communicate with the core firmware, should only
 * be called in direct mode.
 */

/* The caller must have locked gxp->vd_semaphore for reading. */
static inline bool gxp_is_fw_running(struct gxp_dev *gxp, uint core)
{
	return (gxp->firmware_mgr->firmware_running & BIT(core)) != 0;
}

/*
 * Re-program the reset vector and power on the core's LPM if the block had
 * been shut down.
 *
 * @core should be virt core when using per-VD config method, otherwise should
 * be phys core.
 */
int gxp_firmware_setup_hw_after_block_off(struct gxp_dev *gxp, uint core,
					  uint phys_core, bool verbose);

/*
 *  Loads the firmware for the cores in system memory and powers up the cores
 *  to start FW execution.
 */
int gxp_firmware_run(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		     uint core_list);

/*
 * Shuts down the cores and releases the resources.
 */
void gxp_firmware_stop(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		       uint core_list);

/*
 * Sets the specified core's boot mode or suspend request value.
 * This function should be called only after the firmware has been run.
 */
void gxp_firmware_set_boot_mode(struct gxp_dev *gxp,
				struct gxp_virtual_device *vd, uint core,
				u32 mode);

/*
 * Sets the specified core's boot status or suspend request value.
 */
void gxp_firmware_set_boot_status(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint core,
				  u32 status);

/*
 * Returns the specified core's boot status or boot status.
 * This function should be called only after the firmware has been run.
 */
u32 gxp_firmware_get_boot_status(struct gxp_dev *gxp,
				 struct gxp_virtual_device *vd, uint core);

/*
 * Returns the `generate_debug_dump` flag from the given virtual device's shared host-core region
 * for the specified core.
 */
u32 gxp_firmware_get_generate_debug_dump(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
					 uint core);

/*
 * Sets the `generate_debug_dump` flag for the given virtual device's shared host-core region
 * for the specified core.
 */
void gxp_firmware_set_generate_debug_dump(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
					  uint core, u32 generate_debug_dump);

/*
 * Returns the `debug_dump_generated` flag from the given virtual device's shared host-core region
 * for the specified core.
 */
u32 gxp_firmware_get_debug_dump_generated(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
					  uint core);

/*
 * Sets the `generate_debug_dump` flag from the given virtual device's shared host-core region
 * for the specified core.
 */
void gxp_firmware_set_debug_dump_generated(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
					   uint core, u32 debug_dump_generated);

/*
 * Disables external interrupts to core.
 */
void gxp_firmware_disable_ext_interrupts(struct gxp_dev *gxp, uint core);

/*
 * Fetches the virtual address and size specified in image config's iommu_mappings. @res's size and
 * daddr fields are set on success, other fields remain untouched.
 *
 * Returns a negative errno on parsing error, such as the specified type couldn't be found in
 * provided image config.
 */
int gxp_firmware_get_cfg_resource(struct gxp_dev *gxp, const struct gcip_image_config *img_cfg,
				  enum gxp_imgcfg_type type, struct gxp_mapped_resource *res);

#endif /* __GXP_FIRMWARE_H__ */
