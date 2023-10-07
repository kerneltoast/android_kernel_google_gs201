/* SPDX-License-Identifier: GPL-2.0 */
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

#if !IS_ENABLED(CONFIG_GXP_TEST)

#ifdef CHIP_AURORA_SCRATCHPAD_OFF

#define AURORA_SCRATCHPAD_OFF CHIP_AURORA_SCRATCHPAD_OFF
#define AURORA_SCRATCHPAD_LEN CHIP_AURORA_SCRATCHPAD_LEN

#else /* CHIP_AURORA_SCRATCHPAD_OFF */

#define AURORA_SCRATCHPAD_OFF 0x000FF000 /* Last 4KB of ELF load region */
#define AURORA_SCRATCHPAD_LEN 0x00001000 /* 4KB */

#endif /* CHIP_AURORA_SCRATCHPAD_OFF */

#else /* CONFIG_GXP_TEST */
/* Firmware memory is shrunk in unit tests. */
#define AURORA_SCRATCHPAD_OFF 0x000F0000
#define AURORA_SCRATCHPAD_LEN 0x00010000

#endif /* CONFIG_GXP_TEST */

#define Q7_ALIVE_MAGIC	0x55555555

#define SCRATCHPAD_MSG_OFFSET(_msg_) (_msg_  <<  2)

#define PRIVATE_FW_DATA_SIZE SZ_2M
#define SHARED_FW_DATA_SIZE SZ_1M

/* Indexes same as image_config.IommuMappingIdx in the firmware side. */
enum gxp_imgcfg_idx {
	CORE_CFG_REGION_IDX,
	VD_CFG_REGION_IDX,
	SYS_CFG_REGION_IDX,
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

/* The caller must have locked gxp->vd_semaphore for reading. */
static inline bool gxp_is_fw_running(struct gxp_dev *gxp, uint core)
{
	return (gxp->firmware_mgr->firmware_running & BIT(core)) != 0;
}

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
 * Returns the specified core's boot mode or boot status.
 * This function should be called only after the firmware has been run.
 */
u32 gxp_firmware_get_boot_mode(struct gxp_dev *gxp,
			       struct gxp_virtual_device *vd, uint core);

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

/* Returns whether the core firmware running states are managed by us. */
bool gxp_core_boot(struct gxp_dev *gxp);

/*
 * Disable external interrupts to core.
 */
void gxp_firmware_disable_ext_interrupts(struct gxp_dev *gxp, uint core);

#endif /* __GXP_FIRMWARE_H__ */
