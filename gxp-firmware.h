/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP firmware loader.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_FIRMWARE_H__
#define __GXP_FIRMWARE_H__

#include <linux/bitops.h>

#include "gxp-internal.h"

#if !IS_ENABLED(CONFIG_GXP_TEST)

#define AURORA_SCRATCHPAD_OFF 0x000FF000 /* Last 4KB of ELF load region */
#define AURORA_SCRATCHPAD_LEN 0x00001000 /* 4KB */

#else /* CONFIG_GXP_TEST */
/* Firmware memory is shrunk in unit tests. */
#define AURORA_SCRATCHPAD_OFF 0x000F0000
#define AURORA_SCRATCHPAD_LEN 0x00010000

#endif /* CONFIG_GXP_TEST */

#define Q7_ALIVE_MAGIC	0x55555555

#define SCRATCHPAD_MSG_OFFSET(_msg_) (_msg_  <<  2)

enum aurora_msg {
	MSG_CORE_ALIVE,
	MSG_TOP_ACCESS_OK,
	MSG_BOOT_MODE,
	MSG_SCRATCHPAD_MAX,
};

/* The caller must have locked gxp->vd_semaphore for reading. */
static inline bool gxp_is_fw_running(struct gxp_dev *gxp, uint core)
{
	return (gxp->firmware_running & BIT(core)) != 0;
}

/*
 * Initializes the firmware loading/unloading subsystem. This includes
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
 * Check if the DSP firmware files have been requested yet, and if not, request
 * them.
 *
 * Returns 0 if the files have already been requested or were successfully
 * requested by this call; Returns an errno if this call attempted to request
 * the files and it failed.
 */
int gxp_firmware_request_if_needed(struct gxp_dev *gxp);

/*
 * Re-program the reset vector and power on the core's LPM if the block had
 * been shut down.
 */
int gxp_firmware_setup_hw_after_block_off(struct gxp_dev *gxp, uint core,
					  bool verbose);

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
void gxp_firmware_set_boot_mode(struct gxp_dev *gxp, uint core, u32 mode);

/*
 * Returns the specified core's boot mode or boot status.
 * This function should be called only after the firmware has been run.
 */
u32 gxp_firmware_get_boot_mode(struct gxp_dev *gxp, uint core);

#endif /* __GXP_FIRMWARE_H__ */
