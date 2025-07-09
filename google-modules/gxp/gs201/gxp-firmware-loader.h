/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP firmware loading management.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GXP_FIRMWARE_LOADER_H_
#define __GXP_FIRMWARE_LOADER_H_

#include <gcip/gcip-image-config.h>

#include "gxp-config.h"
#include "gxp-internal.h"

struct gxp_firmware_loader_manager {
	const struct firmware *core_firmware[GXP_NUM_CORES];
	char *core_firmware_name;
	/*
	 * Cached core 0 firmware image config, for easier fetching config entries.
	 * Not a pointer to the firmware buffer because we want to forcely change the
	 * privilege level to NS.
	 * Only valid on the firmware loaded.
	 */
	struct gcip_image_config core_img_cfg;
#if GXP_HAS_MCU
	const struct firmware *mcu_firmware;
	char *mcu_firmware_name;
#endif
	bool is_loaded;
	/* Protects above fields */
	struct mutex lock;
};

/*
 * Initializes the firmware loader subsystem.
 */
int gxp_firmware_loader_init(struct gxp_dev *gxp);

/*
 * Tears down the firmware loader subsystem.
 */
void gxp_firmware_loader_destroy(struct gxp_dev *gxp);

/*
 * Requests and loads all firmware only if firmware is not loaded.
 *
 * Returns 0 on success, a negative errno on failure.
 */
int gxp_firmware_loader_load_if_needed(struct gxp_dev *gxp);

/*
 * Unloads firmware.
 */
void gxp_firmware_loader_unload(struct gxp_dev *gxp);

/*
 * Returns a copied core firmware name prefix, the caller needs to release it by
 * kfree.
 */
char *gxp_firmware_loader_get_core_fw_name(struct gxp_dev *gxp);

/*
 * Set the core firmware name prefix to be requested in
 * `gxp_firmware_loader_load_if_needed()`.
 * It's safe for caller to release @fw_name after calling this function.
 *
 * If @fw_name is NULL, this function will simply unset the name.
 */
void gxp_firmware_loader_set_core_fw_name(struct gxp_dev *gxp,
					  const char *fw_name);
/*
 *
 * Returns a copied MCU firmware name, the caller needs to release it by
 * kfree.
 */
char *gxp_firmware_loader_get_mcu_fw_name(struct gxp_dev *gxp);

/*
 * Set the MCU firmware name to be requested in
 * `gxp_firmware_loader_load_if_needed()`.
 * It's safe for caller to release @fw_name after calling this function.
 */
void gxp_firmware_loader_set_mcu_fw_name(struct gxp_dev *gxp,
					 const char *fw_name);

#endif  /* __GXP_FIRMWARE_LOADER_H_ */
