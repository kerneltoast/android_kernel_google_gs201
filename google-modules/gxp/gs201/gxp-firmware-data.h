/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP firmware data manager.
 * A sub-module responsible for managing the resources/data regions shared
 * between the GXP driver and FW.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_FIRMWARE_DATA_H__
#define __GXP_FIRMWARE_DATA_H__

#include <linux/sizes.h>

#include "gxp-dma.h"
#include "gxp-internal.h"
#include "gxp-vd.h"

enum gxp_fw_data_protocol {
	/* Use the per-VD configuration region. */
	FW_DATA_PROTOCOL_PER_VD_CONFIG = 2,
};

/**
 * gxp_fw_data_init() - Initializes the FW data manager submodule.
 * @gxp: The parent GXP device
 *
 * Return:
 * 0       - Successfully initialized submodule
 * -ENOMEM - Insufficient memory to create the submodule
 * -ENODEV - Failed to locate the shared driver-device region
 * -Other  - Error codes propagated from internal functions.
 */
int gxp_fw_data_init(struct gxp_dev *gxp);

/**
 * gxp_fw_data_destroy() - Destroys the FW data manager submodule and free all
 *                         its resources.
 * @gxp: The parent GXP device
 */
void gxp_fw_data_destroy(struct gxp_dev *gxp);

/**
 * gxp_fw_data_populate_vd_cfg() - Sets up the resources to VD's per-core config
 *                                 regions and per-VD config regions.
 * @gxp: The parent GXP device
 * @vd: The virtual device to be populated for
 */
void gxp_fw_data_populate_vd_cfg(struct gxp_dev *gxp,
				 struct gxp_virtual_device *vd);

/**
 * gxp_fw_data_set_core_telemetry_descriptors() - Set new logging or tracing
 *                                                buffers for firmware to write
 *                                                to.
 * @gxp: The GXP device to set buffer descriptors for
 * @host_status:  Bitfield describing the host's core telemetry status. See the
 *                bit definitions in gxp-host-device-structs.h.
 * @buffers: An array of coherent buffers for logging and tracing
 * @per_buffer_size: The size of each core's logging or tracing buffer in bytes
 *
 * `gxp_fw_data_init()` must have been called before this function.
 *
 * Caller must hold gxp->core_telemetry_mgr's lock.
 *
 * Return:
 * 0       - Success
 * -EINVAL - @buffer_addrs are not addressable by @gxp
 */
int gxp_fw_data_set_core_telemetry_descriptors(struct gxp_dev *gxp, u32 host_status,
					       struct gxp_coherent_buf *buffers,
					       u32 per_buffer_size);

/**
 * gxp_fw_data_get_core_telemetry_device_status() - Returns a bitfield
 *                                                  describing a core's
 *                                                  telemetry status.
 * @gxp: The GXP device to get core telemetry status for
 * @core: The core in @gxp to get the core telemetry status for
 *
 * Caller must hold gxp->core_telemetry_mgr's lock.
 *
 * Return: The bitfield describing @core's telemetry status. If @core is invalid, returns 0.
 */
u32 gxp_fw_data_get_core_telemetry_device_status(struct gxp_dev *gxp, uint core);

/**
 * gxp_fw_data_resource() - Returns the resource of data region for host<->core
 *		            communication.
 * @gxp: The GXP device
 *
 * This function requires either @gxp->fwdatabuf or @gxp->shared_buf be
 * initialized, so it couldn't be called during device probe time.
 *
 * Return: The resource.
 */
struct gxp_mapped_resource gxp_fw_data_resource(struct gxp_dev *gxp);

/**
 * gxp_fw_data_system_cfg() - Returns the pointer to the system config region.
 * @gxp: The GXP device
 *
 * This function requires either @gxp->fwdatabuf or @gxp->shared_buf be
 * initialized, so it couldn't be called during device probe time.
 *
 * Return: The pointer. This function never fails.
 */
void *gxp_fw_data_system_cfg(struct gxp_dev *gxp);

/**
 * gxp_fw_data_populate_system_config() - Populate settings onto firmware system config region.
 * @gxp: The GXP device.
 * @sys_cfg_size: The size of system config region.
 *
 * This function is expected to be called after firmware loaded to know the correct size of the
 * system config.
 * The size will be recorded in gxp->data_mgr.
 */
void gxp_fw_data_populate_system_config(struct gxp_dev *gxp, u32 sys_cfg_size);

#endif /* __GXP_FIRMWARE_DATA_H__ */
