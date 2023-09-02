/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP firmware data manager.
 * A sub-module responsible for managing the resources/data regions shared
 * between the GXP driver and FW.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_FIRMWARE_DATA_H__
#define __GXP_FIRMWARE_DATA_H__

#include "gxp-internal.h"

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
 * gxp_fw_data_create_app() - Allocates HW and memory resources needed to create
 *                            a GXP device application (1:1 with a GXP driver
 *                            virtual device) used by the specified physical
 *                            cores.
 * @gxp: The parent GXP device
 * @core_list: A bitmap of the physical cores used in this application
 *
 * Return:
 * ptr     - A pointer of the newly created application handle, an error pointer
 *           (PTR_ERR) otherwise.
 * -ENOMEM - Insufficient memory to create the application
 */
void *gxp_fw_data_create_app(struct gxp_dev *gxp, uint core_list);

/**
 * gxp_fw_data_destroy_app() - Deallocates the HW and memory resources used by
 *                             the specified application.
 * @gxp: The parent GXP device
 * @application: The handle to the application to deallocate
 */
void gxp_fw_data_destroy_app(struct gxp_dev *gxp, void *application);

/**
 * gxp_fw_data_destroy() - Destroys the FW data manager submodule and free all
 *                         its resources.
 * @gxp: The parent GXP device
 */
void gxp_fw_data_destroy(struct gxp_dev *gxp);

/**
 * gxp_fw_data_set_telemetry_descriptors() - Set new logging or tracing buffers
 *                                           for firmware to write to.
 * @gxp: The GXP device to set buffer descriptors for
 * @type: Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 * @host_status:  Bitfield describing the host's telemetry status. See the
 *                bit definitions in gxp-host-device-structs.h.
 * @buffer_addrs: An array containing the IOVA each physical core can access
 *                its logging or tracing buffer at
 * @per_buffer_size: The size of each core's logging or tracing buffer in bytes
 *
 * `gxp_fw_data_init()` must have been called before this function.
 *
 * Caller must hold gxp->telemetry_mgr's lock.
 *
 * Return:
 * 0       - Success
 * -EINVAL - Invalid @type provided or @buffer_addrs are not addressable by @gxp
 */
int gxp_fw_data_set_telemetry_descriptors(struct gxp_dev *gxp, u8 type,
					  u32 host_status,
					  dma_addr_t *buffer_addrs,
					  u32 per_buffer_size);

/**
 * gxp_fw_data_get_telemetry_device_status() - Returns a bitfield describing a
 *                                             core's telemetry status.
 * @gxp: The GXP device to get device telemetry status for
 * @core: The core in @gxp to get the device telemetry status for
 * @type: Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 *
 * Caller must hold gxp->telemetry_mgr's lock.
 *
 * Return: The bitfield describing @core's telemetry status. If @core or @type
 *         are invalid, the result will always be 0.
 */
u32 gxp_fw_data_get_telemetry_device_status(struct gxp_dev *gxp, uint core,
					    u8 type);

#endif /* __GXP_FIRMWARE_DATA_H__ */
