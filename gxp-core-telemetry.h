/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP core telemetry support
 *
 * Copyright (C) 2021-2022 Google LLC
 */

#ifndef __GXP_CORE_TELEMETRY_H__
#define __GXP_CORE_TELEMETRY_H__

#include <linux/eventfd.h>
#include <linux/refcount.h>
#include <linux/types.h>

#include "gxp-dma.h"
#include "gxp-internal.h"
#include "gxp.h"

/* Core telemetry buffer header size */
#define GXP_CORE_TELEMETRY_BUFFER_HEADER_SIZE SZ_64
/* Default telemetry buffer size per core */
#define CORE_TELEMETRY_DEFAULT_BUFFER_SIZE GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE
/**
 * Maximum core telemetry buffer size that can be represented by GXP_GET_SPECS
 * ioctl. 8 bits are reserved to represent telemetry buffer size in GXP_GET_SPECS
 * ioctl and the size is represented in unit of GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE.
 */
#define CORE_TELEMETRY_MAX_BUFFER_SIZE (U8_MAX * GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE)
/* Secure telemetry buffer size per core */
#define SECURE_CORE_TELEMETRY_BUFFER_SIZE GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE

struct gxp_core_telemetry_work {
	struct work_struct work;
	struct gxp_dev *gxp;
	uint core;
};

struct gxp_core_telemetry_manager {
	struct buffer_data {
		u32 host_status;
		struct gxp_coherent_buf buffers[GXP_NUM_CORES];
		u32 size;
		refcount_t ref_count;
		bool is_enabled;
	} *buff_data;
	/* Protects buff_data and efd. */
	struct mutex lock;
	struct gxp_core_telemetry_work notification_works[GXP_NUM_CORES];
	wait_queue_head_t waitq;
	struct eventfd_ctx *efd;
	phys_addr_t secure_buffer_carveout_addr;
};

/**
 * gxp_core_telemetry_init() - Initialize telemetry support
 * @gxp: The GXP device to initialize core telemetry support for
 *
 * Return:
 * * 0       - Success
 * * -ENOMEM - Insufficient memory is available to initialize support
 */
int gxp_core_telemetry_init(struct gxp_dev *gxp);

/**
 * gxp_secure_core_telemetry_init() - Initialize telemetry support for
 *                                    secure core workloads.
 * @gxp: The GXP device to initialize secure core telemetry support for.
 * @phys: Physical address of reserved carveout for secure telemetry.
 *
 * Return:
 * * 0       - Success or provided physical address was zero.
 * * -EINVAL - Memremap of provided physical address failed.
 */
int gxp_secure_core_telemetry_init(struct gxp_dev *gxp, phys_addr_t phys);

/**
 * gxp_core_telemetry_mmap_buffers() - Maps the preallocated telemetry
 *                                     buffers to the user-space vma.
 * @gxp: The GXP device to create the buffers for.
 * @vma: The vma from user-space which all cores' buffers will be mapped into.
 *
 * Return:
 * * 0       - Success.
 * * -ENODEV - Core telemetry support has not been initialized. Must explicitly
 *             check this, since this function is called based on user-input.
 * * -EINVAL - The size of @vma does not match telemetry buffer size.
 */
int gxp_core_telemetry_mmap_buffers(struct gxp_dev *gxp, struct vm_area_struct *vma);

/**
 * gxp_secure_core_telemetry_mmap_buffers() - Maps the reserved secure
 *                                            telemetry buffer carveout area.
 * @gxp: The GXP device to create the buffers for.
 * @vma: The vma from user-space which all cores' buffers will be mapped into.
 *
 * Return:
 * * 0       - Success.
 * * -EINVAL - vma size is not equal to reserved carveout area size.
 * * -ENODATA - No secure buffer details available.
 * * otherwise - Error returned by `remap_pfn_range()`
 */
int gxp_secure_core_telemetry_mmap_buffers(struct gxp_dev *gxp,
					   struct vm_area_struct *vma);

/**
 * gxp_core_telemetry_register_eventfd() - Register an eventfd to be signaled when core telemetry
 *                                         notifications arrive.
 * @gxp: The GXP device to register the eventfd for
 * @fd: A file descriptor for an eventfd from user-space
 *
 * If another eventfd has already been registered, the old* eventfd will be unregistered and
 * replaced.
 *
 * Return:
 * * 0       - Success
 * * -EBADF  - @fd is not a valid file descriptor (via `eventfd_ctx_fdget()`)
 * * -EINVAL - @fd is not an eventfd
 */
int gxp_core_telemetry_register_eventfd(struct gxp_dev *gxp, int fd);

/**
 * gxp_core_telemetry_unregister_eventfd() - Unregister and release a reference
 *                                           to a previously registered eventfd
 * @gxp: The GXP device to unregister the eventfd for
 */
void gxp_core_telemetry_unregister_eventfd(struct gxp_dev *gxp);

/**
 * gxp_core_telemetry_get_notification_handler() - Get the notification handler
 *                                                 work for the specified core
 * @gxp: The GXP device to obtain the handler for
 * @core: The physical core number to obtain the handler
 *
 * Return: A pointer to the work_struct for the @core's notification handler if
 *         successful. NULL if core telemetry has not been initialized or @core
 *         is invalid.
 */
struct work_struct *
gxp_core_telemetry_get_notification_handler(struct gxp_dev *gxp, uint core);

/**
 * gxp_core_telemetry_status_notify() - Checks the telemetry status of the
 *                                      specified core and signals the eventfd.
 * @gxp: The GXP device to obtain the handler for
 * @core: The physical core number to obtain the handler
 *
 */
void gxp_core_telemetry_status_notify(struct gxp_dev *gxp, uint core);

/**
 * gxp_core_telemetry_exit() - Reverts gxp_core_telemetry_init() to release the
 *                             resources acquired by core telemetry manager.
 * @gxp: The GXP device to obtain the handler for
 *
 */
void gxp_core_telemetry_exit(struct gxp_dev *gxp);


#endif /* __GXP_CORE_TELEMETRY_H__ */
