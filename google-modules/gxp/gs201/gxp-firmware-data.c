// SPDX-License-Identifier: GPL-2.0
/*
 * GXP firmware data manager.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/slab.h>

#include "gxp-config.h"
#include "gxp-debug-dump.h"
#include "gxp-firmware-data.h"
#include "gxp-firmware.h" /* gxp_core_boot */
#include "gxp-host-device-structs.h"
#include "gxp-internal.h"
#include "gxp-vd.h"
#include "gxp.h"

/* A byte pattern to pre-populate the FW region with */
#define FW_DATA_DEBUG_PATTERN 0x66

/* Default application parameters */
#define DEFAULT_APP_ID 1

/*
 * Holds information about system-wide HW and memory resources given to the FWs
 * of GXP devices.
 */
struct gxp_fw_data_manager {
	/* Cached core telemetry descriptors. */
	struct gxp_core_telemetry_descriptor core_telemetry_desc;
	/*
	 * A host-view of the System configuration descriptor. This same desc
	 * is provided to all VDs and all cores. This is the R/O section.
	 */
	struct gxp_system_descriptor_ro *sys_desc_ro;
	/*
	 * A host-view of the System configuration descriptor. This same desc
	 * is provided to all VDs and all cores. This is the R/W section.
	 */
	struct gxp_system_descriptor_rw *sys_desc_rw;
};

/*
 * Here assumes sys_cfg contains gxp_system_descriptor_ro in the first page and
 * gxp_system_descriptor_rw in the second page.
 */
static void set_system_cfg_region(struct gxp_dev *gxp, void *sys_cfg)
{
	struct gxp_system_descriptor_ro *des_ro = sys_cfg;
	struct gxp_system_descriptor_rw *des_rw = sys_cfg + PAGE_SIZE;
	struct gxp_core_telemetry_descriptor *descriptor =
		&gxp->data_mgr->core_telemetry_desc;
	struct telemetry_descriptor_ro *tel_ro;
	struct telemetry_descriptor_rw *tel_rw;
	struct core_telemetry_descriptor *tel_des;
	int i;

	if (gxp->debug_dump_mgr)
		des_ro->debug_dump_dev_addr = gxp->debug_dump_mgr->buf.dsp_addr;
	else
		des_ro->debug_dump_dev_addr = 0;

#define COPY_FIELDS(des, ro, rw)                                               \
	do {                                                                   \
		ro->host_status = des->host_status;                            \
		ro->buffer_addr = des->buffer_addr;                            \
		ro->buffer_size = des->buffer_size;                            \
		rw->device_status = des->device_status;                        \
		rw->data_available = des->watermark_level;                     \
	} while (0)
	for (i = 0; i < GXP_NUM_CORES; i++) {
		tel_ro = &des_ro->telemetry_desc.per_core_loggers[i];
		tel_rw = &des_rw->telemetry_desc.per_core_loggers[i];
		tel_des = &descriptor->per_core_loggers[i];
		COPY_FIELDS(tel_des, tel_ro, tel_rw);
		tel_ro = &des_ro->telemetry_desc.per_core_tracers[i];
		tel_rw = &des_rw->telemetry_desc.per_core_tracers[i];
		tel_des = &descriptor->per_core_tracers[i];
		COPY_FIELDS(tel_des, tel_ro, tel_rw);
	}
#undef COPY_FIELDS

	/* Update the global descriptors. */
	gxp->data_mgr->sys_desc_ro = des_ro;
	gxp->data_mgr->sys_desc_rw = des_rw;
}

static void _gxp_fw_data_populate_vd_cfg(struct gxp_dev *gxp,
					 struct gxp_virtual_device *vd)
{
	struct gxp_host_control_region *core_cfg;
	struct gxp_job_descriptor job;
	struct gxp_vd_descriptor *vd_desc;
	int i;

	if (!gxp_core_boot(gxp)) {
		dev_info(gxp->dev, "Skip setting VD and core CFG");
		return;
	}
	if (!vd->vd_cfg.vaddr || !vd->core_cfg.vaddr) {
		dev_warn(
			gxp->dev,
			"Missing VD and core CFG in image config, firmware is not bootable\n");
		return;
	}
	/* Set up VD config region. */
	vd_desc = vd->vd_cfg.vaddr;
	vd_desc->application_id = DEFAULT_APP_ID;
	vd_desc->vd_is_initialized = 0;
	/* Set up core config region. */
	job.workers_count = vd->num_cores;
	for (i = 0; i < ARRAY_SIZE(job.worker_to_fw); i++) {
		/*
		 * Kernel-initiated workloads always act like the entire VD is
		 * one giant N-core job where N is the number of cores allocated
		 * to that VD.
		 * The MCU, on the other hand, can have multiple jobs dispatched
		 * to the same VD at the same time.
		 */
		if (i < job.workers_count)
			job.worker_to_fw[i] = i;
		else
			job.worker_to_fw[i] = -1;
	}
	/* Give each VD a unique HW resources slot. */
	job.hardware_resources_slot = gxp_vd_hw_slot_id(vd);
	/* Assign the same job descriptor to all cores in this VD */
	for (i = 0; i < GXP_NUM_CORES; i++) {
		core_cfg = vd->core_cfg.vaddr +
			   vd->core_cfg.size / GXP_NUM_CORES * i;
		core_cfg->job_descriptor = job;
	}
}

static struct core_telemetry_descriptor *
gxp_fw_data_get_core_telemetry_descriptor(struct gxp_dev *gxp, u8 type)
{
	struct gxp_core_telemetry_descriptor *descriptor =
		&gxp->data_mgr->core_telemetry_desc;

	if (type == GXP_TELEMETRY_TYPE_LOGGING)
		return descriptor->per_core_loggers;
	else if (type == GXP_TELEMETRY_TYPE_TRACING)
		return descriptor->per_core_tracers;
	else
		return ERR_PTR(-EINVAL);
}

int gxp_fw_data_init(struct gxp_dev *gxp)
{
	struct gxp_fw_data_manager *mgr;
	void *virt;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

	virt = memremap(gxp->fwdatabuf.paddr, gxp->fwdatabuf.size, MEMREMAP_WC);

	if (IS_ERR_OR_NULL(virt)) {
		dev_err(gxp->dev, "Failed to map fw data region\n");
		return -ENODEV;
	}
	gxp->fwdatabuf.vaddr = virt;

	/* Populate the region with a pre-defined pattern. */
	memset(virt, FW_DATA_DEBUG_PATTERN, gxp->fwdatabuf.size);
	gxp->data_mgr = mgr;

	return 0;
}

void gxp_fw_data_destroy(struct gxp_dev *gxp)
{
	struct gxp_fw_data_manager *mgr = gxp->data_mgr;

	if (gxp->fwdatabuf.vaddr)
		memunmap(gxp->fwdatabuf.vaddr);

	devm_kfree(gxp->dev, mgr);
	gxp->data_mgr = NULL;
}

void gxp_fw_data_populate_vd_cfg(struct gxp_dev *gxp, struct gxp_virtual_device *vd)
{
	if (gxp_fw_data_use_per_vd_config(vd))
		_gxp_fw_data_populate_vd_cfg(gxp, vd);
}

int gxp_fw_data_set_core_telemetry_descriptors(struct gxp_dev *gxp, u8 type,
					       u32 host_status,
					       struct gxp_coherent_buf *buffers,
					       u32 per_buffer_size)
{
	struct core_telemetry_descriptor *core_descriptors;
	uint core;
	bool enable;

	core_descriptors = gxp_fw_data_get_core_telemetry_descriptor(gxp, type);
	if (IS_ERR(core_descriptors))
		return PTR_ERR(core_descriptors);

	enable = (host_status & GXP_CORE_TELEMETRY_HOST_STATUS_ENABLED);

	if (enable) {
		/* Validate that the provided IOVAs are addressable (i.e. 32-bit) */
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (buffers && buffers[core].dsp_addr > U32_MAX &&
			    buffers[core].size == per_buffer_size)
				return -EINVAL;
		}

		for (core = 0; core < GXP_NUM_CORES; core++) {
			core_descriptors[core].host_status = host_status;
			core_descriptors[core].buffer_addr =
				(u32)buffers[core].dsp_addr;
			core_descriptors[core].buffer_size = per_buffer_size;
		}
	} else {
		for (core = 0; core < GXP_NUM_CORES; core++) {
			core_descriptors[core].host_status = host_status;
			core_descriptors[core].buffer_addr = 0;
			core_descriptors[core].buffer_size = 0;
		}
	}

	return 0;
}

u32 gxp_fw_data_get_core_telemetry_device_status(struct gxp_dev *gxp, uint core,
						 u8 type)
{
	struct gxp_system_descriptor_rw *des_rw = gxp->data_mgr->sys_desc_rw;

	if (core >= GXP_NUM_CORES)
		return 0;

	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		return des_rw->telemetry_desc.per_core_loggers[core]
			.device_status;
	case GXP_TELEMETRY_TYPE_TRACING:
		return des_rw->telemetry_desc.per_core_tracers[core]
			.device_status;
	default:
		return 0;
	}
}

struct gxp_mapped_resource gxp_fw_data_resource(struct gxp_dev *gxp)
{
	/*
	 * For direct mode, the config regions are programmed by host (us); for
	 * MCU mode, the config regions are programmed by MCU.
	 */
	if (gxp_is_direct_mode(gxp)) {
		struct gxp_mapped_resource tmp = gxp->fwdatabuf;

		/* Leave the first piece be used for gxp_fw_data_init() */
		tmp.vaddr += tmp.size / 2;
		tmp.paddr += tmp.size / 2;
		return tmp;
	} else {
		return gxp->shared_buf;
	}
}

void *gxp_fw_data_system_cfg(struct gxp_dev *gxp)
{
	/* Use the end of the shared region for system cfg. */
	return gxp_fw_data_resource(gxp).vaddr + GXP_SHARED_BUFFER_SIZE -
	       GXP_FW_DATA_SYSCFG_SIZE;
}

void gxp_fw_data_populate_system_config(struct gxp_dev *gxp)
{
	set_system_cfg_region(gxp, gxp_fw_data_system_cfg(gxp));
}
