/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP driver common internal definitions.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_INTERNAL_H__
#define __GXP_INTERNAL_H__

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/idr.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/rwsem.h>
#include <linux/spinlock.h>

#include <gcip/gcip-resource-accessor.h>
#include <gcip/gcip-thermal.h>
#include <iif/iif-manager.h>

#include "gxp-config.h"
#include "gxp.h"

#define IS_GXP_TEST IS_ENABLED(CONFIG_GXP_TEST)

#define GXP_NAME "gxp"

enum gxp_chip_revision {
	GXP_CHIP_A0,
	GXP_CHIP_B0,
	/* used when the revision is not explicitly specified */
	GXP_CHIP_ANY,
};

/* Holds Client's TPU mailboxes info used during mapping */
struct gxp_tpu_mbx_desc {
	uint phys_core_list;
	size_t cmdq_size, respq_size;
};

/* ioremapped resource */
struct gxp_mapped_resource {
	void __iomem *vaddr;		 /* starting virtual address */
	phys_addr_t paddr;		 /* starting physical address */
	dma_addr_t daddr;		 /* starting device address */
	resource_size_t size;		 /* size in bytes */
};

/* device properties */
struct gxp_dev_prop {
	struct mutex lock; /* protects initialized and opaque */
	bool initialized;
	u8 opaque[GXP_DEV_PROP_SIZE];
};

/* Structure to hold TPU device info */
struct gxp_tpu_dev {
	struct device *dev;
	phys_addr_t mbx_paddr;
};

/* Forward declarations from submodules */
struct gcip_iommu_domain_pool;
struct gcip_iommu_domain;
struct gxp_client;
struct gxp_mailbox_manager;
struct gxp_debug_dump_manager;
struct gxp_dma_manager;
struct gxp_fw_data_manager;
struct gxp_power_manager;
struct gxp_core_telemetry_manager;
struct gxp_soc_data;
struct gxp_thermal_manager;
struct gxp_usage_stats;
struct gxp_power_states;

struct gxp_dev {
	struct device *dev;		 /* platform bus device */
	struct cdev char_dev; /* char device structure */
	dev_t char_dev_no;
	struct dentry *d_entry;		 /* debugfs dir for this device */
	struct gxp_mapped_resource regs; /* ioremapped CSRs */
	struct gxp_mapped_resource lpm_regs; /* ioremapped LPM CSRs, may be equal to @regs */
	struct gxp_mapped_resource mbx[GXP_NUM_MAILBOXES]; /* mailbox CSRs */
	struct gxp_mapped_resource fwbufs[GXP_NUM_CORES]; /* FW carveout */
	struct gxp_mapped_resource fwdatabuf; /* Shared FW data carveout */
	struct gxp_mapped_resource cmu; /* CMU CSRs */
	struct gxp_mailbox_manager *mailbox_mgr;
	struct gxp_power_manager *power_mgr;
	struct gxp_debug_dump_manager *debug_dump_mgr;
	struct gxp_firmware_loader_manager *fw_loader_mgr;
	struct gxp_firmware_manager *firmware_mgr;
	/* SoC-specific data */
	struct gxp_soc_data *soc_data;
	/*
	 * Lock to ensure only one thread at a time is ever calling
	 * `pin_user_pages_fast()` during mapping, otherwise it will fail.
	 */
	struct mutex pin_user_pages_lock;
	/*
	 * Reader/writer lock protecting usage of virtual cores assigned to
	 * physical cores.
	 * A writer is any function creating or destroying a virtual core, or
	 * running or stopping one on a physical core.
	 * A reader is any function making use of or interacting with a virtual
	 * core without starting or stopping it on a physical core.
	 * The fields `core_to_vd[]` and `firmware_running` are also protected
	 * by this lock.
	 */
	struct rw_semaphore vd_semaphore;
	struct gxp_virtual_device *core_to_vd[GXP_NUM_CORES];
	struct gxp_client *debugfs_client;
	struct mutex debugfs_client_lock;
	bool debugfs_wakelock_held;
	struct gxp_dma_manager *dma_mgr;
	struct gxp_fw_data_manager *data_mgr;
	struct gxp_tpu_dev tpu_dev;
	struct gxp_core_telemetry_manager *core_telemetry_mgr;
	struct gcip_iommu_domain *default_domain;
	struct gcip_thermal *thermal;
	struct gcip_devfreq *devfreq;
	/* The accessor to register resources to the debugfs interface. */
	struct gcip_resource_accessor *resource_accessor;
	/*
	 * Pointer to GSA device for firmware authentication.
	 * May be NULL if the chip does not support firmware authentication
	 */
	struct device *gsa_dev;
	struct gcip_iommu_domain_pool *domain_pool;
	struct list_head client_list;
	struct mutex client_list_lock;
	/* Pointer and mutex of secure virtual device */
	struct gxp_virtual_device *secure_vd;
	struct mutex secure_vd_lock;
	/*
	 * Buffer shared across firmware.
	 * Its paddr is 0 if the shared buffer is not available.
	 */
	struct gxp_mapped_resource shared_buf;
	/*
	 * If the @shared_buf is used as split slices, it will keep track of
	 * which indexes of slices are used by ID allocator.
	 */
	struct ida shared_slice_idp;
	struct gxp_usage_stats *usage_stats; /* Stores the usage stats */

	void __iomem *sysreg_shareability; /* sysreg shareability csr base */
	/* Next virtual device ID. */
	atomic_t next_vdid;

	/* To manage DMA fences. */
	struct gcip_dma_fence_manager *gfence_mgr;

	/* To save device properties */
	struct gxp_dev_prop device_prop;

	/* To manage IIF fences. */
	struct iif_manager *iif_mgr;
	struct device *iif_dev;

	/* callbacks for chip-dependent implementations */

	/*
	 * For parsing chip-dependent device tree attributes.
	 *
	 * Called as the first step in the common device probing procedure.
	 *
	 * Do NOT use non-device managed allocations in this function, to
	 * prevent memory leak when the probe procedure fails.
	 *
	 * Return a non-zero value can fail the probe procedure.
	 *
	 * This callback is optional.
	 */
	int (*parse_dt)(struct platform_device *pdev, struct gxp_dev *gxp);
	/*
	 * Called when common device probing procedure is done.
	 *
	 * Return a non-zero value can fail the probe procedure.
	 *
	 * This callback is optional.
	 */
	int (*after_probe)(struct gxp_dev *gxp);
	/*
	 * Called before common device removal procedure.
	 *
	 * This callback is optional.
	 */
	void (*before_remove)(struct gxp_dev *gxp);
	/*
	 * Device ioctl handler for chip-dependent ioctl calls.
	 * Should return -ENOTTY when the ioctl should be handled by common
	 * device ioctl handler.
	 *
	 * This callback is optional.
	 */
	long (*handle_ioctl)(struct file *file, uint cmd, ulong arg);
	/*
	 * Device mmap handler for chip-dependent mmap calls.
	 * Should return -EOPNOTSUPP when the mmap should be handled by common
	 * device mmap handler.
	 *
	 * This callback is optional.
	 */
	int (*handle_mmap)(struct file *file, struct vm_area_struct *vma);
	/*
	 * Called for sending power states request.
	 *
	 * Return a non-zero value can fail the block wakelock acquisition.
	 *
	 * This callback is optional.
	 */
	int (*request_power_states)(struct gxp_client *client,
				    struct gxp_power_states power_states);
	/*
	 * Called when the client acquired the BLOCK wakelock and allocated a virtual device.
	 * The caller will hold @gxp->vd_semaphore for writing.
	 *
	 * Return a non-zero value can fail the block acquiring.
	 *
	 * This callback is optional.
	 */
	int (*after_vd_block_ready)(struct gxp_dev *gxp,
				    struct gxp_virtual_device *vd);
	/*
	 * Called before releasing the BLOCK wakelock or the virtual device.
	 * The caller will hold @gxp->vd_semaphore for writing.
	 *
	 * This callback is optional.
	 */
	void (*before_vd_block_unready)(struct gxp_dev *gxp,
					struct gxp_virtual_device *vd);
	/*
	 * Called in .power_up callback of gcip_pm, after the block is powered.
	 *
	 * This function is called with holding gcip_pm lock.
	 *
	 * Return a non-zero value can fail gcip_pm_get.
	 *
	 * This callback is optional.
	 */
	int (*pm_after_blk_on)(struct gxp_dev *gxp);
	/*
	 * Called in .power_down callback of gcip_pm, before the block is shutdown.
	 *
	 * This function is called with holding gcip_pm lock.
	 *
	 * This callback is optional.
	 */
	void (*pm_before_blk_off)(struct gxp_dev *gxp);
	/*
	 * Called in gxp_map_tpu_mbx_queue(), after the TPU mailbox buffers are mapped.
	 *
	 * This function is called with holding the write lock of @client->semaphore and the read
	 * lock of @gxp->vd_semaphore.
	 *
	 * This callback is optional.
	 */
	int (*after_map_tpu_mbx_queue)(struct gxp_dev *gxp,
				       struct gxp_client *client);
	/*
	 * Called in gxp_unmap_tpu_mbx_queue(), before unmapping the TPU mailbox buffers.
	 *
	 * This function is called with holding the write lock of @client->semaphore.
	 *
	 * This callback is optional.
	 */
	void (*before_unmap_tpu_mbx_queue)(struct gxp_dev *gxp,
					   struct gxp_client *client);
	/*
	 * Called as the first step in gxp_lpm_init(), to perform chip-specific initialization if
	 * any.
	 *
	 * This callback is optional.
	 */
	void (*lpm_init)(struct gxp_dev *gxp);
};

/* GXP device IO functions */

static inline u32 gxp_read_32(struct gxp_dev *gxp, uint reg_offset)
{
	return readl(gxp->regs.vaddr + reg_offset);
}

static inline void gxp_write_32(struct gxp_dev *gxp, uint reg_offset, u32 value)
{
	writel(value, gxp->regs.vaddr + reg_offset);
}

static inline int gxp_acquire_rmem_resource(struct gxp_dev *gxp,
					    struct resource *r, char *phandle)
{
	int ret;
	struct device_node *np;

	np = of_parse_phandle(gxp->dev->of_node, phandle, 0);
	if (IS_ERR_OR_NULL(np))
		return -ENODEV;

	ret = of_address_to_resource(np, 0, r);
	of_node_put(np);

	if (!ret && !IS_ERR_OR_NULL(gxp->resource_accessor))
		gcip_register_accessible_resource(gxp->resource_accessor, r);

	return ret;
}

/*
 * To specify whether AP and DSP cores directly communicate by the core mailboxes.
 * All platform drivers of each chip should implement this.
 */
bool gxp_is_direct_mode(struct gxp_dev *gxp);

/*
 * Returns the chip revision.
 */
enum gxp_chip_revision gxp_get_chip_revision(struct gxp_dev *gxp);

#if IS_GXP_TEST
/*
 * Sets the fake TPU device to @gxp.
 *
 * It takes a refcount of the device which will be released when @gxp destroys.
 */
void gxp_set_fake_tpu_dev(struct gxp_dev *gxp, struct device *tpu_dev, phys_addr_t tpu_mbx_paddr);
#endif /* IS_GXP_TEST */

#endif /* __GXP_INTERNAL_H__ */
