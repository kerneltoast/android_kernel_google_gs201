/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Edge TPU driver common internal definitions.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_INTERNAL_H__
#define __EDGETPU_INTERNAL_H__

#include <linux/printk.h>

#ifdef CONFIG_X86
#include <asm/pgtable_types.h>
#include <asm/set_memory.h>
#endif

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "edgetpu.h"
#include "edgetpu-pm.h"
#include "edgetpu-thermal.h"
#include "edgetpu-usage-stats.h"

#define get_dev_for_logging(etdev)                                                                 \
	((etdev)->etiface && (etdev)->etiface->etcdev ? (etdev)->etiface->etcdev : (etdev)->dev)

#define etdev_err(etdev, fmt, ...) dev_err(get_dev_for_logging(etdev), fmt, ##__VA_ARGS__)
#define etdev_warn(etdev, fmt, ...)                                            \
	dev_warn(get_dev_for_logging(etdev), fmt, ##__VA_ARGS__)
#define etdev_info(etdev, fmt, ...)                                            \
	dev_info(get_dev_for_logging(etdev), fmt, ##__VA_ARGS__)
#define etdev_dbg(etdev, fmt, ...) dev_dbg(get_dev_for_logging(etdev), fmt, ##__VA_ARGS__)
#define etdev_err_ratelimited(etdev, fmt, ...)                                 \
	dev_err_ratelimited(get_dev_for_logging(etdev), fmt, ##__VA_ARGS__)
#define etdev_warn_ratelimited(etdev, fmt, ...)                                \
	dev_warn_ratelimited(get_dev_for_logging(etdev), fmt, ##__VA_ARGS__)
#define etdev_info_ratelimited(etdev, fmt, ...)                                \
	dev_info_ratelimited(get_dev_for_logging(etdev), fmt, ##__VA_ARGS__)
#define etdev_dbg_ratelimited(etdev, fmt, ...)                                 \
	dev_dbg_ratelimited(get_dev_for_logging(etdev), fmt, ##__VA_ARGS__)
#define etdev_warn_once(etdev, fmt, ...)                                       \
	dev_warn_once(get_dev_for_logging(etdev), fmt, ##__VA_ARGS__)

/* The number of TPU tiles in an edgetpu chip */
#define EDGETPU_NTILES	16

/*
 * Common-layer context IDs for non-secure TPU access, translated to chip-
 * specific values in the mmu driver.
 */
enum edgetpu_context_id {
	EDGETPU_CONTEXT_INVALID = -1,
	EDGETPU_CONTEXT_KCI = 0,	/* TPU firmware/kernel ID 0 */
	EDGETPU_CONTEXT_VII_BASE = 1,	/* groups IDs starts from 1 to (EDGETPU_CONTEXTS - 1) */
	/* A bit mask to mark the context is an IOMMU domain token */
	EDGETPU_CONTEXT_DOMAIN_TOKEN = 1 << 30,
};

typedef u64 tpu_addr_t;

struct edgetpu_coherent_mem {
	void *vaddr;		/* kernel VA, no allocation if NULL */
	dma_addr_t dma_addr;	/* DMA handle for downstream IOMMU, if any */
	tpu_addr_t tpu_addr;	/* DMA handle for TPU internal IOMMU, if any */
	u64 host_addr;		/* address mapped on host for debugging */
	u64 phys_addr;		/* physical address, if available */
	size_t size;
#ifdef CONFIG_X86
	bool is_set_uc;		/* memory has been marked uncached on X86 */
#endif
};

struct edgetpu_device_group;
struct edgetpu_p2p_csr_map;
struct edgetpu_remote_dram_map;
struct edgetpu_wakelock;
struct edgetpu_dev_iface;

#define EDGETPU_NUM_PERDIE_EVENTS	2
#define perdie_event_id_to_num(event_id)				      \
	(event_id - EDGETPU_PERDIE_EVENT_LOGS_AVAILABLE)

struct edgetpu_client {
	pid_t pid;
	pid_t tgid;
	/* Reference count */
	refcount_t count;
	/* protects group. */
	struct mutex group_lock;
	/*
	 * The virtual device group this client belongs to. Can be NULL if the
	 * client doesn't belong to any group.
	 */
	struct edgetpu_device_group *group;
	/*
	 * This client is the idx-th member of @group.
	 * It's meaningless if this client doesn't belong to a group.
	 */
	uint idx;
	/* the device opened by this client */
	struct edgetpu_dev *etdev;
	/* the interface from which this client was opened */
	struct edgetpu_dev_iface *etiface;
	/* Peer CSR dma addrs for this client, if in a group with P2P */
	dma_addr_t *p2p_csrs_dma_addrs;
	/* Peer DRAM dma addrs for this client, if has on-device DRAM */
	dma_addr_t *remote_drams_dma_addrs;
	/* Per-client request to keep device active */
	struct edgetpu_wakelock *wakelock;
	/* Bit field of registered per die events */
	u64 perdie_events;
};

/* Configurable parameters for an edgetpu interface */
struct edgetpu_iface_params {
	/*
	 * Interface-specific name.
	 * May be NULL for the default interface (etdev->dev_name will be used)
	 */
	const char *name;
};

/* edgetpu_dev#clients list entry. */
struct edgetpu_list_device_client {
	struct list_head list;
	struct edgetpu_client *client;
};

/* loop through etdev->clients (hold clients_lock prior). */
#define for_each_list_device_client(etdev, c)                                  \
	list_for_each_entry(c, &etdev->clients, list)

struct edgetpu_mapping;
struct edgetpu_mailbox_manager;
struct edgetpu_kci;
struct edgetpu_kci_response_element;
struct edgetpu_telemetry_ctx;
struct edgetpu_mempool;

#define EDGETPU_DEVICE_NAME_MAX	64

/* ioremapped resource */
struct edgetpu_mapped_resource {
	void __iomem *mem;	/* starting virtual address */
	phys_addr_t phys;	/* starting physical address */
	resource_size_t size;	/* size in bytes */
};

enum edgetpu_dev_state {
	ETDEV_STATE_NOFW = 0,	/* no firmware running on device. */
	ETDEV_STATE_GOOD = 1,	/* healthy firmware running. */
	ETDEV_STATE_FWLOADING = 2, /* firmware is getting loaded on device. */
	ETDEV_STATE_BAD = 3,	/* firmware/device is in unusable state. */
};

/* a mark to know whether we read valid versions from the firmware header */
#define EDGETPU_INVALID_KCI_VERSION (~0u)

struct edgetpu_dev {
	struct device *dev;	   /* platform/pci bus device */
	uint num_ifaces;		   /* Number of device interfaces */
	uint num_cores; /* Number of cores */
	/*
	 * Array of device interfaces
	 * First element is the default interface
	 */
	struct edgetpu_dev_iface *etiface;
	char dev_name[EDGETPU_DEVICE_NAME_MAX];
	struct edgetpu_mapped_resource regs; /* ioremapped CSRs */
	struct dentry *d_entry;    /* debugfs dir for this device */
	struct mutex state_lock;   /* protects state of this device */
	enum edgetpu_dev_state state;
	struct mutex groups_lock;
	/* fields protected by @groups_lock */

	struct list_head groups;
	uint n_groups;		   /* number of entries in @groups */
	bool group_join_lockout;   /* disable group join while reinit */
	u32 vcid_pool;		   /* bitmask of VCID to be allocated */

	/* end of fields protected by @groups_lock */

	struct mutex clients_lock; /* protects clients */
	struct list_head clients;
	void *mmu_cookie;	   /* mmu driver private data */
	void *dram_cookie;	   /* on-device DRAM private data */
	struct edgetpu_mailbox_manager *mailbox_manager;
	struct edgetpu_kci *kci;
	struct edgetpu_firmware *firmware; /* firmware management */
	struct edgetpu_telemetry_ctx *telemetry;
	struct edgetpu_thermal *thermal;
	struct edgetpu_usage_stats *usage_stats; /* usage stats private data */
	struct edgetpu_pm *pm;  /* Power management interface */
	/* Memory pool in instruction remap region */
	struct edgetpu_mempool *iremap_pool;
	int mcp_id;		/* multichip pkg id, or -1 for none */
	uint mcp_die_index;	/* physical die index w/in multichip pkg */
	u8 mcp_pkg_type;	/* multichip pkg type */
	struct edgetpu_sw_wdt *etdev_sw_wdt;	/* software watchdog */
	bool reset_needed;	/* error recovery requests full chip reset. */
	/* version read from the firmware binary file */
	struct edgetpu_fw_version fw_version;
	atomic_t job_count;	/* times joined to a device group */

	/* counts of error events */
	uint firmware_crash_count;
	uint watchdog_timeout_count;

	struct mutex freq_lock;	/* protects below freq_* variables */
	uint32_t *freq_table;	/* Array to record reported frequencies by f/w */
	uint32_t freq_count;	/* Number of entries in freq_table */
};

struct edgetpu_dev_iface {
	struct cdev cdev; /* cdev char device structure */
	struct device *etcdev; /* edgetpu class char device */
	struct edgetpu_dev *etdev; /* Pointer to core device struct */
	dev_t devno; /* char device dev_t */
	const char *name; /* interface specific device name */
	struct dentry *d_entry; /* debugfs symlink if not default device name iface */
};

/* Firmware crash_type codes */
enum edgetpu_fw_crash_type {
	EDGETPU_FW_CRASH_ASSERT = 0,
	EDGETPU_FW_CRASH_DATA_ABORT = 1,
	EDGETPU_FW_CRASH_PREFETCH_ABORT = 2,
	EDGETPU_FW_CRASH_UNDEF_EXCEPT = 3,
	EDGETPU_FW_CRASH_UNRECOV_FAULT = 4,
};

extern const struct file_operations edgetpu_fops;

/* Status regs dump. */
struct edgetpu_dumpregs_range {
	u32 firstreg;
	u32 lastreg;
};
extern struct edgetpu_dumpregs_range edgetpu_chip_statusregs_ranges[];
extern int edgetpu_chip_statusregs_nranges;
extern struct edgetpu_dumpregs_range edgetpu_chip_tile_statusregs_ranges[];
extern int edgetpu_chip_tile_statusregs_nranges;

static inline const char *edgetpu_dma_dir_rw_s(enum dma_data_direction dir)
{
	static const char *tbl[4] = { "rw", "r", "w", "?" };

	return tbl[dir];
}

/* edgetpu device IO functions */

static inline u32 edgetpu_dev_read_32(struct edgetpu_dev *etdev,
				      uint reg_offset)
{
	return readl_relaxed(etdev->regs.mem + reg_offset);
}

/* Read 32-bit reg with memory barrier completing before following CPU reads. */
static inline u32 edgetpu_dev_read_32_sync(struct edgetpu_dev *etdev,
					   uint reg_offset)
{
	return readl(etdev->regs.mem + reg_offset);
}

static inline u64 edgetpu_dev_read_64(struct edgetpu_dev *etdev,
				      uint reg_offset)
{
	return readq_relaxed(etdev->regs.mem + reg_offset);
}

static inline void edgetpu_dev_write_32(struct edgetpu_dev *etdev,
					uint reg_offset, u32 value)
{
	writel_relaxed(value, etdev->regs.mem + reg_offset);
}

/* Write 32-bit reg with memory barrier completing CPU writes first. */
static inline void edgetpu_dev_write_32_sync(struct edgetpu_dev *etdev,
					     uint reg_offset, u32 value)
{
	writel(value, etdev->regs.mem + reg_offset);
}

static inline void edgetpu_dev_write_64(struct edgetpu_dev *etdev,
					uint reg_offset, u64 value)
{
	writeq_relaxed(value, etdev->regs.mem + reg_offset);
}

static inline void
edgetpu_x86_coherent_mem_init(struct edgetpu_coherent_mem *mem)
{
#ifdef CONFIG_X86
	mem->is_set_uc = false;
#endif
}

static inline void
edgetpu_x86_coherent_mem_set_uc(struct edgetpu_coherent_mem *mem)
{
#ifdef CONFIG_X86
	if (!mem->is_set_uc) {
		set_memory_uc((unsigned long)mem->vaddr, mem->size >>
			      PAGE_SHIFT);
		mem->is_set_uc = true;
	}
#endif
}

static inline void
edgetpu_x86_coherent_mem_set_wb(struct edgetpu_coherent_mem *mem)
{
#ifdef CONFIG_X86
	if (mem->is_set_uc) {
		set_memory_wb((unsigned long)mem->vaddr, mem->size >>
			      PAGE_SHIFT);
		mem->is_set_uc = false;
	}
#endif
}

/*
 * Attempt to allocate memory from the dma coherent memory using dma_alloc.
 * Use this to allocate memory outside the instruction remap pool.
 */
int edgetpu_alloc_coherent(struct edgetpu_dev *etdev, size_t size,
			   struct edgetpu_coherent_mem *mem,
			   enum edgetpu_context_id context_id);
/*
 * Free memory allocated by the function above from the dma coherent memory.
 */
void edgetpu_free_coherent(struct edgetpu_dev *etdev,
			   struct edgetpu_coherent_mem *mem,
			   enum edgetpu_context_id context_id);

/* Checks if @file belongs to edgetpu driver */
bool is_edgetpu_file(struct file *file);

/* External drivers can hook up to edgetpu driver using these calls. */
int edgetpu_open(struct edgetpu_dev_iface *etiface, struct file *file);
long edgetpu_ioctl(struct file *file, uint cmd, ulong arg);

#if IS_ENABLED(CONFIG_EDGETPU_EXTERNAL_WRAPPER_CLASS)
extern bool edgetpu_is_external_wrapper_class_file(struct file *file);
#else
static inline bool edgetpu_is_external_wrapper_class_file(struct file *file)
{
	return false;
}
#endif

/* Handle firmware crash event */
void edgetpu_handle_firmware_crash(struct edgetpu_dev *etdev,
				   enum edgetpu_fw_crash_type crash_type);

/* Handle notification of job lockup from firmware */
void edgetpu_handle_job_lockup(struct edgetpu_dev *etdev, u16 vcid);

/* Bus (Platform/PCI) <-> Core API */

int __init edgetpu_init(void);
void __exit edgetpu_exit(void);
int edgetpu_device_add(struct edgetpu_dev *etdev,
		       const struct edgetpu_mapped_resource *regs,
		       const struct edgetpu_iface_params *iface_params,
		       uint num_ifaces);
void edgetpu_device_remove(struct edgetpu_dev *etdev);
/* Registers IRQ. */
int edgetpu_register_irq(struct edgetpu_dev *etdev, int irq);
/* Reverts edgetpu_register_irq */
void edgetpu_unregister_irq(struct edgetpu_dev *etdev, int irq);

/* Core -> Device FS API */

int __init edgetpu_fs_init(void);
void __exit edgetpu_fs_exit(void);
int edgetpu_fs_add(struct edgetpu_dev *etdev, const struct edgetpu_iface_params *etiparams,
		   int num_ifaces);

void edgetpu_fs_remove(struct edgetpu_dev *dev);
/* Get the top-level debugfs directory for the device class */
struct dentry *edgetpu_fs_debugfs_dir(void);

/* Core/Device/FS -> Chip API */

/* Chip-specific init/exit */
void edgetpu_chip_init(struct edgetpu_dev *etdev);
void edgetpu_chip_exit(struct edgetpu_dev *etdev);

/* IRQ handler */
irqreturn_t edgetpu_chip_irq_handler(int irq, void *arg);

/*
 * Called from core to chip layer when MMU is needed during device init.
 *
 * Returns 0 on success, otherwise -errno.
 */
int edgetpu_chip_setup_mmu(struct edgetpu_dev *etdev);

/*
 * Reverts edgetpu_chip_setup_mmu().
 * This is called during device removal.
 */
void edgetpu_chip_remove_mmu(struct edgetpu_dev *etdev);

/* Read TPU timestamp */
u64 edgetpu_chip_tpu_timestamp(struct edgetpu_dev *etdev);

/*
 * Handle chip-specific incoming requests from firmware over KCI
 * Note: This will get called from the system's work queue.
 * Code should not block for extended periods of time
 */
void edgetpu_chip_handle_reverse_kci(struct edgetpu_dev *etdev,
				     struct edgetpu_kci_response_element *resp);

/* Device -> Core API */

/* Add current thread as new TPU client */
struct edgetpu_client *
edgetpu_client_add(struct edgetpu_dev_iface *etiface);

/* Remove TPU client */
void edgetpu_client_remove(struct edgetpu_client *client);

/* Handle chip-specific client removal */
void edgetpu_chip_client_remove(struct edgetpu_client *client);

/* mmap() device/queue memory */
int edgetpu_mmap(struct edgetpu_client *client, struct vm_area_struct *vma);

/* Increase reference count of @client. */
struct edgetpu_client *edgetpu_client_get(struct edgetpu_client *client);

/* Decrease reference count and free @client if count reaches zero */
void edgetpu_client_put(struct edgetpu_client *client);

/* Mark die that fails probe to allow bypassing */
void edgetpu_mark_probe_fail(struct edgetpu_dev *etdev);

/*
 * Get error code corresponding to @etdev state. Caller holds
 * etdev->state_lock.
 */
int edgetpu_get_state_errno_locked(struct edgetpu_dev *etdev);

/*
 * "External mailboxes" below refers to mailboxes that are not handled
 * directly by the runtime, such as secure or device-to-device.
 *
 * Chip specific code will typically keep track of state and inform the firmware
 * that a mailbox has become active/inactive.
 */

/* Chip-specific code to acquire external mailboxes */
int edgetpu_chip_acquire_ext_mailbox(struct edgetpu_client *client,
				     struct edgetpu_ext_mailbox_ioctl *args);

/* Chip-specific code to release external mailboxes */
int edgetpu_chip_release_ext_mailbox(struct edgetpu_client *client,
				     struct edgetpu_ext_mailbox_ioctl *args);

/*
 * Chip specific function to get indexes of external mailbox based on
 * @mbox_type
 */
int edgetpu_chip_get_ext_mailbox_index(u32 mbox_type, u32 *start, u32 *end);

#endif /* __EDGETPU_INTERNAL_H__ */
