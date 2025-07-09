/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Edge TPU kernel-userspace interface definitions.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_H__
#define __EDGETPU_H__

#include <linux/ioctl.h>
#include <linux/types.h>

/* mmap offsets for mailbox CSRs, command queue, and response queue */
#define EDGETPU_MMAP_EXT_CSR_OFFSET 0x1500000
#define EDGETPU_MMAP_EXT_CMD_QUEUE_OFFSET 0x1600000
#define EDGETPU_MMAP_EXT_RESP_QUEUE_OFFSET 0x1700000
#define EDGETPU_MMAP_CSR_OFFSET 0x1800000
#define EDGETPU_MMAP_CMD_QUEUE_OFFSET 0x1900000
#define EDGETPU_MMAP_RESP_QUEUE_OFFSET 0x1A00000
/* mmap offsets for logging and tracing buffers */
#define EDGETPU_MMAP_LOG_BUFFER_OFFSET 0x1B00000
#define EDGETPU_MMAP_TRACE_BUFFER_OFFSET 0x1C00000
#define EDGETPU_MMAP_LOG1_BUFFER_OFFSET 0x1D00000
#define EDGETPU_MMAP_TRACE1_BUFFER_OFFSET 0x1E00000

/* EdgeTPU map flag macros */

typedef __u32 edgetpu_map_flag_t;
/* The mask for specifying DMA direction in EdgeTPU map flag */
#define EDGETPU_MAP_DIR_MASK		3
/* The targeted DMA direction for the buffer */
#define EDGETPU_MAP_DMA_BIDIRECTIONAL   0
#define EDGETPU_MAP_DMA_TO_DEVICE       1
#define EDGETPU_MAP_DMA_FROM_DEVICE     2
#define EDGETPU_MAP_DMA_NONE            3
/* The address is mapped to all dies in a device group */
#define EDGETPU_MAP_MIRRORED		(0u << 2)
/* The address is mapped on the specific die */
#define EDGETPU_MAP_NONMIRRORED		(1u << 2)
/* The TPU address must be accessible to the TPU CPU */
#define EDGETPU_MAP_CPU_ACCESSIBLE	(0u << 3)
#define EDGETPU_MAP_CPU_NONACCESSIBLE	(1u << 3)
/* Skip CPU sync on unmap */
#define EDGETPU_MAP_SKIP_CPU_SYNC	(1u << 4)
/* Offset and mask to set the PBHA bits of IOMMU mappings */
#define EDGETPU_MAP_ATTR_PBHA_SHIFT	5
#define EDGETPU_MAP_ATTR_PBHA_MASK	0xf
/* Create coherent mapping of the buffer */
#define EDGETPU_MAP_COHERENT		(1u << 9)

/* External mailbox types */
#define EDGETPU_EXT_MAILBOX_TYPE_TZ		1
#define EDGETPU_EXT_MAILBOX_TYPE_GSA		2

struct edgetpu_map_ioctl {
	__u64 host_address;	/* user-space address to be mapped */
	__u64 size;		/* size of mapping in bytes */
	__u64 device_address;	/* returned TPU VA */
	/*
	 * Flags or'ed with EDGETPU_MAP_*, indicating mapping attribute requests from
	 * the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [1:0]   - DMA_DIRECTION:
	 *               00 = DMA_BIDIRECTIONAL
	 *               01 = DMA_TO_DEVICE
	 *               10 = DMA_FROM_DEVICE
	 *               11 = DMA_NONE
	 *   [2:2]   - Mirroredness. Mirrored across device group or local to a
	 *             specific die:
	 *               0 = map to all dies in a device group
	 *               1 = map to the @die_index-th die of the device group
	 *   [3:3]   - If the TPU address must be accessible to the TPU CPU:
	 *               0 = yes, returned @device_address must be within the
	 *                   address range addressable by the TPU CPU
	 *               1 = no, returned @device_address can be outside the TPU
	 *                   CPU-addressable range
	 *             Note: this flag may be ignored if the TPU chip does not
	 *             have the capability to internally map memory outside the
	 *             CPU-addressable range.
	 *   [4:4]   - Skip cache invalidation on unmap.
	 *               0 = Don't skip CPU sync. Default DMA API behavior.
	 *               1 = Skip CPU sync.
	 *             Note: This bit is ignored on the map call.
	 *   [8:5]   - Value of PBHA bits for IOMMU mappings. For Abrolhos only.
	 *   [9:9]   - Coherent Mapping:
	 *              0 = Create non-coherent mappings of the buffer.
	 *              1 = Create coherent mappings of the buffer.
	 *             Note: this attribute may be ignored on platforms where
	 *             the TPU is not I/O coherent.
	 *   [31:10]  - RESERVED
	 */
	edgetpu_map_flag_t flags;
	/*
	 * Index of die in a device group. The index is decided by the order of
	 * joining the group, with value from zero to (# dies in group) - 1.
	 * Index 0 for the leader die in the group.
	 *
	 * This field is ignored unless EDGETPU_MAP_NONMIRRORED is passed to
	 * @flags.
	 */
	__u32 die_index;
};

#define EDGETPU_IOCTL_BASE 0xED

/*
 * Map a host buffer to TPU.
 *
 * This operation can be performed without acquiring the wakelock. This
 * characteristic holds for all mapping / un-mapping ioctls.
 *
 * On success, @device_address is set, and TPU can access the content of
 * @host_address by @device_address afterwards.
 *
 * EINVAL: If the group is not finalized.
 * EINVAL: If size equals 0.
 * EINVAL: (for EDGETPU_MAP_NONMIRRORED case) If @die_index exceeds the number
 *         of clients in the group.
 * EINVAL: If the target device group is disbanded.
 */
#define EDGETPU_MAP_BUFFER \
	_IOWR(EDGETPU_IOCTL_BASE, 0, struct edgetpu_map_ioctl)

/*
 * Un-map host buffer from TPU previously mapped by EDGETPU_MAP_BUFFER.
 *
 * Only fields @device_address, @die_index, and @flags (see Note) in the third
 * argument will be used, other fields will be fetched from the kernel's
 * internal records. It is recommended to use the argument that was passed in
 * EDGETPU_MAP_BUFFER to un-map the buffer.
 *
 * Note: Only the SKIP_CPU_SYNC flag is considered, other bits in @flags are
 * fetched from the kernel's record.
 *
 * EINVAL: If the requested @device_address is not found.
 */
#define EDGETPU_UNMAP_BUFFER \
	_IOW(EDGETPU_IOCTL_BASE, 4, struct edgetpu_map_ioctl)

/*
 * Event types for which device group eventfds can be registered
 * for notifications.
 */
#define EDGETPU_EVENT_RESPDATA		0
#define EDGETPU_EVENT_FATAL_ERROR	1

struct edgetpu_event_register {
	__u32 event_id;
	__u32 eventfd;
};

/*
 * Set eventfd for notification of events from kernel to the device group.
 *
 * EINVAL: If @event_id is not one of EDGETPU_EVENT_*.
 * EBADF, EINVAL: If @eventfd is not a valid event file descriptor.
 */
#define EDGETPU_SET_EVENTFD \
	_IOW(EDGETPU_IOCTL_BASE, 5, struct edgetpu_event_register)

/*
 * @priority with this bit means the mailbox could be released when wakelock is
 * released.
 */
#define EDGETPU_PRIORITY_DETACHABLE (1u << 3)
/* For @partition_type. */
#define EDGETPU_PARTITION_NORMAL 0
#define EDGETPU_PARTITION_EXTRA 1
struct edgetpu_mailbox_attr {
	/*
	 * There are limitations on these size fields, see the error cases in
	 * EDGETPU_CREATE_GROUP.
	 */

	__u32 cmd_queue_size; /* size of command queue in KB */
	__u32 resp_queue_size; /* size of response queue in KB */
	__u32 sizeof_cmd; /* size of command element in bytes */
	__u32 sizeof_resp; /* size of response element in bytes */
	__u32 priority          : 4; /* mailbox service priority */
	__u32 cmdq_tail_doorbell: 1; /* auto doorbell on cmd queue tail move */
	/* Type of memory partitions to be used for this group, exact meaning is chip-dependent. */
	__u32 partition_type    : 1;
	__u32 client_priv : 1; /* client privilege level */
};

/*
 * Create a new device group with the caller as the master.
 *
 * EINVAL: If the caller already belongs to a group.
 * EINVAL: If @cmd_queue_size or @resp_queue_size equals 0.
 * EINVAL: If @sizeof_cmd or @sizeof_resp equals 0.
 * EINVAL: If @cmd_queue_size * 1024 / @sizeof_cmd >= 1024, this is a hardware
 *         limitation. Same rule for the response sizes pair.
 */
#define EDGETPU_CREATE_GROUP \
	_IOW(EDGETPU_IOCTL_BASE, 6, struct edgetpu_mailbox_attr)

/*
 * Join the calling fd to the device group of the supplied fd.
 *
 * EINVAL: If the caller already belongs to a group.
 * EINVAL: If the supplied FD is not for an open EdgeTPU device file.
 */
#define EDGETPU_JOIN_GROUP \
	_IOW(EDGETPU_IOCTL_BASE, 7, __u32)

/*
 * Finalize the device group with the caller as the leader.
 *
 * EINVAL: If the dies in this group are not allowed to form a device group.
 * ETIMEDOUT: If the handshake with TPU firmware times out.
 */
#define EDGETPU_FINALIZE_GROUP \
	_IO(EDGETPU_IOCTL_BASE, 8)

/*
 * Event types for which per-die eventfds can be registered for
 * notifications.
 */
#define EDGETPU_PERDIE_EVENT_LOGS_AVAILABLE		0x1000
#define EDGETPU_PERDIE_EVENT_TRACES_AVAILABLE		0x1001

/*
 * Set eventfd for notification of per-die events from kernel.
 *
 * EINVAL: If @event_id is not one of EDGETPU_PERDIE_EVENT_*.
 * EBADF, EINVAL: If @eventfd is not a valid eventfd.
 */
#define EDGETPU_SET_PERDIE_EVENTFD \
	_IOW(EDGETPU_IOCTL_BASE, 9, struct edgetpu_event_register)

/* Unset event by event_id registered with EDGETPU_SET_EVENTFD. */
#define EDGETPU_UNSET_EVENT \
	_IOW(EDGETPU_IOCTL_BASE, 14, __u32)

/* Unset event by event_id registered with EDGETPU_SET_PERDIE_EVENTFD. */
#define EDGETPU_UNSET_PERDIE_EVENT \
	_IOW(EDGETPU_IOCTL_BASE, 15, __u32)

#define EDGETPU_SYNC_FOR_DEVICE		(0 << 2)
#define EDGETPU_SYNC_FOR_CPU		(1 << 2)

struct edgetpu_sync_ioctl {
	/*
	 * The starting address of the buffer to be synchronized. Must be a
	 * device address returned by EDGETPU_MAP_BUFFER.
	 */
	__u64 device_address;
	/* Size in bytes to be sync'ed. */
	__u64 size;
	/*
	 * Offset in bytes at which the sync operation is to begin from the
	 * start of the buffer.
	 */
	__u64 offset;
	/*
	 * The die index passed to EDGETPU_MAP_BUFFER if it was an
	 * EDGETPU_MAP_NONMIRRORED request, otherwise this field is ignored.
	 */
	__u32 die_index;
	/*
	 * Flags indicating sync operation requested from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [1:0]   - DMA_DIRECTION:
	 *               00 = DMA_BIDIRECTIONAL
	 *               01 = DMA_TO_DEVICE
	 *               10 = DMA_FROM_DEVICE
	 *               11 = DMA_NONE
	 *   [2:2]   - Sync direction. Sync for device or CPU.
	 *               0 = sync for device
	 *               1 = sync for CPU
	 *   [31:3]  - RESERVED
	 */
	__u32 flags;
};

/*
 * Sync the buffer previously mapped by EDGETPU_MAP_BUFFER.
 *
 * EINVAL: If a mapping for @device_address is not found.
 * EINVAL: If @size equals 0.
 * EINVAL: If @offset plus @size exceeds the mapping size.
 * EINVAL: If the target device group is disbanded.
 */
#define EDGETPU_SYNC_BUFFER \
	_IOW(EDGETPU_IOCTL_BASE, 16, struct edgetpu_sync_ioctl)

struct edgetpu_map_dmabuf_ioctl {
	/* Ignored. */
	__u64 offset;
	/* Ignored; the entire dma-buf is mapped. */
	__u64 size;
	/*
	 * Returned TPU VA.
	 *
	 * The address is page-aligned.
	 */
	__u64 device_address;
	/* A dma-buf FD. */
	__s32 dmabuf_fd;
	/*
	 * Flags indicating mapping attributes. See edgetpu_map_ioctl.flags for
	 * details.
	 *
	 * Note: the SKIP_CPU_SYNC and PBHA flags are ignored, DMA flags to be
	 * used is controlled by the dma-buf exporter.
	 */
	edgetpu_map_flag_t flags;
	/*
	 * Index of die in a device group. See edgetpu_map_ioctl.die_index for
	 * details.
	 */
	__u32 die_index;
};

/*
 * Map the dma-buf FD to TPU.
 *
 * On success, @device_address is set and the syscall returns zero.
 *
 * EINVAL: If @offset is not page-aligned.
 * EINVAL: (for EDGETPU_MAP_NONMIRRORED case) If @die_index exceeds the number
 *         of clients in the group.
 * EINVAL: If the target device group is disbanded.
 */
#define EDGETPU_MAP_DMABUF \
	_IOWR(EDGETPU_IOCTL_BASE, 17, struct edgetpu_map_dmabuf_ioctl)
/*
 * Un-map address previously mapped by EDGETPU_MAP_DMABUF.
 *
 * Only fields @die_index and @device_address in the third argument will be
 * used, other fields will be fetched from the kernel's internal records. If the
 * buffer was requested as EDGETPU_MAP_MIRRORED, @die_index is ignored as well.
 *
 * EINVAL: If @device_address is not found.
 * EINVAL: If the target device group is disbanded.
 */
#define EDGETPU_UNMAP_DMABUF \
	_IOW(EDGETPU_IOCTL_BASE, 18, struct edgetpu_map_dmabuf_ioctl)

/*
 * Allocate device buffer of provided @size(__u64) and
 * return a dma-buf FD on success.
 *
 * EINVAL: If @size is zero.
 * ENODEV: If the on-device DRAM is not supported or failed on initialization.
 * ENOTTY: If config EDGETPU_DEVICE_DRAM is disabled.
 */
#define EDGETPU_ALLOCATE_DEVICE_BUFFER \
	_IOW(EDGETPU_IOCTL_BASE, 19, __u64)

/*
 * struct edgetpu_create_sync_fence_data
 * @seqno:		the seqno to initialize the fence with
 * @timeline_name:	the name of the timeline the fence belongs to
 * @fence:		returns the fd of the new sync_file with the new fence
 *
 * Timeline names can be up to 128 characters (including trailing NUL byte)
 * for edgetpu debugfs and kernel debug logs.  These names are truncated to
 * 32 characters in the data returned by the standard SYNC_IOC_FILE_INFO
 * ioctl.
 */
#define EDGETPU_SYNC_TIMELINE_NAME_LEN	128
struct edgetpu_create_sync_fence_data {
	__u32 seqno;
	char  timeline_name[EDGETPU_SYNC_TIMELINE_NAME_LEN];
	__s32 fence;
};

/*
 * Create a DMA sync fence, return the sync_file fd for the new fence.
 */
#define EDGETPU_CREATE_SYNC_FENCE \
	_IOWR(EDGETPU_IOCTL_BASE, 20, struct edgetpu_create_sync_fence_data)

/*
 * struct edgetpu_signal_sync_fence_data
 * @fence:		fd of the sync_file for the fence
 * @error:		error status errno value or zero for success
 */
struct edgetpu_signal_sync_fence_data {
	__s32 fence;
	__s32 error;
};

/*
 * Signal a DMA sync fence with optional error status.
 * Can pass a sync_file fd created by any driver.
 * Signals the first DMA sync fence in the sync file.
 */
#define EDGETPU_SIGNAL_SYNC_FENCE \
	_IOW(EDGETPU_IOCTL_BASE, 21, struct edgetpu_signal_sync_fence_data)

#define EDGETPU_IGNORE_FD (-1)
#define EDGETPU_MAX_NUM_DEVICES_IN_GROUP 36
struct edgetpu_map_bulk_dmabuf_ioctl {
	__u64 size; /* Size to be mapped in bytes. */
	__u64 device_address; /* returned TPU VA */
	/*
	 * Same format as edgetpu_map_dmabuf_ioctl.flags, except:
	 *   - [2:2] Mirroredness is ignored.
	 */
	edgetpu_map_flag_t flags;
	/*
	 * The list of file descriptors backed by dma-buf.
	 *
	 * The first FD will be mapped to the first device in the target group
	 * (i.e. the master die); the second FD will be mapped to the second
	 * device and so on.
	 * Only the first N FDs will be used, where N is the number of devices
	 * in the group.
	 *
	 * Use EDGETPU_IGNORE_FD if it's not required to map on specific
	 * device(s). For example, if one passes {fd0, EDGETPU_IGNORE_FD, fd2}
	 * to this field for mapping a group with 3 devices, only the first
	 * device and the third device has the mapping on @device_address.
	 */
	__s32 dmabuf_fds[EDGETPU_MAX_NUM_DEVICES_IN_GROUP];
};

/*
 * Map a list of dma-buf FDs to devices in the group.
 *
 * On success, @device_address is set and the syscall returns zero.
 *
 * EINVAL: If @size is zero.
 * EINVAL: If the target device group is not finalized.
 * EINVAL: If any file descriptor is not backed by dma-buf.
 * EINVAL: If @size exceeds the size of any buffer.
 * EINVAL: If all file descriptors are EDGETPU_IGNORE_FD.
 */
#define EDGETPU_MAP_BULK_DMABUF \
	_IOWR(EDGETPU_IOCTL_BASE, 22, struct edgetpu_map_bulk_dmabuf_ioctl)
/*
 * Un-map address previously mapped by EDGETPU_MAP_BULK_DMABUF.
 *
 * Only field @device_address in the third argument is used, other fields such
 * as @size will be fetched from the kernel's internal records.
 *
 * EINVAL: If @device_address is not found.
 * EINVAL: If the target device group is disbanded.
 */
#define EDGETPU_UNMAP_BULK_DMABUF \
	_IOW(EDGETPU_IOCTL_BASE, 23, struct edgetpu_map_bulk_dmabuf_ioctl)

/*
 * struct edgetpu_sync_fence_status
 * @fence:		fd of the sync_file for the fence
 * @status:		returns:
 *			   0 if active
 *			   1 if signaled with no error
 *			   negative errno value if signaled with error
 */
struct edgetpu_sync_fence_status {
	__s32 fence;
	__s32 status;
};

/*
 * Retrieve DMA sync fence status.
 * Can pass a sync_file fd created by any driver.
 * Returns the status of the first DMA sync fence in the sync file.
 */
#define EDGETPU_SYNC_FENCE_STATUS \
	_IOWR(EDGETPU_IOCTL_BASE, 24, struct edgetpu_sync_fence_status)

/*
 * Release the current client's wakelock, allowing firmware to be shut down if
 * no other clients are active.
 * Groups and buffer mappings are preserved.
 *
 * Some mmap operations (listed below) are not allowed when the client's
 * wakelock is released. And if the runtime is holding the mmap'ed buffers, this
 * ioctl returns EAGAIN and the wakelock is not released.
 *   - EDGETPU_MMAP_CSR_OFFSET
 *   - EDGETPU_MMAP_CMD_QUEUE_OFFSET
 *   - EDGETPU_MMAP_RESP_QUEUE_OFFSET
 */
#define EDGETPU_RELEASE_WAKE_LOCK	_IO(EDGETPU_IOCTL_BASE, 25)

/*
 * Acquire the wakelock for this client, ensures firmware keeps running.
 */
#define EDGETPU_ACQUIRE_WAKE_LOCK	_IO(EDGETPU_IOCTL_BASE, 26)

struct edgetpu_fw_version {
	__u32 major_version; /* Returned firmware major version number */
	__u32 minor_version; /* Returned firmware minor version number */
	__u32 vii_version; /* Returned firmware VII version number */
	__u32 kci_version; /* Returned firmware KCI version number */
};

/*
 * Query the version information of the firmware currently loaded.
 *
 * When there is an attempt to load firmware, its version numbers are recorded
 * by the kernel and will be returned on the following EDGETPU_FIRMWARE_VERSION
 * calls. If the latest firmware attempted to load didn't exist or had an
 * invalid header, this call returns ENODEV.
 */
#define EDGETPU_FIRMWARE_VERSION \
	_IOR(EDGETPU_IOCTL_BASE, 27, struct edgetpu_fw_version)

/*
 * Read TPU reference clock / timestamp.  Value is a count of ticks at a
 * chip-specific frequency.
 *
 * Returns EAGAIN if TPU is powered down, that is, the client does not hold a
 * wakelock.
 */
#define EDGETPU_GET_TPU_TIMESTAMP \
	_IOR(EDGETPU_IOCTL_BASE, 28, __u64)

/*
 * struct edgetpu_device_dram_usage
 * @allocated:		size of allocated dram in bytes
 * @available:		size of free device dram in bytes
 */
struct edgetpu_device_dram_usage {
	__u64 allocated;
	__u64 available;
};

/*
 * Query the allocated and free device DRAM.
 *
 * @available and @allocated are set to 0 for chips without a device DRAM.
 */
#define EDGETPU_GET_DRAM_USAGE \
	_IOR(EDGETPU_IOCTL_BASE, 29, struct edgetpu_device_dram_usage)

/*
 * struct edgetpu_ext_mailbox_ioctl
 * @client_id:		Client identifier (may not be needed depending on type)
 * @attrs:		Array of mailbox attributes (pointer to
 *			edgetpu_mailbox_attr, may be NULL depending on type)
 * @type:		One of the EDGETPU_EXT_MAILBOX_xxx values
 * @count:		Number of mailboxes to acquire
 */
struct edgetpu_ext_mailbox_ioctl {
	__u64 client_id;
	__u64 attrs;
	__u32 type;
	__u32 count;
};

/*
 * Acquire a chip-specific mailbox that is not directly managed by the TPU
 * runtime. This can be a secure mailbox or a device-to-device mailbox.
 */
#define EDGETPU_ACQUIRE_EXT_MAILBOX \
	_IOW(EDGETPU_IOCTL_BASE, 30, struct edgetpu_ext_mailbox_ioctl)

/*
 * Release a chip-specific mailbox that is not directly managed by the TPU
 * runtime. This can be a secure mailbox or a device-to-device mailbox.
 */
#define EDGETPU_RELEASE_EXT_MAILBOX \
	_IOW(EDGETPU_IOCTL_BASE, 31, struct edgetpu_ext_mailbox_ioctl)

/* Fatal error event bitmasks... */
/* Firmware crash in non-restartable thread */
#define EDGETPU_ERROR_FW_CRASH		0x1
/* Host or device watchdog timeout */
#define EDGETPU_ERROR_WATCHDOG_TIMEOUT	0x2
/* Thermal shutdown */
#define EDGETPU_ERROR_THERMAL_STOP	0x4
/* TPU hardware inaccessible: link fail, memory protection unit blocking... */
#define EDGETPU_ERROR_HW_NO_ACCESS	0x8
/* Various hardware failures */
#define EDGETPU_ERROR_HW_FAIL		0x10
/* Firmware-reported timeout on runtime processing of workload */
#define EDGETPU_ERROR_RUNTIME_TIMEOUT	0x20

/*
 * Return fatal errors raised for the client's device group, as a bitmask of
 * the above fatal error event codes, or zero if no errors encountered or
 * client is not part of a device group.
 */
#define EDGETPU_GET_FATAL_ERRORS \
	_IOR(EDGETPU_IOCTL_BASE, 32, __u32)

/* The size of device properties pre-agreed with firmware */
#define EDGETPU_DEV_PROP_SIZE 256
/*
 * struct edgetpu_set_device_properties_ioctl
 * @opaque:		Device properties defined by runtime and firmware.
 */
struct edgetpu_set_device_properties_ioctl {
	__u8 opaque[EDGETPU_DEV_PROP_SIZE];
};

/* Registers device properties which will be passed down to firmware on boot. */
#define EDGETPU_SET_DEVICE_PROPERTIES                                                              \
	_IOW(EDGETPU_IOCTL_BASE, 34, struct edgetpu_set_device_properties_ioctl)

#endif /* __EDGETPU_H__ */
