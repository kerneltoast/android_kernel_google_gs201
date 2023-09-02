/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP kernel-userspace interface definitions.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_H__
#define __GXP_H__

#include <linux/ioctl.h>
#include <linux/types.h>

/* Interface Version */
#define GXP_INTERFACE_VERSION_MAJOR	1
#define GXP_INTERFACE_VERSION_MINOR	3
#define GXP_INTERFACE_VERSION_BUILD	0

/*
 * mmap offsets for logging and tracing buffers
 * Requested size will be divided evenly among all cores. The whole buffer
 * must be page-aligned, and the size of each core's buffer must be a multiple
 * of PAGE_SIZE.
 */
#define GXP_MMAP_LOG_BUFFER_OFFSET	0x10000
#define GXP_MMAP_TRACE_BUFFER_OFFSET	0x20000

#define GXP_IOCTL_BASE 0xEE

#define GXP_INTERFACE_VERSION_BUILD_BUFFER_SIZE 64
struct gxp_interface_version_ioctl {
	/*
	 * Driver major version number.
	 * Increments whenever a non-backwards compatible change to the
	 * interface defined in this file changes.
	 */
	__u16 version_major;
	/*
	 * Driver minor version number.
	 * Increments whenever a backwards compatible change, such as the
	 * addition of a new IOCTL, is made to the interface defined in this
	 * file.
	 */
	__u16 version_minor;
	/*
	 * Driver build identifier.
	 * NULL-terminated string of the git hash of the commit the driver was
	 * built from. If the driver had uncommitted changes the string will
	 * end with "-dirty".
	 */
	char version_build[GXP_INTERFACE_VERSION_BUILD_BUFFER_SIZE];
};

/* Query the driver's interface version. */
#define GXP_GET_INTERFACE_VERSION                                              \
	_IOR(GXP_IOCTL_BASE, 26, struct gxp_interface_version_ioctl)

struct gxp_specs_ioctl {
	/* Maximum number of cores that can be allocated to a virtual device */
	__u8 core_count;
	/* Deprecated fields that should be ignored */
	__u16 reserved_0;
	__u16 reserved_1;
	__u16 reserved_2;
	__u8 reserved_3;
	/*
	 * Amount of "tightly-coupled memory" or TCM available to each core.
	 * The value returned will be in kB, or 0 if the value was not
	 * specified in the device-tree.
	 */
	__u32 memory_per_core;
};

/* Query system specs. */
#define GXP_GET_SPECS \
	_IOR(GXP_IOCTL_BASE, 5, struct gxp_specs_ioctl)

struct gxp_virtual_device_ioctl {
	/*
	 * Input:
	 * The number of cores requested for the virtual device.
	 */
	__u8 core_count;
	/*
	 * Input:
	 * The number of threads requested per core.
	 */
	__u16 threads_per_core;
	/*
	 * Input:
	 * The amount of memory requested per core, in kB.
	 */
	__u32 memory_per_core;
	/*
	 * Output:
	 * The ID assigned to the virtual device and shared with its cores.
	 */
	__u32 vdid;
};

/* Allocate virtual device. */
#define GXP_ALLOCATE_VIRTUAL_DEVICE \
	_IOWR(GXP_IOCTL_BASE, 6, struct gxp_virtual_device_ioctl)

/*
 * Components for which a client may hold a wakelock.
 * Acquired by passing these values as `components_to_wake` in
 * `struct gxp_acquire_wakelock_ioctl` to GXP_ACQUIRE_WAKELOCK and released by
 * passing these values directly as the argument to GXP_RELEASE_WAKELOCK.
 *
 * Multiple wakelocks can be acquired or released at once by passing multiple
 * components, ORed together.
 */
#define WAKELOCK_BLOCK		(1 << 0)
#define WAKELOCK_VIRTUAL_DEVICE	(1 << 1)

/*
 * DSP subsystem Power state values for use as `gxp_power_state` in
 * `struct gxp_acquire_wakelock_ioctl`.
 * Note: GXP_POWER_STATE_READY is a deprecated state. The way to achieve
 * original state is to request GXP_POWER_STATE_UUD with setting
 * GXP_POWER_LOW_FREQ_CLKMUX flag. Requesting GXP_POWER_STATE_READY is treated
 * as identical to GXP_POWER_STATE_UUD.
 */
#define GXP_POWER_STATE_OFF	0
#define GXP_POWER_STATE_UUD	1
#define GXP_POWER_STATE_SUD	2
#define GXP_POWER_STATE_UD	3
#define GXP_POWER_STATE_NOM	4
#define GXP_POWER_STATE_READY	5
#define GXP_POWER_STATE_UUD_PLUS 6
#define GXP_POWER_STATE_SUD_PLUS 7
#define GXP_POWER_STATE_UD_PLUS 8
#define GXP_NUM_POWER_STATES (GXP_POWER_STATE_UD_PLUS + 1)

/*
 * Memory interface power state values for use as `memory_power_state` in
 * `struct gxp_acquire_wakelock_ioctl`.
 */
#define MEMORY_POWER_STATE_UNDEFINED	0
#define MEMORY_POWER_STATE_MIN		1
#define MEMORY_POWER_STATE_VERY_LOW	2
#define MEMORY_POWER_STATE_LOW		3
#define MEMORY_POWER_STATE_HIGH		4
#define MEMORY_POWER_STATE_VERY_HIGH	5
#define MEMORY_POWER_STATE_MAX		6

/*
 * GXP power flag macros, supported by `flags` in `gxp_acquire_wakelock_ioctl`
 * and `power_flags in `gxp_mailbox_command_ioctl`.
 *
 * Non-aggressor flag is deprecated. Setting this flag is a no-op since
 * non-aggressor support is defeatured.
 */
#define GXP_POWER_NON_AGGRESSOR		(1 << 0)
/*
 * The client can request low frequency clkmux vote by this flag, which means
 * the kernel driver will switch the CLKMUX clocks to save more power.
 *
 * Note: The kernel driver keep separate track of low frequency clkmux votes
 * and normal votes, and the low frequency clkmux votes will have lower priority
 * than all normal votes.
 * For example, if the kerenl driver has two votes, one is GXP_POWER_STATE_UUD
 * without GXP_POWER_LOW_FREQ_CLKMUX, and the other one is GXP_POWER_STATE_NOM
 * with GXP_POWER_LOW_FREQ_CLKMUX. The voting result is GXP_POWER_STATE_UUD
 * without GXP_POWER_LOW_FREQ_CLKMUX.
 */
#define GXP_POWER_LOW_FREQ_CLKMUX       (1 << 1)

struct gxp_acquire_wakelock_ioctl {
	/*
	 * The components for which a wakelock will be acquired.
	 * Should be one of WAKELOCK_BLOCK or WAKELOCK_VIRTUAL_DEVICE, or a
	 * bitwise OR of both.
	 *
	 * A VIRTUAL_DEVICE wakelock cannot be acquired until the client has
	 * allocated a virtual device. To acquire a VIRTUAL_DEVICE wakelock, a
	 * client must already have acquired a BLOCK wakelock or acquire both
	 * in the same call.
	 */
	__u32 components_to_wake;
	/*
	 * Minimum power state to operate the entire DSP subsystem at until
	 * the BLOCK wakelock is released. One of the GXP_POWER_STATE_* defines
	 * from above. Note that the requested power state will not be cleared
	 * if only the VIRTUAL_DEVICE wakelock is released.
	 *
	 * `GXP_POWER_STATE_OFF` is not a valid value when acquiring a
	 * wakelock.
	 */
	__u32 gxp_power_state;
	/*
	 * Memory interface power state to request from the system so long as
	 * the BLOCK wakelock is held. One of the MEMORY_POWER_STATE* defines
	 * from above. The requested memory power state will not be cleared if
	 * only the VIRTUAL_DEVICE wakelock is released.
	 *
	 * If `MEMORY_POWER_STATE_UNDEFINED` is passed, no request to change
	 * the memory interface power state will be made.
	 */
	__u32 memory_power_state;
	/*
	 * How long to wait, in microseconds, before returning if insufficient
	 * physical cores are available when attempting to acquire a
	 * VIRTUAL_DEVICE wakelock. A value of 0 indicates that the IOCTL
	 * should not wait at all if cores are not available.
	 */
	__u32 vd_timeout_us;
	/*
	 * Flags indicating power attribute requests from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [0:0]   - Deprecated, do not use
	 *   [1:1]   - LOW_FREQ_CLKMUX setting for power management
	 *		 0 = Don't switch CLKMUX clocks, default value
	 *		 1 = Switch CLKMUX clocks
	 *   [31:2]  - RESERVED
	 */
	__u32 flags;
};

/*
 * Acquire a wakelock and request minimum power states for the DSP subsystem
 * and the memory interface.
 *
 * Upon a successful return, the specified components will be powered on and if
 * they were not already running at the specified or higher power states,
 * requests will have been sent to transition both the DSP subsystem and
 * memory interface to the specified states.
 *
 * If the same client invokes this IOCTL for the same component more than once
 * without a corresponding call to `GXP_RELEASE_WAKE_LOCK` in between, the
 * second call will update requested power states, but have no other effects.
 * No additional call to `GXP_RELEASE_WAKE_LOCK` will be required.
 *
 * If a client attempts to acquire a VIRTUAL_DEVICE wakelock and there are
 * insufficient physical cores available, the driver will wait up to
 * `vd_timeout_us` microseconds, then return -EBUSY if sufficient cores were
 * never made available. In this case, if both BLOCK and VIRTUAL_DEVICE
 * wakelocks were being requested, neither will have been acquired.
 */
#define GXP_ACQUIRE_WAKE_LOCK                                                  \
	_IOW(GXP_IOCTL_BASE, 25, struct gxp_acquire_wakelock_ioctl)

/*
 * Legacy "acquire wakelock" IOCTL that does not support power flags.
 * This IOCTL exists for backwards compatibility with older runtimes. All other
 * fields are the same as in `struct gxp_acquire_wakelock_ioctl`.
 */
struct gxp_acquire_wakelock_compat_ioctl {
	__u32 components_to_wake;
	__u32 gxp_power_state;
	__u32 memory_power_state;
	__u32 vd_timeout_us;
};

#define GXP_ACQUIRE_WAKE_LOCK_COMPAT                                           \
	_IOW(GXP_IOCTL_BASE, 18, struct gxp_acquire_wakelock_compat_ioctl)

/*
 * Release a wakelock acquired via `GXP_ACQUIRE_WAKE_LOCK`.
 *
 * The argument should be one of WAKELOCK_BLOCK or WAKELOCK_VIRTUAL_DEVICE, or a
 * bitwise OR of both.
 *
 * Upon releasing a VIRTUAL_DEVICE wakelock, a client's virtual device will be
 * removed from physical cores. At that point the cores may be reallocated to
 * another client or powered down.
 *
 * If no clients hold a BLOCK wakelock, the entire DSP subsytem may be powered
 * down. If a client attempts to release a BLOCK wakelock while still holding
 * a VIRTUAL_DEVICE wakelock, this IOCTL will return -EBUSY.
 *
 * If a client attempts to release a wakelock it does not hold, this IOCTL will
 * return -ENODEV.
 */
#define GXP_RELEASE_WAKE_LOCK _IOW(GXP_IOCTL_BASE, 19, __u32)

/* GXP map flag macros */
/* The mask for specifying DMA direction in GXP map flag */
#define GXP_MAP_DIR_MASK		3
/* The targeted DMA direction for the buffer */
#define GXP_MAP_DMA_BIDIRECTIONAL	0
#define GXP_MAP_DMA_TO_DEVICE		1
#define GXP_MAP_DMA_FROM_DEVICE		2

struct gxp_map_ioctl {
	/*
	 * Bitfield indicating which virtual cores to map the buffer for.
	 * To map for virtual core X, set bit X in this field, i.e. `1 << X`.
	 *
	 * This field is not used by the unmap IOCTL, which always unmaps a
	 * buffer for all cores it had been mapped for.
	 */
	__u16 virtual_core_list;
	__u64 host_address;	/* virtual address in the process space */
	__u32 size;		/* size of mapping in bytes */
	/*
	 * Flags indicating mapping attribute requests from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [1:0]   - DMA_DIRECTION:
	 *               00 = DMA_BIDIRECTIONAL (host/device can write buffer)
	 *               01 = DMA_TO_DEVICE     (host can write buffer)
	 *               10 = DMA_FROM_DEVICE   (device can write buffer)
	 *             Note: DMA_DIRECTION is the direction in which data moves
	 *             from the host's perspective.
	 *   [31:2]  - RESERVED
	 */
	__u32 flags;
	__u64 device_address;	/* returned device address */
};

/*
 * Map host buffer.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_MAP_BUFFER \
	_IOWR(GXP_IOCTL_BASE, 0, struct gxp_map_ioctl)

/*
 * Un-map host buffer previously mapped by GXP_MAP_BUFFER.
 *
 * Only the @device_address field will be used. Other fields will be fetched
 * from the kernel's internal records. It is recommended to use the argument
 * that was passed in GXP_MAP_BUFFER to un-map the buffer.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_UNMAP_BUFFER \
	_IOW(GXP_IOCTL_BASE, 1, struct gxp_map_ioctl)

/* GXP sync flag macros */
#define GXP_SYNC_FOR_DEVICE		(0)
#define GXP_SYNC_FOR_CPU		(1)

struct gxp_sync_ioctl {
	/*
	 * The starting address of the buffer to be synchronized. Must be a
	 * device address returned by GXP_MAP_BUFFER.
	 */
	__u64 device_address;
	/* size in bytes to be sync'ed */
	__u32 size;
	/*
	 * offset in bytes at which the sync operation is to begin from the
	 * start of the buffer
	 */
	__u32 offset;
	/*
	 * Flags indicating sync operation requested from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [0:0]   - Sync direction. Sync for device or CPU.
	 *               0 = sync for device
	 *               1 = sync for CPU
	 *   [31:1]  - RESERVED
	 */
	__u32 flags;
};

/*
 * Sync buffer previously mapped by GXP_MAP_BUFFER.
 *
 * The client must have allocated a virtual device.
 *
 * EINVAL: If a mapping for @device_address is not found.
 * EINVAL: If @size equals 0.
 * EINVAL: If @offset plus @size exceeds the mapping size.
 */
#define GXP_SYNC_BUFFER \
	_IOW(GXP_IOCTL_BASE, 2, struct gxp_sync_ioctl)

struct gxp_map_dmabuf_ioctl {
	/*
	 * Bitfield indicating which virtual cores to map the dma-buf for.
	 * To map for virtual core X, set bit X in this field, i.e. `1 << X`.
	 *
	 * This field is not used by the unmap dma-buf IOCTL, which always
	 * unmaps a dma-buf for all cores it had been mapped for.
	 */
	__u16 virtual_core_list;
	__s32 dmabuf_fd;	/* File descriptor of the dma-buf to map. */
	/*
	 * Flags indicating mapping attribute requests from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [1:0]   - DMA_DIRECTION:
	 *               00 = DMA_BIDIRECTIONAL (host/device can write buffer)
	 *               01 = DMA_TO_DEVICE     (host can write buffer)
	 *               10 = DMA_FROM_DEVICE   (device can write buffer)
	 *             Note: DMA_DIRECTION is the direction in which data moves
	 *             from the host's perspective.
	 *   [31:2]  - RESERVED
	 */
	__u32 flags;
	/*
	 * Device address the dmabuf is mapped to.
	 * - GXP_MAP_DMABUF uses this field to return the address the dma-buf
	 *   can be accessed from by the device.
	 * - GXP_UNMAP_DMABUF expects this field to contain the value from the
	 *   mapping call, and uses it to determine which dma-buf to unmap.
	 */
	__u64 device_address;
};

/*
 * Map host buffer via its dma-buf FD.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_MAP_DMABUF _IOWR(GXP_IOCTL_BASE, 20, struct gxp_map_dmabuf_ioctl)

/*
 * Un-map host buffer previously mapped by GXP_MAP_DMABUF.
 *
 * Only the @device_address field is used. Other fields are fetched from the
 * kernel's internal records. It is recommended to use the argument that was
 * passed in GXP_MAP_DMABUF to un-map the dma-buf.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_UNMAP_DMABUF _IOW(GXP_IOCTL_BASE, 21, struct gxp_map_dmabuf_ioctl)

struct gxp_mailbox_command_ioctl {
	/*
	 * Input:
	 * The virtual core to dispatch the command to.
	 */
	__u16 virtual_core_id;
	/*
	 * Output:
	 * The sequence number assigned to this command. The caller can use
	 * this value to match responses fetched via `GXP_MAILBOX_RESPONSE`
	 * with this command.
	 */
	__u64 sequence_number;
	/*
	 * Input:
	 * Device address to the buffer containing a GXP command. The user
	 * should have obtained this address from the GXP_MAP_BUFFER ioctl.
	 */
	__u64 device_address;
	/*
	 * Input:
	 * Size of the buffer at `device_address` in bytes.
	 */
	__u32 size;
	/*
	 * Input:
	 * Minimum power state to operate the entire DSP subsystem at until
	 * the mailbox command is finished(executed or timeout). One of the
	 * GXP_POWER_STATE_* defines from below.
	 *
	 * `GXP_POWER_STATE_OFF` is not a valid value when executing a
	 * mailbox command. The caller should pass GXP_POWER_STATE_UUD if the
	 * command is expected to run at the power state the wakelock has
	 * specified.
	 */
	__u32 gxp_power_state;
	/*
	 * Input:
	 * Memory interface power state to request from the system so long as
	 * the mailbox command is executing. One of the MEMORY_POWER_STATE*
	 * defines from below.
	 *
	 * If `MEMORY_POWER_STATE_UNDEFINED` is passed, no request to change
	 * the memory interface power state will be made.
	 */
	__u32 memory_power_state;
	/*
	 * Input:
	 * Flags describing the command, for use by the GXP device.
	 */
	__u32 flags;
	/*
	 * Input:
	 * Flags indicating power attribute requests from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [0:0]   - Deprecated, do not use
	 *   [1:1]   - LOW_FREQ_CLKMUX setting for power management
	 *		 0 = Don't switch CLKMUX clocks, default value
	 *		 1 = Switch CLKMUX clocks
	 *   [31:2]  - RESERVED
	 */
	__u32 power_flags;
};

/*
 * Push element to the mailbox commmand queue.
 *
 * The client must hold a VIRTUAL_DEVICE wakelock.
 */
#define GXP_MAILBOX_COMMAND \
	_IOWR(GXP_IOCTL_BASE, 23, struct gxp_mailbox_command_ioctl)

/*
 * Legacy "mailbox command" IOCTL that does not support power requests.
 * This IOCTL exists for backwards compatibility with older runtimes. All
 * fields, other than the unsupported `gxp_power_state`, `memory_power_state`,
 * and `power_flags`, are the same as in `struct gxp_mailbox_command_ioctl`.
 */
struct gxp_mailbox_command_compat_ioctl {
	__u16 virtual_core_id;
	__u64 sequence_number;
	__u64 device_address;
	__u32 size;
	__u32 flags;
};

/* The client must hold a VIRTUAL_DEVICE wakelock. */
#define GXP_MAILBOX_COMMAND_COMPAT \
	_IOW(GXP_IOCTL_BASE, 3, struct gxp_mailbox_command_compat_ioctl)

/* GXP mailbox response error code values */
#define GXP_RESPONSE_ERROR_NONE         (0)
#define GXP_RESPONSE_ERROR_INTERNAL     (1)
#define GXP_RESPONSE_ERROR_TIMEOUT      (2)

struct gxp_mailbox_response_ioctl {
	/*
	 * Input:
	 * The virtual core to fetch a response from.
	 */
	__u16 virtual_core_id;
	/*
	 * Output:
	 * Sequence number indicating which command this response is for.
	 */
	__u64 sequence_number;
	/*
	 * Output:
	 * Driver error code.
	 * Indicates if the response was obtained successfully,
	 * `GXP_RESPONSE_ERROR_NONE`, or what error prevented the command
	 * from completing successfully.
	 */
	__u16 error_code;
	/*
	 * Output:
	 * Value returned by firmware in response to a command.
	 * Only valid if `error_code` == GXP_RESPONSE_ERROR_NONE
	 */
	__u32 cmd_retval;
};

/*
 * Pop element from the mailbox response queue. Blocks until mailbox response
 * is available.
 *
 * The client must hold a VIRTUAL_DEVICE wakelock.
 */
#define GXP_MAILBOX_RESPONSE \
	_IOWR(GXP_IOCTL_BASE, 4, struct gxp_mailbox_response_ioctl)

struct gxp_register_mailbox_eventfd_ioctl {
	/*
	 * This eventfd will be signaled whenever a mailbox response arrives
	 * for the core specified by `virtual_core_id`.
	 *
	 * When registering, if an eventfd has already been registered for the
	 * specified core, the old eventfd will be unregistered and replaced.
	 *
	 * Not used during the unregister call, which clears any existing
	 * eventfd.
	 */
	__u32 eventfd;
	/*
	 * Reserved.
	 * Pass 0 for backwards compatibility.
	 */
	__u32 flags;
	/*
	 * The virtual core to register or unregister an eventfd from.
	 * While an eventfd is registered, it will be signaled exactly once
	 * any time a command to this virtual core receives a response or times
	 * out.
	 */
	__u16 virtual_core_id;
};

/*
 * Register an eventfd to be signaled whenever the specified virtual core
 * sends a mailbox response.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_REGISTER_MAILBOX_EVENTFD                                           \
	_IOW(GXP_IOCTL_BASE, 22, struct gxp_register_mailbox_eventfd_ioctl)

/*
 * Clear a previously registered mailbox response eventfd.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_UNREGISTER_MAILBOX_EVENTFD                                         \
	_IOW(GXP_IOCTL_BASE, 24, struct gxp_register_mailbox_eventfd_ioctl)

#define ETM_TRACE_LSB_MASK 0x1
#define ETM_TRACE_SYNC_MSG_PERIOD_MIN 8
#define ETM_TRACE_SYNC_MSG_PERIOD_MAX 256
#define ETM_TRACE_PC_MATCH_MASK_LEN_MAX 31

/*
 * For all *_enable and pc_match_sense fields, only the least significant bit is
 * considered. All other bits are ignored.
 */
struct gxp_etm_trace_start_ioctl {
	__u16 virtual_core_id;
	__u8 trace_ram_enable; /* Enables local trace memory. */
	/* When set, trace output is sent out on the ATB interface. */
	__u8 atb_enable;
	/* Enables embedding timestamp information in trace messages. */
	__u8 timestamp_enable;
	/*
	 * Determines the rate at which synchronization messages are
	 * automatically emitted in the output trace.
	 * Valid values: 0, 8, 16, 32, 64, 128, 256
	 * Eg. A value of 16 means 1 synchronization message will be emitted
	 * every 16 messages.
	 * A value of 0 means no synchronization messages will be emitted.
	 */
	__u16 sync_msg_period;
	__u8 pc_match_enable; /* PC match causes Stop trigger. */
	/*
	 * 32-bit address to compare to processor PC when pc_match_enable = 1.
	 * A match for a given executed instruction triggers trace stop.
	 * Note: trigger_pc is ignored when pc_match_enable = 0.
	 */
	__u32 trigger_pc;
	/*
	 * Indicates how many of the lower bits of trigger_pc to ignore.
	 * Valid values: 0 to 31
	 * Note: pc_match_mask_length is ignored when pc_match_enable = 0.
	 */
	__u8 pc_match_mask_length;
	/* When 0, match when the processor's PC is in-range of trigger_pc and
	 * mask. When 1, match when the processor's PC is out-of-range of
	 * trigger_pc and mask.
	 * Note: pc_match_sense is ignored when pc_match_enable = 0.
	 */
	__u8 pc_match_sense;
};

/*
 * Configure ETM trace registers and start ETM tracing.
 *
 * The client must hold a VIRTUAL_DEVICE wakelock.
 */
#define GXP_ETM_TRACE_START_COMMAND \
	_IOW(GXP_IOCTL_BASE, 7, struct gxp_etm_trace_start_ioctl)

/*
 * Halts trace generation via a software trigger. The virtual core id is passed
 * in as an input.
 *
 * The client must hold a VIRTUAL_DEVICE wakelock.
 */
#define GXP_ETM_TRACE_SW_STOP_COMMAND \
	_IOW(GXP_IOCTL_BASE, 8, __u16)

/*
 * Users should call this IOCTL after tracing has been stopped for the last
 * trace session of the core. Otherwise, there is a risk of having up to 3 bytes
 * of trace data missing towards the end of the trace session.
 * This is a workaround for b/180728272 and b/181623511.
 * The virtual core id is passed in as an input.
 *
 * The client must hold a VIRTUAL_DEVICE wakelock.
 */
#define GXP_ETM_TRACE_CLEANUP_COMMAND \
	_IOW(GXP_IOCTL_BASE, 9, __u16)

#define GXP_TRACE_HEADER_SIZE 256
#define GXP_TRACE_RAM_SIZE 4096
struct gxp_etm_get_trace_info_ioctl {
	/*
	 * Input:
	 * The virtual core to fetch a response from.
	 */
	__u16 virtual_core_id;
	/*
	 * Input:
	 * The type of data to retrieve.
	 * 0: Trace Header only
	 * 1: Trace Header + Trace Data in Trace RAM
	 */
	__u8 type;
	/*
	 * Input:
	 * Trace header user space address to contain trace header information
	 * that is used for decoding the trace.
	 */
	__u64 trace_header_addr;
	/*
	 * Input:
	 * Trace data user space address to contain Trace RAM data.
	 * Note: trace_data field will be empty if type == 0
	 */
	__u64 trace_data_addr;
};

/*
 * Retrieves trace header and/or trace data for decoding purposes.
 *
 * The client must hold a VIRTUAL_DEVICE wakelock.
 */
#define GXP_ETM_GET_TRACE_INFO_COMMAND \
	_IOWR(GXP_IOCTL_BASE, 10, struct gxp_etm_get_trace_info_ioctl)

#define GXP_TELEMETRY_TYPE_LOGGING	(0)
#define GXP_TELEMETRY_TYPE_TRACING	(1)

/*
 * Enable either logging or software tracing for all cores.
 * Accepts either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 * to specify whether logging or software tracing is to be enabled.
 *
 * Buffers for logging or tracing must have already been mapped via an `mmap()`
 * call with the respective offset and initialized by the client, prior to
 * calling this ioctl.
 *
 * If firmware is already running on any cores, they will be signaled to begin
 * logging/tracing to their buffers. Any cores booting after this call will
 * begin logging/tracing as soon as their firmware is able to.
 */
#define GXP_ENABLE_TELEMETRY _IOWR(GXP_IOCTL_BASE, 11, __u8)

/*
 * Disable either logging or software tracing for all cores.
 * Accepts either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 * to specify whether logging or software tracing is to be disabled.
 *
 * This call will block until any running cores have been notified and ACKed
 * that they have disabled the specified telemetry type.
 */
#define GXP_DISABLE_TELEMETRY _IOWR(GXP_IOCTL_BASE, 12, __u8)

struct gxp_register_telemetry_eventfd_ioctl {
	/*
	 * File-descriptor obtained via eventfd().
	 *
	 * Not used during the unregister step; the driver will unregister
	 * whichever eventfd it has currently registered for @type, if any.
	 */
	__u32 eventfd;
	/*
	 * Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`.
	 * The driver will signal @eventfd whenever any core signals a
	 * telemetry state change while this type of telemetry is active.
	 */
	__u8 type;
};

#define GXP_REGISTER_TELEMETRY_EVENTFD                                         \
	_IOW(GXP_IOCTL_BASE, 15, struct gxp_register_telemetry_eventfd_ioctl)

#define GXP_UNREGISTER_TELEMETRY_EVENTFD                                       \
	_IOW(GXP_IOCTL_BASE, 16, struct gxp_register_telemetry_eventfd_ioctl)

/*
 * Reads the 2 global counter registers in AURORA_TOP and combines them to
 * return the full 64-bit value of the counter.
 *
 * The client must hold a BLOCK wakelock.
 */
#define GXP_READ_GLOBAL_COUNTER _IOR(GXP_IOCTL_BASE, 17, __u64)

struct gxp_tpu_mbx_queue_ioctl {
	__u32 tpu_fd; /* TPU virtual device group fd */
	/*
	 * Bitfield indicating which virtual cores to allocate and map the
	 * buffers for.
	 * To map for virtual core X, set bit X in this field, i.e. `1 << X`.
	 *
	 * This field is not used by the unmap IOCTL, which always unmaps the
	 * buffers for all cores it had been mapped for.
	 */
	__u32 virtual_core_list;
	/*
	 * The user address of an edgetpu_mailbox_attr struct, containing
	 * cmd/rsp queue size, mailbox priority and other relevant info.
	 * This structure is defined in edgetpu.h in the TPU driver.
	 */
	__u64 attr_ptr;
};

/*
 * Map TPU-DSP mailbox cmd/rsp queue buffers.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_MAP_TPU_MBX_QUEUE \
	_IOW(GXP_IOCTL_BASE, 13, struct gxp_tpu_mbx_queue_ioctl)

/*
 * Un-map TPU-DSP mailbox cmd/rsp queue buffers previously mapped by
 * GXP_MAP_TPU_MBX_QUEUE.
 *
 * Only the @tpu_fd field will be used. Other fields will be fetched
 * from the kernel's internal records. It is recommended to use the argument
 * that was passed in GXP_MAP_TPU_MBX_QUEUE to un-map the buffers.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_UNMAP_TPU_MBX_QUEUE \
	_IOW(GXP_IOCTL_BASE, 14, struct gxp_tpu_mbx_queue_ioctl)

/*
 * Triggers a debug dump to be generated for cores.
 *
 * The cores requested to generate a debug dump are indicated by the bitmap of
 * the argument. For example, an argument of 'b1001 represents a request to
 * generate debug dumps for core 0 and 3.
 *
 * Returns 0 if all the debug dumps for the requested cores are successfully
 * triggered. If a debug dump fails to be triggered for one or more requested
 * cores, -EINVAL will be returned.
 *
 * The client must hold a VIRTUAL_DEVICE wakelock.
 *
 * Note: Root access is required to use this IOCTL.
 */
#define GXP_TRIGGER_DEBUG_DUMP _IOW(GXP_IOCTL_BASE, 27, __u32)

#endif /* __GXP_H__ */
