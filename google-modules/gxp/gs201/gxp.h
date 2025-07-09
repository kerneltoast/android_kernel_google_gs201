/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP kernel-userspace interface definitions.
 *
 * Copyright (C) 2020-2022 Google LLC
 */

#ifndef __GXP_H__
#define __GXP_H__

#include <linux/ioctl.h>
#include <linux/types.h>

/* Interface Version */
#define GXP_INTERFACE_VERSION_MAJOR 1
#define GXP_INTERFACE_VERSION_MINOR 31
#define GXP_INTERFACE_VERSION_BUILD 0

/* mmap offsets for MCU logging and tracing buffers */
#define GXP_MMAP_MCU_LOG_BUFFER_OFFSET 0x30000
#define GXP_MMAP_MCU_TRACE_BUFFER_OFFSET 0x40000

/* mmap offsets for core telemetry buffers */
#define GXP_MMAP_CORE_LOG_BUFFER_OFFSET 0x50000

/* mmap offset for secure core logging and tracing */
#define GXP_MMAP_SECURE_CORE_LOG_BUFFER_OFFSET 0x70000

#define GXP_IOCTL_BASE 0xEE

/* GXP map flag macros */
/* The mask for specifying DMA direction in GXP map flag */
#define GXP_MAP_DIR_MASK 3
/* The targeted DMA direction for the buffer */
#define GXP_MAP_DMA_BIDIRECTIONAL 0
#define GXP_MAP_DMA_TO_DEVICE 1
#define GXP_MAP_DMA_FROM_DEVICE 2
/* Create coherent mappings of the buffer. */
#define GXP_MAP_COHERENT (1 << 2)

/* To check whether the driver is working in MCU mode. */
#define GXP_SPEC_FEATURE_MODE_MCU (1 << 0)

/* To specify the secureness of the virtual device. */
#define GXP_ALLOCATE_VD_SECURE BIT(0)

/* Core telemetry buffer size is a multiple of 64 kB */
#define GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE 0x10000u
/* Magic code used to indicate the validity of telemetry buffer contents */
#define GXP_TELEMETRY_BUFFER_VALID_MAGIC_CODE 0xC0DEC0DEu
/* Magic code used to indicate the validity of secure telemetry buffer contents */
#define GXP_TELEMETRY_SECURE_BUFFER_VALID_MAGIC_CODE 0xA0B0C0D0u

struct gxp_map_ioctl {
	/*
	 * Deprecated. All virtual cores will be mapped.
	 *
	 * Bitfield indicating which virtual cores to map the buffer for.
	 * To map for virtual core X, set bit X in this field, i.e. `1 << X`.
	 *
	 * This field is not used by the unmap IOCTL, which always unmaps a
	 * buffer for all cores it had been mapped for.
	 */
	__u16 virtual_core_list;
	__u64 host_address; /* virtual address in the process space */
	__u32 size; /* size of mapping in bytes */
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
	 *   [2:2]   - Coherent Mapping:
	 *              0 = Create non-coherent mappings of the buffer.
	 *              1 = Create coherent mappings of the buffer.
	 *              Note: this attribute may be ignored on platforms where
	 *              gxp is not I/O coherent.
	 *   [31:3]  - RESERVED
	 */
	__u32 flags;
	/*
	 * - GXP_MAP_BUFFER (Input / Output):
	 * If the value is 0, the buffer will be mapped to any free location of
	 * the unreserved region and its device address will be returned to this
	 * field.
	 *
	 * If the value is non-zero page-aligned device address, the buffer will
	 * be mapped to the address. The user must reserve an IOVA region which
	 * can cover the size of buffer from @device_address first.
	 * (See GXP_RESERVE_IOVA_REGION)
	 *
	 * If @device_address is not page-aligned, the mapping will fail.
	 *
	 * Note that if @host_address is not page-aligned, the returned
	 * @device_address can be different from the input since the driver maps
	 * the buffer in the page unit. Therefore, the runtime should check the
	 * returned value even though they passed one target address to here.
	 * The buffer is not guaranteed to be mapped to the exactly the same
	 * address.
	 *
	 * E.g., if @device_address is 0x2000, @host_address is 0x1800, @size
	 * is 0x900 and the page size is 0x1000, the driver will allocate two
	 * pages from the reserved region, [0x2000, 0x3000) and [0x3000, 0x4000),
	 * and will map the buffer to [0x2800, 0x3100) since the page masked
	 * offset of @host_address is 0x800 (0x1800 & ~PAGE_MASK = 0x800, 0x2000
	 * + 0x800 = 0x2800). As a result, the size of occupied area in the
	 * region will be two pages which is larger than @size and the returned
	 * @device_address will be 0x2800 which is different from the input.
	 *
	 * - GXP_UNMAP_BUFFER (Input):
	 * The device address of the buffer to be unmapped.
	 */
	__u64 device_address;
};

/*
 * Map host buffer.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_MAP_BUFFER _IOWR(GXP_IOCTL_BASE, 0, struct gxp_map_ioctl)

/*
 * Un-map host buffer previously mapped by GXP_MAP_BUFFER.
 *
 * Only the @device_address field will be used. Other fields will be fetched
 * from the kernel's internal records. It is recommended to use the argument
 * that was passed in GXP_MAP_BUFFER to un-map the buffer.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_UNMAP_BUFFER _IOW(GXP_IOCTL_BASE, 1, struct gxp_map_ioctl)

/* GXP sync flag macros */
#define GXP_SYNC_FOR_DEVICE (0)
#define GXP_SYNC_FOR_CPU (1)

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
#define GXP_SYNC_BUFFER _IOW(GXP_IOCTL_BASE, 2, struct gxp_sync_ioctl)

/* GXP mailbox response error code values */
#define GXP_RESPONSE_ERROR_NONE (0)
#define GXP_RESPONSE_ERROR_INTERNAL (1)
#define GXP_RESPONSE_ERROR_TIMEOUT (2)
#define GXP_RESPONSE_ERROR_NOENT (3)
#define GXP_RESPONSE_ERROR_AGAIN (4)
#define GXP_RESPONSE_ERROR_CANCELED (5)

struct gxp_mailbox_response_ioctl {
	/*
	 * Input:
	 * The virtual core to fetch a response from.
	 * Only used in direct mode.
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
 * Pop an element from the mailbox response queue. Blocks until mailbox response
 * is available.
 *
 * The client must hold a VIRTUAL_DEVICE wakelock.
 */
#define GXP_MAILBOX_RESPONSE                                                   \
	_IOWR(GXP_IOCTL_BASE, 4, struct gxp_mailbox_response_ioctl)

struct gxp_specs_ioctl {
	/* Maximum number of cores that can be allocated to a virtual device */
	__u8 core_count;
	/*
	 * A field to indicate the features or modes the device supports.
	 * Bitfields:
	 *   [0:0]   - Mode:
	 *               0 = direct mode
	 *               1 = MCU mode
	 *   [7:1]   - RESERVED
	 */
	__u8 features;
	/*
	 * Size of per core allocated telemetry buffer represented in units
	 * of GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE.
	 */
	__u8 telemetry_buffer_size;
	/*
	 * Size of per core reserved secure telemetry buffer represented in
	 * units of GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE.
	 */
	__u8 secure_telemetry_buffer_size;
	/*
	 * The number of virtual devices can be allocated at the same time.
	 */
	__u8 max_vd_allocation;
	/*
	 * The number of virtual devices can acquire wakelock at the same time.
	 */
	__u8 max_vd_activation;
	/*
	 * Size of the device address space can be mapped per virtual device, in
	 * unit MB.
	 */
	__u16 total_iova_size;
	/* Deprecated fields that should be ignored */
	__u8 reserved[8];
};

/* Query system specs. */
#define GXP_GET_SPECS _IOR(GXP_IOCTL_BASE, 5, struct gxp_specs_ioctl)

struct gxp_virtual_device_ioctl {
	/*
	 * Input:
	 * The number of cores requested for the virtual device.
	 */
	__u8 core_count;
	/*
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [0:0]   - GXP_ALLOCATE_VD_SECURE setting for vd secureness
	 *		 0 = Non-secure, default value
	 *		 1 = Secure
	 *   [31:1]  - RESERVED
	 */
	__u8 flags;
	/* Deprecated field that should be ignored. */
	__u8 reserved[6];
	/*
	 * Output:
	 * The ID assigned to the virtual device and shared with its cores.
	 */
	__u32 vdid;
};

/* Allocate virtual device. */
#define GXP_ALLOCATE_VIRTUAL_DEVICE                                            \
	_IOWR(GXP_IOCTL_BASE, 6, struct gxp_virtual_device_ioctl)

#define GXP_TELEMETRY_TYPE_LOGGING (0)
#define GXP_TELEMETRY_TYPE_TRACING (1)

struct gxp_tpu_mbx_queue_ioctl {
	__u32 tpu_fd; /* TPU virtual device group fd */
	/*
	 * Deprecated. All virtual cores will be mapped.
	 *
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
#define GXP_MAP_TPU_MBX_QUEUE                                                  \
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
#define GXP_UNMAP_TPU_MBX_QUEUE                                                \
	_IOW(GXP_IOCTL_BASE, 14, struct gxp_tpu_mbx_queue_ioctl)

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

#define GXP_REGISTER_CORE_TELEMETRY_EVENTFD                                    \
	_IOW(GXP_IOCTL_BASE, 15, struct gxp_register_telemetry_eventfd_ioctl)

#define GXP_UNREGISTER_CORE_TELEMETRY_EVENTFD                                  \
	_IOW(GXP_IOCTL_BASE, 16, struct gxp_register_telemetry_eventfd_ioctl)

/* For backward compatibility. */
#define GXP_REGISTER_TELEMETRY_EVENTFD GXP_REGISTER_CORE_TELEMETRY_EVENTFD
#define GXP_UNREGISTER_TELEMETRY_EVENTFD GXP_UNREGISTER_CORE_TELEMETRY_EVENTFD

/*
 * Reads the 2 global counter registers in AURORA_TOP and combines them to
 * return the full 64-bit value of the counter.
 *
 * The client must hold a BLOCK wakelock.
 */
#define GXP_READ_GLOBAL_COUNTER _IOR(GXP_IOCTL_BASE, 17, __u64)

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

struct gxp_map_dmabuf_ioctl {
	/*
	 * Deprecated. All virtual cores will be mapped.
	 *
	 * Bitfield indicating which virtual cores to map the dma-buf for.
	 * To map for virtual core X, set bit X in this field, i.e. `1 << X`.
	 *
	 * This field is not used by the unmap dma-buf IOCTL, which always
	 * unmaps a dma-buf for all cores it had been mapped for.
	 */
	__u16 virtual_core_list;
	__s32 dmabuf_fd; /* File descriptor of the dma-buf to map. */
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
	 * - GXP_MAP_DMABUF (Input / Output):
	 * If the value is 0, the dma-buf will be mapped to any free location of
	 * the unreserved region and its device address will be returned to this
	 * field.
	 *
	 * If the value is non-zero page-aligned device address, the dma-buf
	 * will be mapped to the address. The user must reserve an IOVA region
	 * which can cover the size of dma-buf from @device_address first.
	 * (See GXP_RESERVE_IOVA_REGION)
	 *
	 * If @device_address is not page-aligned, the mapping will fail.
	 *
	 * - GXP_UNMAP_DMABUF (Input):
	 * The device address of the dma-buf to be unmapped.
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

struct gxp_mailbox_command_ioctl {
	/*
	 * Input:
	 * The virtual core to dispatch the command to.
	 * Only used in direct mode.
	 */
	__u16 virtual_core_id;
	/*
	 * Input:
	 * The number of cores to dispatch the command to.
	 * Only used in non-direct mode.
	 */
	__u16 num_cores;
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
 * Push an element to the mailbox command queue.
 *
 * The client must hold a VIRTUAL_DEVICE wakelock.
 */
#define GXP_MAILBOX_COMMAND                                                    \
	_IOWR(GXP_IOCTL_BASE, 23, struct gxp_mailbox_command_ioctl)

/*
 * Clear a previously registered mailbox response eventfd.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_UNREGISTER_MAILBOX_EVENTFD                                         \
	_IOW(GXP_IOCTL_BASE, 24, struct gxp_register_mailbox_eventfd_ioctl)

/*
 * Components for which a client may hold a wakelock.
 * Acquired by passing these values as `components_to_wake` in
 * `struct gxp_acquire_wakelock_ioctl` to GXP_ACQUIRE_WAKELOCK and released by
 * passing these values directly as the argument to GXP_RELEASE_WAKELOCK.
 *
 * Multiple wakelocks can be acquired or released at once by passing multiple
 * components, ORed together.
 */
#define WAKELOCK_BLOCK (1 << 0)
#define WAKELOCK_VIRTUAL_DEVICE (1 << 1)

/*
 * DSP subsystem Power state values for use as `gxp_power_state` in
 * `struct gxp_acquire_wakelock_ioctl`.
 * Note: GXP_POWER_STATE_READY is a deprecated state. The way to achieve
 * original state is to request GXP_POWER_STATE_UUD with setting
 * GXP_POWER_LOW_FREQ_CLKMUX flag. Requesting GXP_POWER_STATE_READY is treated
 * as identical to GXP_POWER_STATE_UUD.
 */
#define GXP_POWER_STATE_OFF 0
#define GXP_POWER_STATE_UUD 1
#define GXP_POWER_STATE_SUD 2
#define GXP_POWER_STATE_UD 3
#define GXP_POWER_STATE_NOM 4
#define GXP_POWER_STATE_READY 5
#define GXP_POWER_STATE_UUD_PLUS 6
#define GXP_POWER_STATE_SUD_PLUS 7
#define GXP_POWER_STATE_UD_PLUS 8
#define GXP_POWER_STATE_PERCENT_FREQUENCY_5 GXP_POWER_STATE_UUD
#define GXP_POWER_STATE_PERCENT_FREQUENCY_10 9
#define GXP_POWER_STATE_PERCENT_FREQUENCY_15 10
#define GXP_POWER_STATE_PERCENT_FREQUENCY_20 11
#define GXP_POWER_STATE_PERCENT_FREQUENCY_25 12
#define GXP_POWER_STATE_PERCENT_FREQUENCY_30 13
#define GXP_POWER_STATE_PERCENT_FREQUENCY_35 GXP_POWER_STATE_UUD_PLUS
#define GXP_POWER_STATE_PERCENT_FREQUENCY_40 14
#define GXP_POWER_STATE_PERCENT_FREQUENCY_45 15
#define GXP_POWER_STATE_PERCENT_FREQUENCY_50 GXP_POWER_STATE_SUD
#define GXP_POWER_STATE_PERCENT_FREQUENCY_55 16
#define GXP_POWER_STATE_PERCENT_FREQUENCY_60 17
#define GXP_POWER_STATE_PERCENT_FREQUENCY_65 GXP_POWER_STATE_SUD_PLUS
#define GXP_POWER_STATE_PERCENT_FREQUENCY_70 18
#define GXP_POWER_STATE_PERCENT_FREQUENCY_75 GXP_POWER_STATE_UD
#define GXP_POWER_STATE_PERCENT_FREQUENCY_80 19
#define GXP_POWER_STATE_PERCENT_FREQUENCY_85 GXP_POWER_STATE_UD_PLUS
#define GXP_POWER_STATE_PERCENT_FREQUENCY_90 20
#define GXP_POWER_STATE_PERCENT_FREQUENCY_95 21
#define GXP_POWER_STATE_MAX_FREQUENCY GXP_POWER_STATE_NOM
#define GXP_POWER_STATE_OVERDRIVE 22
#define GXP_NUM_POWER_STATES (GXP_POWER_STATE_OVERDRIVE + 1)

/*
 * Memory interface power state values for use as `memory_power_state` in
 * `struct gxp_acquire_wakelock_ioctl`.
 */
#define MEMORY_POWER_STATE_UNDEFINED 0
#define MEMORY_POWER_STATE_MIN 1
#define MEMORY_POWER_STATE_VERY_LOW 2
#define MEMORY_POWER_STATE_LOW 3
#define MEMORY_POWER_STATE_HIGH 4
#define MEMORY_POWER_STATE_VERY_HIGH 5
#define MEMORY_POWER_STATE_MAX 6

/*
 * GXP power flag macros, supported by `flags` in `gxp_acquire_wakelock_ioctl`
 * and `power_flags in `gxp_mailbox_command_ioctl`.
 *
 * Non-aggressor flag is deprecated. Setting this flag is a no-op since
 * non-aggressor support is defeatured.
 */
#define GXP_POWER_NON_AGGRESSOR (1 << 0)
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
#define GXP_POWER_LOW_FREQ_CLKMUX (1 << 1)

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
	 * Reserved for future use.
	 */
	__u32 reserved;
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
 * Upon a successful return, the specified components will be powered on.
 * If the specified components contain VIRTUAL_DEVICE, and they were not
 * already running at the specified or higher power states, requests will
 * have been sent to transition both the DSP subsystem and memory interface
 * to the specified states.
 *
 * If the same client invokes this IOCTL for the same component more than once
 * without a corresponding call to `GXP_RELEASE_WAKE_LOCK` in between, the
 * second call may update requested power states, but have no other effects.
 * No additional call to `GXP_RELEASE_WAKE_LOCK` will be required.
 */
#define GXP_ACQUIRE_WAKE_LOCK                                                  \
	_IOW(GXP_IOCTL_BASE, 25, struct gxp_acquire_wakelock_ioctl)

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

#define GXP_REGISTER_MCU_TELEMETRY_EVENTFD                                     \
	_IOW(GXP_IOCTL_BASE, 28, struct gxp_register_telemetry_eventfd_ioctl)

#define GXP_UNREGISTER_MCU_TELEMETRY_EVENTFD                                   \
	_IOW(GXP_IOCTL_BASE, 29, struct gxp_register_telemetry_eventfd_ioctl)

#define GXP_UCI_CMD_OPAQUE_SIZE 48

struct gxp_mailbox_uci_command_compat_ioctl {
	/*
	 * Output:
	 * The sequence number assigned to this command. The caller can use
	 * this value to match responses fetched via `GXP_MAILBOX_UCI_RESPONSE`
	 * with this command.
	 */
	__u64 sequence_number;
	/* reserved fields */
	__u8 reserved[8];
	/*
	 * Input:
	 * Will be copied to the UCI command without modification.
	 */
	__u8 opaque[GXP_UCI_CMD_OPAQUE_SIZE];
};

/*
 * Push an element to the UCI command queue.
 *
 * The client must hold a BLOCK wakelock.
 *
 * Note that this ioctl is deprecated and the runtime should use
 * GXP_MAILBOX_UCI_COMMAND instead.
 */
#define GXP_MAILBOX_UCI_COMMAND_COMPAT                                         \
	_IOWR(GXP_IOCTL_BASE, 30, struct gxp_mailbox_uci_command_compat_ioctl)

struct gxp_mailbox_uci_response_ioctl {
	/*
	 * Output:
	 * Sequence number indicating which command this response is for.
	 */
	__u64 sequence_number;
	/*
	 * Output:
	 * Error code propagated from the MCU firmware side.
	 */
	__u16 error_code;
	/* reserved fields */
	__u8 reserved[6];
	/*
	 * Output:
	 * Is copied from the UCI response without modification.
	 * Only valid if this IOCTL returns 0.
	 */
	__u8 opaque[16];
};

/*
 * Pop an element from the UCI response queue. Blocks until mailbox response
 * is available.
 *
 * The client must hold a BLOCK wakelock.
 *
 * Returns:
 *  0          - A response arrived from the MCU firmware. Note that this doesn't guarantee the
 *               success of the UCI command. The runtime must refer to @error_code field to check
 *               whether there was an error from the MCU side while processing the request.
 *
 *  -ETIMEDOUT - MCU firmware is not responding.
 */
#define GXP_MAILBOX_UCI_RESPONSE                                               \
	_IOR(GXP_IOCTL_BASE, 31, struct gxp_mailbox_uci_response_ioctl)

/*
 * struct gxp_create_sync_fence_data
 * @seqno:		the seqno to initialize the fence with
 * @timeline_name:	the name of the timeline the fence belongs to
 * @fence:		returns the fd of the new sync_file with the new fence
 *
 * Timeline names can be up to 128 characters (including trailing NUL byte)
 * for gxp debugfs and kernel debug logs.  These names are truncated to 32
 * characters in the data returned by the standard SYNC_IOC_FILE_INFO
 * ioctl.
 */
#define GXP_SYNC_TIMELINE_NAME_LEN 128
struct gxp_create_sync_fence_data {
	__u32 seqno;
	char timeline_name[GXP_SYNC_TIMELINE_NAME_LEN];
	__s32 fence;
};

/*
 * Create a DMA sync fence, return the sync_file fd for the new fence.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_CREATE_SYNC_FENCE                                                  \
	_IOWR(GXP_IOCTL_BASE, 32, struct gxp_create_sync_fence_data)

/*
 * struct gxp_signal_sync_fence_data
 * @fence:		fd of the sync_file for the fence
 * @error:		error status errno value or zero for success
 */
struct gxp_signal_sync_fence_data {
	__s32 fence;
	__s32 error;
};

/*
 * Signal a DMA sync fence with optional error status.
 * Can pass a sync_file fd created by any driver.
 * Signals the first DMA sync fence in the sync file.
 */
#define GXP_SIGNAL_SYNC_FENCE                                                  \
	_IOW(GXP_IOCTL_BASE, 33, struct gxp_signal_sync_fence_data)

/*
 * struct gxp_sync_fence_status
 * @fence:		fd of the sync_file for the fence
 * @status:		returns:
 *			   0 if active
 *			   1 if signaled with no error
 *			   negative errno value if signaled with error
 */
struct gxp_sync_fence_status {
	__s32 fence;
	__s32 status;
};

/*
 * Retrieve DMA sync fence status.
 * Can pass a sync_file fd created by any driver.
 * Returns the status of the first DMA sync fence in the sync file.
 */
#define GXP_SYNC_FENCE_STATUS                                                  \
	_IOWR(GXP_IOCTL_BASE, 34, struct gxp_sync_fence_status)

/*
 * struct gxp_register_invalidated_eventfd_ioctl
 * @eventfd:            File-descriptor obtained via eventfd().
 *                      Not used during the unregister step.
 */
struct gxp_register_invalidated_eventfd_ioctl {
	__u32 eventfd;
};

/*
 * Registers an eventfd which will be triggered when the device crashes and
 * the virtual device of the client is invalidated.
 */
#define GXP_REGISTER_INVALIDATED_EVENTFD                                       \
	_IOW(GXP_IOCTL_BASE, 35, struct gxp_register_invalidated_eventfd_ioctl)

#define GXP_UNREGISTER_INVALIDATED_EVENTFD                                     \
	_IOW(GXP_IOCTL_BASE, 36, struct gxp_register_invalidated_eventfd_ioctl)

/* The size of device properties pre-agreed with firmware */
#define GXP_DEV_PROP_SIZE 256
/*
 * struct gxp_set_device_properties_ioctl
 * @opaque:		Device properties defined by runtime and firmware.
 */
struct gxp_set_device_properties_ioctl {
	__u8 opaque[GXP_DEV_PROP_SIZE];
};

/*
 * Registers device properties which will be passed down to firmware on every
 * MCU boot.
 */
#define GXP_SET_DEVICE_PROPERTIES                                              \
	_IOW(GXP_IOCTL_BASE, 37, struct gxp_set_device_properties_ioctl)

/*
 * The reason why the device is invalidated.
 * - GXP_INVALIDATED_NONE: The device is not invalidated.
 * - GXP_INVALIDATED_MCU_CRASH: The device is invalidated because the MCU is broken.
 * - GXP_INVALIDATED_CLIENT_CRASH: The device is invalidated because the client is broken.
 * - GXP_INVALIDATED_VMBOX_RELEASE_FAILED: The vmbox is not released successfully.
 */
#define GXP_INVALIDATED_NONE 0
#define GXP_INVALIDATED_MCU_CRASH 1
#define GXP_INVALIDATED_CLIENT_CRASH 2
#define GXP_INVALIDATED_VMBOX_RELEASE_FAILED 3

/* Provides the reason why the device is invalidated.  */
#define GXP_GET_INVALIDATED_REASON _IOR(GXP_IOCTL_BASE, 38, __u32)

#define GXP_MAX_FENCES_PER_UCI_COMMAND 4

/*
 * Indicates the end of the fence FD array. This macro will be used by the
 * ioctls which receive multiple fence FDs as an array.
 */
#define GXP_FENCE_ARRAY_TERMINATION (~0u)

struct gxp_mailbox_uci_command_ioctl {
	/*
	 * Output:
	 * The sequence number assigned to this command. The caller can use
	 * this value to match responses fetched via `GXP_MAILBOX_UCI_RESPONSE`
	 * with this command.
	 */
	__u64 sequence_number;
	/*
	 * Input:
	 * The FDs of in-fences that this command will waits for. The kernel
	 * driver will read FDs from this array until it meets
	 * `GXP_FENCE_ARRAY_TERMINATION` or end-of-array. (i.e., reads at most
	 * GXP_MAX_FENCES_PER_UCI_COMMAND fences) The fences can be either IIF
	 * or in-kernel fence.
	 *
	 * Note that the type of fences must be the same.
	 */
	__u32 in_fences[GXP_MAX_FENCES_PER_UCI_COMMAND];
	/*
	 * Input:
	 * The concept is the same with `in_fences`, but these are out-fences
	 * that this command will signal once its job is finished.
	 *
	 * Note that the type of fences can be mixed.
	 */
	__u32 out_fences[GXP_MAX_FENCES_PER_UCI_COMMAND];
	/*
	 * Input:
	 * The user-defined timeout in milliseconds.
	 */
	__u32 timeout_ms;
	/*
	 * Input:
	 * Flags indicating attribute of the command.
	 *
	 * Bitfields:
	 *    [0:0]    - Nullity of the command. The purpose of this is to
	 *               support a command which requires more than 4 fan-in or
	 *               fan-out fences. By having a NULL command which does
	 *               NO-OP, but waits on / signals fences, we can achieve
	 *               that as a workaround.
	 *                 0 = normal command
	 *                 1 = NULL command
	 *    [31:1]   - RESERVED
	 *
	 */
	__u32 flags;
	/*
	 * Input:
	 * RuntimeCommand which will be copied to the UCI command without
	 * modification by the kernel driver.
	 */
	__u8 opaque[GXP_UCI_CMD_OPAQUE_SIZE];
	/* Reserved fields. */
	__u8 reserved[32];
};

/*
 * Push an element to the UCI command queue.
 *
 * The client must hold a BLOCK wakelock.
 */
#define GXP_MAILBOX_UCI_COMMAND                                                \
	_IOWR(GXP_IOCTL_BASE, 39, struct gxp_mailbox_uci_command_ioctl)

/* The type of IP for IIF. Must be synced with IIF driver. */
enum gxp_iif_ip_type {
	GXP_IIF_IP_DSP,
	GXP_IIF_IP_TPU,
	GXP_IIF_IP_GPU,
};

struct gxp_create_iif_fence_ioctl {
	/*
	 * Input:
	 * The type of the fence signaler IP. (See enum gxp_iif_ip_type)
	 */
	__u8 signaler_ip;
	/*
	 * Input:
	 * The number of the signalers.
	 */
	__u16 total_signalers;
	/*
	 * Output:
	 * The file descriptor of the created fence.
	 */
	__s32 fence;
};

/* Create an IIF fence. */
#define GXP_CREATE_IIF_FENCE                                                   \
	_IOWR(GXP_IOCTL_BASE, 40, struct gxp_create_iif_fence_ioctl)

/*
 * The ioctl won't register @eventfd and will simply return the number of remaining signalers of
 * each fence. Must be synced with IIF driver.
 *
 * The value must be synced with `GCIP_FENCE_REMAINING_SIGNALERS_NO_REGISTER_EVENTFD`.
 */
#define GXP_FENCE_REMAINING_SIGNALERS_NO_REGISTER_EVENTFD (~0u)

struct gxp_fence_remaining_signalers_ioctl {
	/*
	 * Input:
	 * Array of fence file descriptors to check whether there are remaining
	 * signalers to be submitted or not. The fences must be IIF. The
	 * kernel driver will read FDs from this array until it meets
	 * `GXP_FENCE_ARRAY_TERMINATION` or end-of-array. (i.e., reads at most
	 * GXP_MAX_FENCES_PER_UCI_COMMAND fences)
	 */
	__u32 fences[GXP_MAX_FENCES_PER_UCI_COMMAND];
	/*
	 * Input:
	 * The eventfd which will be triggered if there were fence(s) which
	 * haven't finished the signaler submission yet when the ioctl is called
	 * and when they eventually have finished the submission. Note that if
	 * all fences already finished the submission (i.e., all values in the
	 * returned @remaining_signalers are 0), this eventfd will be ignored.
	 *
	 * Note that if `GXP_FENCE_REMAINING_SIGNALERS_NO_REGISTER_EVENTFD` is
	 * passed, this ioctl will simply return the number of remaining
	 * signalers of each fence to @remaining_signalers.
	 */
	__u32 eventfd;
	/*
	 * Output:
	 * The number of remaining signalers to be submitted per fence. The
	 * order should be same with @fences.
	 */
	__u32 remaining_signalers[GXP_MAX_FENCES_PER_UCI_COMMAND];
};

/*
 * Check whether there are remaining signalers to be submitted to fences.
 * If all signalers have been submitted, the runtime is expected to send UCI
 * commands right away. Otherwise, it will listen the eventfd to wait signaler
 * submission to be finished.
 */
#define GXP_FENCE_REMAINING_SIGNALERS                                          \
	_IOWR(GXP_IOCTL_BASE, 41, struct gxp_fence_remaining_signalers_ioctl)

struct gxp_reserve_iova_region_ioctl {
	/*
	 * Input (GXP_RESERVE_IOVA_REGION):
	 * The size of region to reserve. It should be page-aligned.
	 */
	__u64 size;
	/*
	 * Output (GXP_RESERVE_IOVA_REGION):
	 * The start IOVA address of the reserved region.
	 *
	 * Input (GXP_RETIRE_IOVA_REGION):
	 * The start IOVA address of the region to be retired.
	 */
	__u64 device_address;
};

/*
 * Reserves an IOVA region from the virtual device's IOMMU domain.
 *
 * The runtime can use `GXP_MAP_{BUFFER,DMABUF}` ioctls with specifying
 * the address inside of the reserved region to map to @device_address
 * field of those ioctl.
 *
 * The reserved region can be returned using `GXP_RETIRE_IOVA_REGION` ioctl.
 * Otherwise, the regions will be returned when the virtual device is going to
 * be destroyed.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_RESERVE_IOVA_REGION                                                \
	_IOWR(GXP_IOCTL_BASE, 42, struct gxp_reserve_iova_region_ioctl)

/*
 * Retires the reserved IOVA region.
 *
 * If there are buffers or dma-bufs which are not yet unmapped from the region,
 * this ioctl will try to unmap all of them. If all mappings have been unmapped
 * normally, it will return the reserved region eventually.
 *
 * However, if there are mapping(s) which are still accessed by other threads
 * by the race condition and are not unmapped even after this ioctl, the region
 * will be returned later once all mappings are not in use.
 *
 * The runtime must not map any buffers/dma-bufs to the retired region and not
 * access the mappings of the region after this ioctl is called.
 *
 * Only the @device_address field will be used.
 *
 * The client must have allocated a virtual device.
 */
#define GXP_RETIRE_IOVA_REGION                                                 \
	_IOW(GXP_IOCTL_BASE, 43, struct gxp_reserve_iova_region_ioctl)

#endif /* __GXP_H__ */
