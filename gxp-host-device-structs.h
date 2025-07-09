/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP host-device interface structures.
 *
 * Copyright (C) 2021 Google LLC
 *
 * This header is shared with the GXP firmware. It establishes the format of the
 * shared structures used by the GXP driver to describe to the GXP FW the HW
 * setup and memory regions needed by the FW to operate.
 * Since the header is shared with the FW, it cannot rely on kernel-specific
 * headers or data structures.
 *
 * Note: since the structures are shared across entities (cores, kernel, MCU),
 * and they may change as the code is running (since they are used to build the
 * system synchronization primitives), care should be taken to prevent the
 * compiler from optimizing reads to such structures (for example, when polling
 * on a value of a member in a struct; waiting for another core to change it).
 * To achieve that, it's generally advised to access these structures as
 * volatile to forbid the compiler from caching field values in CPU registers.
 */

#ifndef __GXP_HOST_DEVICE_STRUCTURES_H__
#define __GXP_HOST_DEVICE_STRUCTURES_H__

/*
 * The structure currently doesn't change its layout between the different
 * HW generations; thus max core count is kept the same for all.
 */
#define MAX_NUM_CORES 4

/*
 * The number of physical doorbells, sync barriers and timers allocated to each
 * VD. The HW supports a total of 16 sync barriers; divided across 4 active VDs.
 * Some are reserved for system use such as the UART sync barrier. It also
 * supports a total of 32 doorbells; divided across 4 active VDs. Some are
 * reserved for system use; such as the 4 doorbells for waking up cores.
 * 8 timers are also supported by the HW. Some are reserved for system use; such
 * as timers 0-3 and timer 7.
 */
#define GXP_NUM_DOORBELLS_PER_VD 7
#define GXP_NUM_SYNC_BARRIERS_PER_VD 4
#define GXP_NUM_TIMERS_PER_VD 1

/* The first allowed doorbell and sync barrier to be used for VDs' usage */
#define GXP_DOORBELLS_START 4 /* The first 4 are used for boot */
#define GXP_SYNC_BARRIERS_START 1 /* The first 1 is used for UART */
#define GXP_TIMERS_START 4 /* The first 4 are used for global and cores */

/* Definitions for host->device boot mode requests */
/*
 * No boot action is needed. This is a valid mode once a core is running.
 * However, it's an invalid state when a FW is powering on. The DSP core will
 * write it to the boot mode register once it starts a transition.
 * This is helpful in case the core reboots/crashes while performing the
 * transition so it doesn't get stuck in a boot loop.
 */
#define GXP_BOOT_MODE_NONE 0

/*
 * Request that the core performs a normal cold boot on the next power-on event.
 * This does not actually wake the core up, but is required before powering the
 * core up if cold boot is desired.
 * Core power-on could be performed using any wake-up source like the doorbells.
 * Upon success, the boot status should be GXP_BOOT_STATUS_ACTIVE.
 */
#define GXP_BOOT_MODE_COLD_BOOT 1

/*
 * Request that the core suspends on the next suspend signal arrival. This does
 * not trigger a suspend operation. A subsequent mailbox command or notification
 * is needed to trigger the actual transition. Upon success, the boot status
 * should be GXP_BOOT_STATUS_SUSPENDED.
 */
#define GXP_BOOT_MODE_SUSPEND 2

/*
 * Request that the core to preempt the active workload on the next suspend
 * signal arrival.Upon success, the boot status should be
 * GXP_BOOT_STATUS_SUSPENDED.
 */
#define GXP_BOOT_MODE_PREEMPT 3

/*
 * Request the core resumes on the next power on-event. This does not trigger a
 * resume operation, but is required before powering the core up if warm
 * boot/resume is desired.
 * Core power-on could be performed using any wake-up source like direct LPM
 * transition into PS0. Upon success, the boot status should be
 * GXP_BOOT_STATUS_ACTIVE
 */
#define GXP_BOOT_MODE_RESUME 4

/*
 * Request the core shutdown. A subsequent mailbox command or notification
 * is needed to trigger the actual transition. Upon success, the boot status
 * should be GXP_BOOT_STATUS_OFF.
 */
#define GXP_BOOT_MODE_SHUTDOWN 5

/* Definitions for host->device boot status */
/* Initial status */
#define GXP_BOOT_STATUS_NONE 0

/* Final status */
#define GXP_BOOT_STATUS_ACTIVE 1
#define GXP_BOOT_STATUS_SUSPENDED 2
#define GXP_BOOT_STATUS_OFF 3

/* Transition status */
#define GXP_BOOT_STATUS_INVALID_MODE 4
#define GXP_BOOT_STATUS_BOOTING 5
#define GXP_BOOT_STATUS_BOOTING_FAILED 6
#define GXP_BOOT_STATUS_SUSPEND_WAITING_FOR_WL 20
#define GXP_BOOT_STATUS_SUSPEND_WAITING_FOR_DMA 21
#define GXP_BOOT_STATUS_SUSPEND_SAVING_TCM 22
#define GXP_BOOT_STATUS_SUSPEND_SAVING_TCM_FAILED 23
#define GXP_BOOT_STATUS_SUSPEND_SAVING_TOP 24
#define GXP_BOOT_STATUS_SUSPEND_SAVING_CORE 25
#define GXP_BOOT_STATUS_SUSPEND_FAILED 26
#define GXP_BOOT_STATUS_RESUME_RESTORING_CORE 40
#define GXP_BOOT_STATUS_RESUME_RESTORING_CORE_FAILED 41
#define GXP_BOOT_STATUS_RESUME_RESTORING_MISC 42
#define GXP_BOOT_STATUS_RESUME_RESTORING_TCM 43
#define GXP_BOOT_STATUS_RESUME_RESTORING_TOP 44
#define GXP_BOOT_STATUS_RESUME_FAILED 44
#define GXP_BOOT_STATUS_SHUTTING_DOWN 60

/* Definitions for host->device warm up cache action requests */

/* No boot action is needed. */
#define GXP_WARM_UP_CACHE_NONE 0

/* Call SystemImpl::HandleUserDispatch with empty command after resuming.  */
#define GXP_WARM_UP_CACHE_CALL_HANDLE_USER_DISPATCH 1

/* Timing info enums */
/* Suspend */
#define GXP_TIMING_MCU_INTENT_SUSPEND 0
#define GXP_TIMING_DSP_ACK_SUSPEND 1
#define GXP_TIMING_START_SUSPEND 2
#define GXP_TIMING_CORE_QUIESCENCED 3
#define GXP_TIMING_TCM_SAVED 4
#define GXP_TIMING_TOP_SAVED 5
#define GXP_TIMING_INTERNAL_SAVED 6
#define GXP_TIMING_SUSPEND_COMPLETED 7

/* Resume */
#define GXP_TIMING_MCU_INTENT_RESUME 8
#define GXP_TIMING_DSP_FIRST_INSTR 9
#define GXP_TIMING_CACHE_RESET 10
#define GXP_TIMING_INTERNAL_RESTORE_START 11
#define GXP_TIMING_INTERNAL_RESTORED 12
#define GXP_TIMING_TCM_RESTORED 13
#define GXP_TIMING_TOP_RESTORED 14
#define GXP_TIMING_RESUME_COMPLETED 15

/* Bit masks for the status fields in the core telemetry structures. */
/* The core telemetry buffers have been setup by the host. */
#define GXP_CORE_TELEMETRY_HOST_STATUS_ENABLED (1 << 0)
/* The core telemetry buffers are being used by the device. */
#define GXP_CORE_TELEMETRY_DEVICE_STATUS_ENABLED (1 << 0)
/* There was an attempt to use the buffers but their content was invalid. */
#define GXP_CORE_TELEMETRY_DEVICE_STATUS_SANITY_CHECK_FAILED (1 << 1)

/* Mailbox command buffer descriptor invalid address for null command */
#define GXP_DEVICE_ADDRESS_INVALID 0

/*
 * A structure describing the core telemetry (logging and tracing) parameters
 * and buffers.
 */
struct gxp_core_telemetry_descriptor {
	/* A struct for describing the parameters for core telemetry buffers. */
	struct core_telemetry_descriptor {
		/*
		 * The core telemetry status from the host's point of view. See
		 * the top of the file for the appropriate flags.
		 */
		uint32_t host_status;
		/*
		 * The core telemetry status from the device point of view. See
		 * the top of the file for the appropriate flags.
		 */
		uint32_t device_status;
		/*
		 * The device address for the buffer used for storing events.
		 * The head and tail indices are described inside the data
		 * pointed to by `buffer_addr`.
		 */
		uint32_t buffer_addr;
		/* The size of the buffer (in bytes) */
		uint32_t buffer_size;
		/* The watermark interrupt threshold (in bytes) */
		uint32_t watermark_level;
	} per_core_loggers[MAX_NUM_CORES], per_core_tracers[MAX_NUM_CORES];
};

/*
 * A structure for describing the state of the job this worker core is part of.
 * This struct is expected to change per dispatch/context switch/preepmtion as
 * it describes the HW resources, FW IDs, and other parameters that may change
 * across job dispatches.
 * It also establishes a slot used for the various HW resources this VD is
 * expected to use.
 * Each FW in a VD is expected to be provided its own copy of this structure
 * based on the job that it's part of.
 */
struct gxp_job_descriptor {
	/* The number of workers participating in this job. */
	uint32_t workers_count;

	/*
	 * A mapping between a worker ID and the FW ID handling it. The FW ID
	 * used for handling worker 'w' is defined in worker_to_fw[w].
	 */
	int32_t worker_to_fw[MAX_NUM_CORES];

	/*
	 * A slot ID between 0 and MAX_NUM_CORES (exclusive) that indicates
	 * which block of HW resources this VD is expected to use. All system
	 * HW resources (such as doorbells, sync barriers, etc) are split across
	 * the slots evenly; usually starting at a specific physical ID and
	 * spanning a number consecutive instances. The start ID for each HW
	 * resource category is defined in GXP_<resource_name>_START; and the
	 * number of resources alloted to each slot is defined in
	 * GXP_NUM_<resource_name>_PER_VD.
	 */
	uint32_t hardware_resources_slot;
};

/*
 * A per-FW control structure used to communicate between the host (MCU or
 * kernel) and the DSP core. The region is expected to be hosted in uncached
 * memory.
 */
struct gxp_host_control_region {
	/*
	 * Written to by the FW to indicate to the host that the core is
	 * alive.
	 */
	uint32_t core_alive_magic;

	/*
	 * Written to by the FW to indicate to the host that the core can read
	 * TOP registers.
	 */
	uint32_t top_access_ok;

	/*
	 * Written to by the host to specify the request FW boot mode. See the
	 * GXP_BOOT_MODE_* definitions for valid values. Always set by the FW to
	 * GXP_BOOT_MODE_NONE once the requested boot mode transition is
	 * completed.
	 */
	uint32_t boot_mode;

	/*
	 * Written to by the FW to indicate the boot status. See
	 * GXP_BOOT_STATUS_* definitions for valid values.
	 */
	uint32_t boot_status;

	/* GXP kernel driver major version. Host is responsible for updating it. */
	uint16_t gxp_kernel_driver_major_version;

	/* GXP kernel driver minor version. Host is responsible for updating it. */
	uint16_t gxp_kernel_driver_minor_version;

	/* Reserved fields for future expansion */
	uint32_t reserved_boot[11];

	/* To be used to communicate statistics for timing events during boot */
	uint32_t timing_entries[16];

	/* To be used to communicate crash events in case of failures */
	uint32_t crash_handler_stage;
	uint32_t crash_exccause;
	uint32_t crash_excvaddr;
	uint32_t crash_epc1;
	/* Written by host to request debug dump generation on core. */
	uint32_t generate_debug_dump;
	/* Written by core to notify the host about debug dump completion. */
	uint32_t debug_dump_generated;
	/* To be used by telemetry to check if IRQ needs to be sent to MCU/kernel. */
	uint32_t telemetry_threshold_reached;

	/* Debug keys to communicate runtime context */
	/*
	 * Android Process ID in runtime that triggered the workload on this DSP core.
	 * It gets written by the runtime while preparing the workloads.
	 */
	uint16_t android_pid;
	/*
	 * Unique id of a GxpDevice within a process. It gets written by the runtime
	 * while preparing the workloads.
	 */
	uint16_t device_id;
	/*
	 * Unique id associated with each run command of a device. It gets written by
	 * the DSP firmware.
	 */
	uint32_t job_id;

	/* Currently loaded active library details on DSP firmware */
	/*
	 * Base address of the currently loaded active library's code section on DSP
	 * firmware.
	 */
	uint32_t code_section_base_addr;
	/*
	 * Base address of the currently loaded active library's data0 section on DSP
	 * firmware.
	 */
	uint32_t data0_section_base_addr;
	/*
	 * Base address of the currently loaded active library's data1 section on DSP
	 * firmware.
	 */
	uint32_t data1_section_base_addr;

	uint32_t reserved_crash_info[4];

	/*
	 * Written to by the host to specify the action after resuming the core. See
	 * the GXP_WARM_UP_* definitions for valid values.
	 */
	uint8_t warm_up_cache;

	/* Reserved for more categories */
	uint8_t reserved[63];

	/*
	 * The per-core job descriptor. This struct will be inspected by the FW
	 * at the beginning of every dispatch.
	 */
	struct gxp_job_descriptor job_descriptor;
};

/*
 * A structure describing the external state of the VD. This structure is read
 * once by the FW upon the first cold boot and is never checked again.
 */
struct gxp_vd_descriptor {
	/* The ID for this GXP application. */
	uint32_t application_id;

	/*
	 * Whether or not this VD has been initialized by one of its cores.
	 * This variable is protected by sync barrier at offset 0. Should be
	 * initialized by the host to 0.
	 */
	uint32_t vd_is_initialized;
};

/*
 * A structure describing the telemetry (logging and tracing) parameters and
 * buffers; this describes R/O aspects of the telemetry buffers.
 */
struct gxp_telemetry_descriptor_ro {
	struct telemetry_descriptor_ro {
		/*
		 * The telemetry status from the host's point of view. See the
		 * top of the file for the appropriate flags.
		 */
		uint32_t host_status;

		/*
		 * The device address for the buffer used for storing events.
		 * The head and tail indices are described inside the data
		 * pointed to by `buffer_addr`.
		 */
		uint32_t buffer_addr;

		/* The size of the buffer (in bytes) */
		uint32_t buffer_size;
	} per_core_loggers[MAX_NUM_CORES], per_core_tracers[MAX_NUM_CORES];
};

/*
 * A descriptor for data that is common to the entire system; usually accessed
 * by physical core. This region is mapped as R/O for all VDs. Should be
 * writable by the host (MCU/Kernel)
 */
struct gxp_system_descriptor_ro {
	/* A device address for the common debug dump region */
	uint32_t debug_dump_dev_addr;

	/*
	 * A R/O descriptor for the telemetry data. Describing buffer
	 * parameters.
	 */
	struct gxp_telemetry_descriptor_ro telemetry_desc;
};

/*
 * A structure describing the telemetry (logging and tracing) parameters; this
 * describes R/W aspects of the telemetry system.
 */
struct gxp_telemetry_descriptor_rw {
	/* A struct for describing R/W status parameters of the buffer  */
	struct telemetry_descriptor_rw {
		/*
		 * The telemetry status from the device point of view. See the
		 * top of the file for the appropriate flags.
		 */
		uint32_t device_status;

		/*
		 * Whether or not this telemetry category has data available
		 * for the host
		 */
		uint32_t data_available;
	} per_core_loggers[MAX_NUM_CORES], per_core_tracers[MAX_NUM_CORES];
};

/*
 * A descriptor for data that is common to the entire system; usually accessed
 * by physical core. This region is mapped as R/W for all VDs.
 */
struct gxp_system_descriptor_rw {
	/* A R/W descriptor for the telemetry data */
	struct gxp_telemetry_descriptor_rw telemetry_desc;
};

#endif /* __GXP_HOST_DEVICE_STRUCTURES_H__ */
