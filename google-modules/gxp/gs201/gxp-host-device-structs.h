/* SPDX-License-Identifier: GPL-2.0 */
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
 */

#ifndef __GXP_HOST_DEVICE_STRUCTURES_H__
#define __GXP_HOST_DEVICE_STRUCTURES_H__

#define MAX_NUM_CORES 4

/* The number of physical doorbells and sync barriers allocated to each VD */
#define GXP_NUM_DOORBELLS_PER_VD 7
#define GXP_NUM_SYNC_BARRIERS_PER_VD 4

/* The first allowed doorbell and sync barrier to be used for VDs' usage */
#define GXP_DOORBELLS_START 4 /* The first 4 are used for boot */
#define GXP_SYNC_BARRIERS_START 1 /* The first 1 is used for UART */

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
#define GXP_BOOT_STATUS_SUSPENDING 7
#define GXP_BOOT_STATUS_SUSPENDING_FAILED 8
#define GXP_BOOT_STATUS_SUSPENDING_FAILED_ACTIVE_WL 9
#define GXP_BOOT_STATUS_WAITING_FOR_WORKLOAD 10
#define GXP_BOOT_STATUS_WAITING_FOR_DMA 11
#define GXP_BOOT_STATUS_SHUTTING_DOWN 12

/* Bit masks for the status fields in the core telemetry structures. */
/* The core telemetry buffers have been setup by the host. */
#define GXP_CORE_TELEMETRY_HOST_STATUS_ENABLED (1 << 0)
/* The core telemetry buffers are being used by the device. */
#define GXP_CORE_TELEMETRY_DEVICE_STATUS_ENABLED (1 << 0)
/* There was an attempt to use the buffers but their content was invalid. */
#define GXP_CORE_TELEMETRY_DEVICE_STATUS_SANITY_CHECK_FAILED (1 << 1)

/* Definitions for host->device boot mode requests */
/*
 * Request that the core performs a normal cold boot on the next power-on event.
 * This does not actually wake the core up, but's required before powering the
 * core up if cold boot is desired.
 * Core power-on could be performed using any wake-up source like the doorbells.
 */
#define GXP_BOOT_MODE_REQUEST_COLD_BOOT                 0

/*
 * Request that the core suspends on the next suspend signal arrival. This does
 * not trigger a suspend operation. A subsequent mailbox command or notification
 * is needed to trigger the actual transition.
 */
#define GXP_BOOT_MODE_REQUEST_SUSPEND                   1

/*
 * Request the core resumes on the next power on-event. This does not trigger a
 * resume operation, but's required before powering the core up if warm
 * boot/resume is desired.
 * Core power-on could be performed using any wake-up source like direct LPM
 * transition into PS0.
 */
#define GXP_BOOT_MODE_REQUEST_RESUME                    2

/* Cold boot status definitions */
#define GXP_BOOT_MODE_STATUS_COLD_BOOT_PENDING          0
#define GXP_BOOT_MODE_STATUS_COLD_BOOT_COMPLETED        3

/* Core suspend status definitions */
#define GXP_BOOT_MODE_STATUS_SUSPEND_PENDING            1
#define GXP_BOOT_MODE_STATUS_SUSPEND_STARTED            4
#define GXP_BOOT_MODE_STATUS_SUSPEND_COMPLETED          5
#define GXP_BOOT_MODE_STATUS_SUSPEND_ABORTED            6

/* Core resume/warm boot status definitions */
#define GXP_BOOT_MODE_STATUS_RESUME_PENDING             2
#define GXP_BOOT_MODE_STATUS_RESUME_STARTED             7
#define GXP_BOOT_MODE_STATUS_RESUME_COMPLETED           8
#define GXP_BOOT_MODE_STATUS_RESUME_FAILED              9

/* Invalid boot mode request code */
#define GXP_BOOT_MODE_STATUS_INVALID_MODE               10

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

	/* Reserved fields for future expansion */
	uint32_t reserved_boot[12];

	/* To be used to communicate statistics for timing events during boot */
	uint32_t timing_entries[16];

	/* To be used to communicate crash events in case of failures */
	uint32_t valid_crash_info;
	uint32_t crash_exccause;
	uint32_t crash_excvaddr;
	uint32_t crash_epc1;
	uint32_t reserved_crash_info[12];

	/* Reserved for more categories */
	uint32_t reserved[16];

	/*
	 * The per-core job descriptor. This struct will be inspected by the FW
	 * at the beginning of every dispatch.
	 */
	struct gxp_job_descriptor job_descriptor;
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
