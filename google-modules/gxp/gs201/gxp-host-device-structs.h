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
#define NUM_SYSTEM_SEMAPHORES 64

/* Bit masks for the status fields in the telemetry structures. */
/* The telemetry buffers have been setup by the host. */
#define GXP_TELEMETRY_HOST_STATUS_ENABLED (1 << 0)
/* The telemetry buffers are being used by the device. */
#define GXP_TELEMETRY_DEVICE_STATUS_ENABLED (1 << 0)
/* There was an attempt to use the buffers but their content was invalid. */
#define GXP_TELEMETRY_DEVICE_STATUS_SANITY_CHECK_FAILED (1 << 1)

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

/* A structure describing the state of the doorbells on the system. */
struct gxp_doorbells_descriptor {
	/* The app this descriptor belongs to. */
	uint32_t application_id;
	/* The physical ID of the sync barrier protecting this region. */
	uint32_t protection_barrier;
	/* The number of doorbells described in this region. */
	uint32_t num_items;
	/* The list of doorbells available for usage. */
	struct dooorbell_metadata_t {
		/*
		 * The number of users using this doorbell. 0 when it's
		 * available.
		 */
		uint32_t users_count;
		/* The 0-based index of the doorbell described by this entry. */
		uint32_t hw_doorbell_idx;
	} doorbells[];
};

/* A structure describing the state of the sync barriers on the system. */
struct gxp_sync_barriers_descriptor {
	/* The app this descriptor belongs to. */
	uint32_t application_id;
	/* The physical ID of the sync barrier protecting this region. */
	uint32_t protection_barrier;
	/* The number of sync barriers described in this region. */
	uint32_t num_items;
	/* The list of sync barriers available for usage. */
	struct sync_barrier_metadata_t {
		/*
		 * The number of users using this barrier. 0 when it's
		 * available.
		 */
		uint32_t users_count;
		/*
		 * The 0-based index of the sync barrier described by this
		 * entry.
		 */
		uint32_t hw_barrier_idx;
	} barriers[];
};

/* A structure describing the state of the watchdog on the system. */
struct gxp_watchdog_descriptor {
	/* The physical ID of the sync barrier protecting this region. */
	uint32_t protection_barrier;
	/*
	 * The number of timer ticks before the watchdog expires.
	 * This is in units of 244.14 ns.
	 */
	uint32_t target_value;
	/* A bit mask of the cores expected to tickle the watchdog. */
	uint32_t participating_cores;
	/* A bit mask of the cores that have tickled the watchdog. */
	uint32_t responded_cores;
	/* A flag indicating whether or not the watchdog has tripped. */
	uint32_t tripped;
};

/*
 * A structure describing the telemetry (logging and tracing) parameters and
 * buffers.
 */
struct gxp_telemetry_descriptor {
	/* A struct for describing the parameters for telemetry buffers  */
	struct telemetry_descriptor {
		/*
		 * The telemetry status from the host's point of view. See the
		 * top of the file for the appropriate flags.
		 */
		uint32_t host_status;
		/*
		 * The telemetry status from the device point of view. See the
		 * top of the file for the appropriate flags.
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
 * A structure describing the state and allocations of the SW-based semaphores
 * on the system.
 */
struct gxp_semaphores_descriptor {
	/* The app this descriptor belongs to. */
	uint32_t application_id;
	/* The physical ID of the sync barrier protecting this region. */
	uint32_t protection_barrier;
	/*
	 * An array where each element is dedicated to a core. The element is a
	 * bit map describing of all the semaphores in the list below that have
	 * been unlocked but haven't been processed yet by the receiptient core.
	 */
	uint64_t woken_pending_semaphores[MAX_NUM_CORES];
	/*
	 * A mapping of which doorbells to use as a wakeup signal source per
	 * core.
	 */
	uint32_t wakeup_doorbells[MAX_NUM_CORES];
	/* The number of items described in this region. */
	uint32_t num_items;
	/* The list of semaphores available for usage. */
	struct semaphore_metadata {
		/*
		 * The number of users using this semaphore. 0 when it's for
		 * creation.
		 * Note: this is not the count value of the semaphore, but just
		 * an indication if this slot is available.
		 */
		uint32_t users_count;
		/*
		 * This is the semaphore count. Cores will block when they call
		 * 'Wait()' while this count is 0.
		 */
		uint32_t count;
		/*
		 * A bit map of 'NUM_DSP_CORES' bits indicating which cores are
		 * currently waiting on this semaphore to become available.
		 */
		uint32_t waiters;
	} semaphores[NUM_SYSTEM_SEMAPHORES];
};

/* A basic unidirectional queue. */
struct gxp_queue_info {
	/* A header describing the queue and its state. */
	struct queue_header {
		/* A device-side pointer of the storage managed by this queue */
		uint32_t storage;
		/* The index to the head of the queue. */
		uint32_t head_idx;
		/* The index to the tail of the queue. */
		uint32_t tail_idx;
		/* The size of an element stored this queue. */
		uint32_t element_size;
		/* The number of elements that can be stored in this queue. */
		uint32_t elements_count;
	} header;
	/* The semaphore ID controlling exclusive access to this core. */
	uint32_t access_sem_id;
	/*
	 * The ID for the semaphore containing the number of unprocessed items
	 * pushed to this queue.
	 */
	uint32_t posted_slots_sem_id;
	/*
	 * The ID for the semaphore containing the number of free slots
	 * available to store data in this queue.
	 */
	uint32_t free_slots_sem_id;
};

/* A struct describing a single core's set of incoming queues. */
struct gxp_core_info {
	/*
	 * The metadata for the queue holding incoming commands from other
	 * cores.
	 */
	struct gxp_queue_info incoming_commands_queue;
	/*
	 * The metadata for the queue holding incoming responses from other
	 * cores.
	 */
	struct gxp_queue_info incoming_responses_queue;
};

/* A structure describing all the cores' per-core metadata. */
struct gxp_cores_descriptor {
	/* The number of cores described in this descriptor. */
	uint32_t num_items;
	/* The descriptors for each core. */
	struct gxp_core_info cores[];
};

/*
 * The top level descriptor describing memory regions used to access system-wide
 * structures and resources.
 */
struct gxp_system_descriptor {
	/* A device address for the application data descriptor. */
	uint32_t app_descriptor_dev_addr[MAX_NUM_CORES];
	/* A device address for the watchdog descriptor. */
	uint32_t watchdog_dev_addr;
	/* A device address for the telemetry descriptor */
	uint32_t telemetry_dev_addr;
	/* A device address for the common debug dump region */
	uint32_t debug_dump_dev_addr;
};

/* A structure describing the metadata belonging to a specific application. */
struct gxp_application_descriptor {
	/* The ID for this GXP application. */
	uint32_t application_id;
	/* The number of cores this application has. */
	uint16_t core_count;
	/*
	 * The cores mask; a bit at index `n` indicates that core `n` is part of
	 * this app.
	 */
	uint16_t cores_mask;
	/* The number of threads allocated for each core. */
	uint16_t threads_count;
	/* The size of system memory given to this app. */
	uint32_t system_memory_size;
	/* The device-address of the system memory given to this app. */
	uint32_t system_memory_addr;
	/* The size of TCM memory allocated per bank for this app. */
	uint32_t tcm_memory_per_bank;   /* in units of 4 kB */
	/* A device address for the doorbells descriptor. */
	uint32_t doorbells_dev_addr;
	/* A device address for the sync barriers descriptor. */
	uint32_t sync_barriers_dev_addr;
	/* A device address for the semaphores descriptor. */
	uint32_t semaphores_dev_addr;
	/* A device address for the cores cmd/rsp queues descriptor. */
	uint32_t cores_info_dev_addr;
};

/* The structure describing a core-to-core command. */
struct gxp_core_to_core_command {
	/* The source of port number (the core's virtual ID) of the command. */
	uint32_t source;
	/* The command's sequence number. */
	uint64_t sequence_number;
	/* The command payload device address. */
	uint64_t device_address;
	/* The size of the payload in bytes. */
	uint32_t size;
	/* The generic command flags. */
	uint32_t flags;
};

/* The structure describing a core-to-core response. */
struct gxp_core_to_core_response {
	/* The source of port number (the core's virtual ID) of the response. */
	uint32_t source;
	/* The response's sequence number. */
	uint64_t sequence_number;
	/* The response error code (if any). */
	uint16_t error_code;
	/* The response return value (filled-in by the user). */
	int32_t cmd_retval;
};

#endif /* __GXP_HOST_DEVICE_STRUCTURES_H__ */
