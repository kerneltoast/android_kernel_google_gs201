/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP debug dump handler
 *
 * Copyright (C) 2020-2022 Google LLC
 */

#ifndef __GXP_DEBUG_DUMP_H__
#define __GXP_DEBUG_DUMP_H__

#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "gxp-config.h"
#include "gxp-dma.h"
#include "gxp-internal.h"

#define HAS_COREDUMP (IS_GXP_TEST || IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP))

#if HAS_COREDUMP
#include <linux/platform_data/sscoredump.h>
#endif

#if GXP_HAS_MCU
/* Additional +1 is for the MCU core. */
#define GXP_NUM_DEBUG_DUMP_CORES (GXP_NUM_CORES + 1)
#else
#define GXP_NUM_DEBUG_DUMP_CORES GXP_NUM_CORES
#endif

#define GXP_NUM_COMMON_SEGMENTS 2
/*
 * Maximum number of segments to be dumped from core side.
 * TODO(b/362165250): [SW]Increase GXP_NUM_SEGMENTS to add dump segments from core side.
 * GXP_MAX_NUM_CORE_SEGMENTS gives the maximum possible segments that can be dumped.
 */
#define GXP_MAX_NUM_CORE_SEGMENTS 8
/* Maximum number of segments that can be dumped for the individual cores via MCU firmware. */
#define GXP_MAX_NUM_SEGMENTS_VIA_MCU_FIRMWARE 8
/* Maximum number of dump metadata segments to be dumped for the MCU firmware. */
#define GXP_MAX_NUM_MCU_DUMP_METADATA_SEGMENTS 1
/* The count of core segments dumped by initial versions of firmware. */
#define GXP_CORE_SEGMENT_COMPAT_COUNT 7
/* FW RO, FW RW, VD Private, Core Config, VD Config sections */
#define GXP_NUM_CORE_DATA_SEGMENTS 5
#define GXP_NUM_BUFFER_MAPPINGS 32
#define GXP_SEG_HEADER_NAME_LENGTH 32
#define GXP_NUM_SEGMENTS_PER_CORE                                                           \
	(GXP_NUM_COMMON_SEGMENTS + GXP_MAX_NUM_CORE_SEGMENTS + GXP_NUM_CORE_DATA_SEGMENTS + \
	 GXP_NUM_BUFFER_MAPPINGS + GXP_MAX_NUM_SEGMENTS_VIA_MCU_FIRMWARE +                  \
	 GXP_MAX_NUM_MCU_DUMP_METADATA_SEGMENTS)

/*
 * The minimum wait time in millisecond to be enforced between two successive calls to the SSCD
 * module to prevent the overwrite of the previous generated core dump files. SSCD module generates
 * the files whose name are at second precision i.e.
 * crashinfo_<SUBSYSTEM_NAME>_<%Y-%m-%d_%H-%M-%S>.txt and
 * coredump_<SUBSYSTEM_NAME>_<%Y-%m-%d_%H-%M-%S>.bin.
 */
#define SSCD_REPORT_WAIT_TIME (1000ULL)

#define GXP_Q7_ICACHE_SIZE 131072 /* I-cache size in bytes */
#define GXP_Q7_ICACHE_LINESIZE 64 /* I-cache line size in bytes */
#define GXP_Q7_ICACHE_WAYS 4
#define GXP_Q7_ICACHE_SETS                                                     \
	((GXP_Q7_ICACHE_SIZE / GXP_Q7_ICACHE_WAYS) / GXP_Q7_ICACHE_LINESIZE)
#define GXP_Q7_ICACHE_WORDS_PER_LINE (GXP_Q7_ICACHE_LINESIZE / sizeof(u32))

#define GXP_Q7_DCACHE_SIZE 65536 /* D-cache size in bytes */
#define GXP_Q7_DCACHE_LINESIZE 64 /* D-cache line size in bytes */
#define GXP_Q7_DCACHE_WAYS 4
#define GXP_Q7_DCACHE_SETS                                                     \
	((GXP_Q7_DCACHE_SIZE / GXP_Q7_DCACHE_WAYS) / GXP_Q7_DCACHE_LINESIZE)
#define GXP_Q7_DCACHE_WORDS_PER_LINE (GXP_Q7_DCACHE_LINESIZE / sizeof(u32))
#define GXP_Q7_NUM_AREGS 64
#define GXP_Q7_DCACHE_TAG_RAMS 2

#define GXP_DEBUG_DUMP_INT 0x1
#define GXP_DEBUG_DUMP_INT_MASK BIT(GXP_DEBUG_DUMP_INT)
#define GXP_DEBUG_DUMP_RETRY_NUM 5

struct gxp_timer_registers {
	u32 comparator;
	u32 control;
	u32 value;
};

struct gxp_lpm_transition_registers {
	u32 next_state;
	u32 seq_addr;
	u32 timer_val;
	u32 timer_en;
	u32 trigger_num;
	u32 trigger_en;
};

struct gxp_lpm_state_table_registers {
	struct gxp_lpm_transition_registers trans[PSM_TRANS_COUNT];
	u32 enable_state;
};

struct gxp_lpm_psm_registers {
	struct gxp_lpm_state_table_registers state_table[PSM_STATE_TABLE_COUNT];
	u32 data[PSM_DATA_COUNT];
	u32 cfg;
	u32 status;
	u32 debug_cfg;
	u32 break_addr;
	u32 gpin_lo_rd;
	u32 gpin_hi_rd;
	u32 gpout_lo_rd;
	u32 gpout_hi_rd;
	u32 debug_status;
};

struct gxp_common_registers {
	u32 aurora_revision;
	u32 common_int_pol_0;
	u32 common_int_pol_1;
	u32 dedicated_int_pol;
	u32 raw_ext_int;
	u32 core_pd[GXP_NUM_CORES];
	u32 global_counter_low;
	u32 global_counter_high;
	u32 wdog_control;
	u32 wdog_value;
	struct gxp_timer_registers timer[GXP_REG_TIMER_COUNT];
	u32 doorbell[DOORBELL_COUNT];
	u32 sync_barrier[SYNC_BARRIER_COUNT];
};

struct gxp_lpm_registers {
	u32 lpm_version;
	u32 trigger_csr_start;
	u32 imem_start;
	u32 lpm_config;
	u32 psm_descriptor[PSM_DESCRIPTOR_COUNT];
	u32 events_en[EVENTS_EN_COUNT];
	u32 events_inv[EVENTS_INV_COUNT];
	u32 function_select;
	u32 trigger_status;
	u32 event_status;
	u32 ops[OPS_COUNT];
	struct gxp_lpm_psm_registers psm_regs[PSM_COUNT];
};

struct gxp_mailbox_queue_desc {
	u16 cmd_queue_head;
	u16 cmd_queue_tail;
	u16 resp_queue_head;
	u16 resp_queue_tail;
	u32 cmd_queue_size;
	u32 cmd_elem_size;
	u32 resp_queue_size;
	u32 resp_elem_size;
};

struct gxp_user_buffer {
	/* Device address of user buffer */
	u64 device_addr;
	/* Size of user buffer */
	u32 size;
	/* Reserved for future expansion. */
	u32 reserved;
};

struct gxp_core_header {
	/* Firmware image unique identifier (0-based). */
	u32 firmware_id;
	/* Dump data is available for core. */
	u32 dump_available;
	/* Code indicating reason for debug dump request. */
	u8 dump_req_reason;
	/* Reserved for future expansion. */
	u8 reserved1[3];
	/* Number of segments dumped by the firmware. */
	u16 num_dumped_segments_by_fw;
	/* Number of segments dumped by the GXP kernel driver. */
	u16 num_dumped_segments_by_kd;
	/* Changelist number of the firmware. */
	u32 changelist;
	/* Size of core dump. */
	u32 core_dump_size;
	/* List of user buffers to be dumped by the GXP kernel driver. */
	struct gxp_user_buffer user_bufs[GXP_NUM_BUFFER_MAPPINGS];
};

enum gxp_segment_type {
	/* Unknown segment. */
	UNKNOWN = 0,
	/*
	 * Segment contains the dump of the common Aurora TOP registers. This segment is dumped by
	 * the GXP kernel driver.
	 */
	COMMON_REGISTERS = 1,
	/*
	 * Segment contains the dump of the LPM registers. This segment is dumped by the GXP kernel
	 * driver.
	 */
	LPM_REGISTERS = 2,
	/*
	 * Segment contains the dump of the TCM data. This segment is dumped by the core firmware.
	 */
	TCM = 3,
	/*
	 * Segment contains the dump of the core registers. This segment is dumped by the core
	 * firmware.
	 */
	CORE_REGISTERS = 4,
	/*
	 * Segment contains the dump of the mailbox registers. This segment is dumped by the core
	 * firmware.
	 */
	MAILBOX_REGISTERS = 5,
	/*
	 * Segment contains the dump of the instruction cache. This segment is dumped by the core
	 * firmware.
	 */
	INSTRUCTION_CACHE = 6,
	/*
	 * Segment contains the dump of the data cache. This segment is dumped by the core firmware.
	 */
	DATA_CACHE = 7,
	/*
	 * Segment contains the dump of the TRAX buffer. This segment is dumped by the core
	 * firmware.
	 */
	STACK_TRACE = 8,
	/*
	 * Segment contains the dump of the xtensa processor registers. This segment is dumped by
	 * the core firmware.
	 */
	PROCSESSOR_REGISTERS = 10,
	/*
	 * Segment contains the dump of the interrupt controller registers. This segment is dumped
	 * by the core firmware.
	 */
	INTERRUPT_REGISTERS = 11,
	/*
	 * Segment contains the dump of the ETM buffer. This segment is dumped by the MCU
	 * firmware.
	 */
	ETF = 12,
	/*
	 * Segment contains the dump of the TRAX buffer. This segment is dumped by the MCU
	 * firmware.
	 */
	TRAX = 13,
} __packed;

struct gxp_seg_header {
	/* Name of data type */
	enum gxp_segment_type type;
	/* Reserved for future expansion. */
	uint8_t reserved[31];
	/* Size of segment data */
	u32 size;
	/* Validity of segment data */
	u32 valid;
};

struct gxp_core_dump_header {
	struct gxp_core_header core_header;
	struct gxp_seg_header seg_header[GXP_MAX_NUM_CORE_SEGMENTS];
};

struct gxp_common_dump_data {
	struct gxp_common_registers common_regs; /* Seg 0 */
	struct gxp_lpm_registers lpm_regs; /* Seg 1 */
};

struct gxp_common_dump {
	struct gxp_seg_header seg_header[GXP_NUM_COMMON_SEGMENTS];
	struct gxp_common_dump_data common_dump_data;
};

struct gxp_core_dump {
	struct gxp_core_dump_header core_dump_header[GXP_NUM_CORES];
	/*
	 * A collection of 'GXP_NUM_CORES' core dumps;
	 * Each is core_dump_header[i].core_dump_size bytes long.
	 */
	uint32_t dump_data[];
};

/*
 * Descriptor for dump related information of individual cores that is dumped by the MCU firmware.
 */
struct gxp_mcu_dump_descriptor {
	/* Core ID of the DSP core or MCU. */
	uint8_t core_id;
	/*
	 * This flag is used by the kernel driver to determine if the dump data for a particular
	 * core is available for further processing. Its value is set by the MCU firmware and
	 * reset to zero by the kernel driver.
	 */
	uint8_t dump_available;
	/* Indicates the number of segments dumped for the core. */
	uint8_t num_segment_dumped;
	/* Reserved for future expansion. */
	uint8_t reserved;
	/* Client ID of the aborted client. */
	uint32_t client_id;
	/* Offset to the dump region reserved for the core. */
	uint32_t offset;
	/* Size of the dump region reserved for the core. */
	uint32_t size;
	/* Reserved for future expansion. */
	uint32_t reserved1[2];
	/* Segment headers for the core. */
	struct gxp_seg_header segment_headers[GXP_MAX_NUM_SEGMENTS_VIA_MCU_FIRMWARE];
};

/* Metadata for the dump region shared between MCU and GXP kernel driver. */
struct gxp_mcu_dump_metadata {
	/* Dump implementation version in the kernel driver. */
	uint8_t kernel_version;
	/* Dump implementation version in the MCU firmware. */
	uint8_t firmware_version;
	/* Reserved for future expansion. */
	uint16_t reserved;
	/* Reserved for future expansion. */
	uint32_t reserved1[2];
	/* Dump descriptors for the individual cores. */
	struct gxp_mcu_dump_descriptor dump_descriptors[GXP_NUM_DEBUG_DUMP_CORES];
};

/*
 * Dump region shared by GXP kernel driver to the MCU firmware for dumping the MCU and DSP cores
 * debug data.
 */
struct gxp_mcu_dump_region {
	/* Metadata for the dump region. */
	struct gxp_mcu_dump_metadata dump_metadata;
	/* Starting offset for the dump data. */
	uint32_t dump_data[];
};

struct gxp_debug_dump_work {
	struct work_struct work;
	struct gxp_dev *gxp;
	uint core_id;
};

struct gxp_debug_dump_manager {
	struct gxp_dev *gxp;
	/* Buffer holding debug dump data from core firmware. */
	struct gxp_coherent_buf core_buf;
	/* Buffer holding debug dump data from MCU firmware. */
	struct gxp_mapped_resource mcu_buf;
	struct gxp_debug_dump_work debug_dump_works[GXP_NUM_CORES];
	/* Start of the core dump */
	struct gxp_core_dump *core_dump;
	/* Start of the MCU dump region. */
	struct gxp_mcu_dump_region *mcu_dump;
	struct gxp_common_dump *common_dump;
	void *sscd_dev;
	void *sscd_pdata;
	dma_addr_t debug_dump_dma_handle; /* dma handle for debug dump */
	/*
	 * Debug dump lock to ensure only one debug dump is being processed at a
	 * time.
	 */
	struct mutex debug_dump_lock;
#if HAS_COREDUMP
	struct sscd_segment segs[GXP_NUM_DEBUG_DUMP_CORES][GXP_NUM_SEGMENTS_PER_CORE];
#endif /* HAS_COREDUMP */
};

int gxp_debug_dump_init(struct gxp_dev *gxp, void *sscd_dev, void *sscd_pdata);
void gxp_debug_dump_exit(struct gxp_dev *gxp);
struct work_struct *gxp_debug_dump_get_notification_handler(struct gxp_dev *gxp,
							    uint core);
bool gxp_debug_dump_is_enabled(void);

/**
 * gxp_debug_dump_invalidate_core_segments() - Invalidate core dump segments to enable core
 *                                             firmware to populate them on next debug dump
 *                                             trigger.
 *
 * This function is not thread safe. Caller should take the necessary precautions.
 *
 * @gxp: The GXP device to obtain the handler for
 * @core_id: physical id of the DSP core whose dump segments need to be invalidated.
 */
void gxp_debug_dump_invalidate_core_segments(struct gxp_dev *gxp, uint32_t core_id);

/**
 * gxp_debug_dump_send_forced_debug_dump_request() - Sends the forced debug dump request to the
 *                                                   running cores of the given vd.
 * @gxp: The GXP device to obtain the handler for.
 * @vd: vd whose cores to send the forced debug dump request.
 *
 * The caller must hold @gxp->vd_semaphore.
 *
 * In case of single core VD no forced debug dump request will be sent since there will be no other
 * core. For multicore VD, forced debug dump request will be sent to other cores only during the
 * first worker thread run. For successive worker thread runs since the forced request was already
 * sent, it will not be sent again.
 */
void gxp_debug_dump_send_forced_debug_dump_request(struct gxp_dev *gxp,
						   struct gxp_virtual_device *vd);

/**
 * gxp_debug_dump_process_dump_mcu_mode() - Checks and process the debug dump
 *                                          for cores from core_list.
 * @gxp: The GXP device to obtain the handler for
 * @core_list: A bitfield enumerating the physical cores on which crash is
 *             reported from firmware.
 * @crashed_vd: vd that has crashed.
 *
 * The caller must hold @crashed_vd->debug_dump_lock.
 *
 * Return:
 * * 0       - Success.
 * * -EINVAL - If vd state is not GXP_VD_UNAVAILABLE.
 */
int gxp_debug_dump_process_dump_mcu_mode(struct gxp_dev *gxp, uint core_list,
					 struct gxp_virtual_device *crashed_vd);

/**
 * gxp_debug_dump_report_mcu_crash() - Reports the SSCD module about the MCU crash.
 * @gxp: The GXP device to obtain the handler for
 */
void gxp_debug_dump_report_mcu_crash(struct gxp_dev *gxp);

#endif /* __GXP_DEBUG_DUMP_H__ */
