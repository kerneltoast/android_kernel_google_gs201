/* BEGIN-INTERNAL */
/*
 * Google LWIS IOCTL Commands and Data Structures
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/* END-INTERNAL */
#ifndef LWIS_COMMANDS_H_
#define LWIS_COMMANDS_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#endif /* __KERNEL__ */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#pragma pack(push)
#pragma pack(4)

/*
 *  IOCTL Types and Data Structures.
 */

/*
 * Device tree strings have a maximum length of 31, according to specs.
 * Adding 1 byte for the null character.
 */
#define LWIS_MAX_NAME_STRING_LEN 32
/* Maximum clock number defined in device tree. */
#define LWIS_MAX_CLOCK_NUM 20
/* Maximum number of register blocks per device */
#define LWIS_MAX_REG_NUM 20

/*
 * lwis_device_types
 * top  : top level device that overlooks all the LWIS devices. Will be used to
 *        list the information of the other LWIS devices in the system.
 * i2c  : for controlling i2c devices
 * ioreg: for controlling mapped register I/O devices
 * slc  : for configuring system level cache partitions
 * dpm  : for dynamic power manager requests update.
 * test : for test-specific devices.
 */
enum lwis_device_types {
	DEVICE_TYPE_UNKNOWN = -1,
	DEVICE_TYPE_TOP,
	DEVICE_TYPE_I2C,
	DEVICE_TYPE_IOREG,
	DEVICE_TYPE_SLC,
	DEVICE_TYPE_DPM,
	DEVICE_TYPE_TEST,
	NUM_DEVICE_TYPES
};

/* Qos clock family. */
enum lwis_clock_family {
	CLOCK_FAMILY_INVALID = -1,
	CLOCK_FAMILY_CAM,
	CLOCK_FAMILY_INTCAM,
	CLOCK_FAMILY_TNR,
	CLOCK_FAMILY_MIF,
	CLOCK_FAMILY_INT,
	NUM_CLOCK_FAMILY
};

struct lwis_clk_setting {
	// clock name defined in device tree.
	char name[LWIS_MAX_NAME_STRING_LEN];
	// clock index stored in lwis_dev->clocks
	int32_t clk_index;
	// clock rate
	uint32_t frequency;
};

struct lwis_reg_block {
	// reg block name defined in device tree.
	char name[LWIS_MAX_NAME_STRING_LEN];
	// reg index stored in reg_list.block
	int32_t reg_index;
	// reg start address defined in device tree.
	uint32_t start;
	// reg block size defined in device tree.
	uint32_t size;
};

struct lwis_device_info {
	int32_t id;
	int32_t type;
	char name[LWIS_MAX_NAME_STRING_LEN];
	struct lwis_clk_setting clks[LWIS_MAX_CLOCK_NUM];
	int32_t num_clks;
	struct lwis_reg_block regs[LWIS_MAX_REG_NUM];
	int32_t num_regs;
	int32_t transaction_worker_thread_pid;
	int32_t periodic_io_thread_pid;
};

enum lwis_dma_alloc_flags {
	// Allocates a cached buffer.
	LWIS_DMA_BUFFER_CACHED = 1UL << 0,
	// Allocates a buffer which is not initialized to 0 to avoid
	// initialization overhead.
	LWIS_DMA_BUFFER_UNINITIALIZED = 1UL << 1,
	// Allocates a buffer which is stored in contiguous memory.
	LWIS_DMA_BUFFER_CONTIGUOUS = 1UL << 2,
	// Allocates a buffer represent system cache reservation.
	LWIS_DMA_SYSTEM_CACHE_RESERVATION = 1UL << 3,
	// Allocates a secure buffer.
	LWIS_DMA_BUFFER_SECURE = 1UL << 4,
};

struct lwis_alloc_buffer_info {
	// IOCTL input for BUFFER_ALLOC
	size_t size;
	uint32_t flags; // lwis_dma_alloc_flags
	// IOCTL output for BUFFER_ALLOC
	int32_t dma_fd;
	int32_t partition_id;
};

struct lwis_buffer_info {
	// IOCTL input for BUFFER_ENROLL
	int32_t fd;
	bool dma_read;
	bool dma_write;
	// IOCTL output for BUFFER_ENROLL
	uint64_t dma_vaddr;
};

struct lwis_enrolled_buffer_info {
	int32_t fd;
	uint64_t dma_vaddr;
};

struct lwis_buffer_cpu_access_op {
	int32_t fd;
	bool start;
	bool read;
	bool write;
	uint32_t offset;
	size_t len;
};

enum lwis_io_entry_types {
	LWIS_IO_ENTRY_READ,
	LWIS_IO_ENTRY_READ_BATCH,
	LWIS_IO_ENTRY_WRITE,
	LWIS_IO_ENTRY_WRITE_BATCH,
	LWIS_IO_ENTRY_MODIFY,
	LWIS_IO_ENTRY_POLL,
	LWIS_IO_ENTRY_READ_ASSERT
};

// For io_entry read and write types.
struct lwis_io_entry_rw {
	int32_t bid;
	uint64_t offset;
	uint64_t val;
};

struct lwis_io_entry_rw_batch {
	int32_t bid;
	uint64_t offset;
	size_t size_in_bytes;
	uint8_t *buf;
	bool is_offset_fixed;
};

// For io_entry modify types.
struct lwis_io_entry_modify {
	int32_t bid;
	uint64_t offset;
	uint64_t val;
	uint64_t val_mask;
};

// For io_entry read assert type.
struct lwis_io_entry_read_assert {
	int32_t bid;
	uint64_t offset;
	uint64_t val;
	uint64_t mask;
	uint64_t timeout_ms;
};

struct lwis_io_entry {
	int32_t type;
	union {
		struct lwis_io_entry_rw rw;
		struct lwis_io_entry_rw_batch rw_batch;
		struct lwis_io_entry_modify mod;
		struct lwis_io_entry_read_assert read_assert;
	};
};

struct lwis_io_entries {
	uint32_t num_io_entries;
	struct lwis_io_entry *io_entries;
};

struct lwis_echo {
	size_t size;
	const char *msg;
	bool kernel_log;
};

/* The first 4096 event IDs are reserved for generic events shared by all
 * devices.
 *
 * The rest are specific to device specializations
 */
// Event NONE and INVALID are intended to be sharing the same ID.
#define LWIS_EVENT_ID_NONE 0
#define LWIS_EVENT_ID_INVALID 0
#define LWIS_EVENT_ID_HEARTBEAT 1
#define LWIS_EVENT_ID_CLIENT_CLEANUP 2
// ...
// Error event defines
#define LWIS_EVENT_ID_START_OF_ERROR_RANGE 2048
#define LWIS_ERROR_EVENT_ID_MEMORY_PAGE_FAULT 2048
#define LWIS_ERROR_EVENT_ID_SYSTEM_SUSPEND 2049
#define LWIS_ERROR_EVENT_ID_EVENT_QUEUE_OVERFLOW 2050
// ...
#define LWIS_EVENT_ID_START_OF_SPECIALIZED_RANGE 4096

/*
 * LWIS event id structure:
 *   bit[0..31] Event code - actual software/hardware event ID
 *   bit[32..47] Device ID - the device that the event belongs to
 *   bit[48..63] flags
 */
#define LWIS_EVENT_ID_EVENT_CODE_LEN 32
#define LWIS_EVENT_ID_DEVICE_ID_LEN 16

// Event flags used for transaction events.
#define LWIS_TRANSACTION_EVENT_FLAG (1ULL << 63)
#define LWIS_TRANSACTION_FAILURE_EVENT_FLAG (1ULL << 62)
#define LWIS_HW_IRQ_EVENT_FLAG (1ULL << 61)
#define LWIS_PERIODIC_IO_EVENT_FLAG (1ULL << 60)
#define LWIS_OVERFLOW_IRQ_EVENT_FLAG (1ULL << 59)

// Status code for unsignaled LWIS fence
#define LWIS_FENCE_STATUS_NOT_SIGNALED -1

// Interval in ms for the Heartbeat Event if enabled
#define LWIS_HEARTBEAT_EVENT_INTERVAL_MS 10

struct lwis_event_info {
	// IOCTL Inputs
	size_t payload_buffer_size;
	void *payload_buffer;
	// IOCTL Outputs
	int64_t event_id;
	int64_t event_counter;
	int64_t timestamp_ns;
	size_t payload_size;
};

#define LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE (1ULL << 0)
#define LWIS_EVENT_CONTROL_FLAG_QUEUE_ENABLE (1ULL << 1)
#define LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE_ONCE (1ULL << 2)

struct lwis_event_control {
	// IOCTL Inputs
	int64_t event_id;
	// IOCTL Outputs
	uint64_t flags;
};

struct lwis_event_control_list {
	size_t num_event_controls;
	struct lwis_event_control *event_controls;
};

enum lwis_transaction_trigger_node_types {
	LWIS_TRIGGER_EVENT,
	LWIS_TRIGGER_FENCE,
	LWIS_TRIGGER_FENCE_PLACEHOLDER
};

struct lwis_transaction_trigger_event {
	int64_t id;
	int64_t counter;
	int32_t precondition_fence_fd;
};

struct lwis_transaction_trigger_node {
	int32_t type; //lwis_transaction_trigger_node_types
	union {
		int32_t fence_fd;
		struct lwis_transaction_trigger_event event;
	};
};

enum lwis_transaction_trigger_node_operator {
	LWIS_TRIGGER_NODE_OPERATOR_INVALID = -1,
	LWIS_TRIGGER_NODE_OPERATOR_NONE,
	LWIS_TRIGGER_NODE_OPERATOR_AND,
	LWIS_TRIGGER_NODE_OPERATOR_OR,
};

#define LWIS_TRIGGER_NODES_MAX_NUM 16
struct lwis_transaction_trigger_condition {
	size_t num_nodes;
	int32_t operator_type; //lwis_transaction_trigger_node_operator
	struct lwis_transaction_trigger_node trigger_nodes[LWIS_TRIGGER_NODES_MAX_NUM];
};

// Status code for completion fences
#define LWIS_NO_COMPLETION_FENCE -1
#define LWIS_CREATE_COMPLETION_FENCE -2

// Invalid ID for Transaction id and Periodic IO id
#define LWIS_ID_INVALID (-1LL)
#define LWIS_EVENT_COUNTER_ON_NEXT_OCCURRENCE (-1LL)
#define LWIS_EVENT_COUNTER_EVERY_TIME (-2LL)

struct lwis_transaction_info {
	// Input
	int64_t trigger_event_id;
	int64_t trigger_event_counter;
	size_t num_io_entries;
	struct lwis_io_entry *io_entries;
	bool run_in_event_context;
	// Use reserved to keep the original interface
	bool reserved;
	int64_t emit_success_event_id;
	int64_t emit_error_event_id;
	bool is_level_triggered;
	// Output
	int64_t id;
	// Only will be set if trigger_event_id is specified.
	// Otherwise, the value is -1.
	int64_t current_trigger_event_counter;
	int64_t submission_timestamp_ns;
};

struct lwis_transaction_info_v2 {
	// Input
	int64_t trigger_event_id;
	int64_t trigger_event_counter;
	struct lwis_transaction_trigger_condition trigger_condition;
	int32_t completion_fence_fd;
	size_t num_io_entries;
	struct lwis_io_entry *io_entries;
	bool run_in_event_context;
	// Use reserved to keep the original interface
	bool reserved;
	int64_t emit_success_event_id;
	int64_t emit_error_event_id;
	bool is_level_triggered;
	// Output
	int64_t id;
	// Only will be set if trigger_event_id is specified.
	// Otherwise, the value is -1.
	int64_t current_trigger_event_counter;
	int64_t submission_timestamp_ns;
};

// Actual size of this struct depends on num_entries
struct lwis_transaction_response_header {
	int64_t id;
	int32_t error_code;
	int32_t completion_index;
	size_t num_entries;
	size_t results_size_bytes;
};

struct lwis_io_result {
	int32_t bid;
	uint64_t offset;
	size_t num_value_bytes;
	uint8_t values[];
};

struct lwis_periodic_io_info {
	// Input
	int32_t batch_size;
	int64_t period_ns;
	size_t num_io_entries;
	struct lwis_io_entry *io_entries;
	int64_t emit_success_event_id;
	int64_t emit_error_event_id;
	// Output
	int64_t id;
};

// Header of a periodic_io response as a payload of lwis_event_info
// Actual size of this struct depends on batch_size and num_entries_per_period
struct lwis_periodic_io_response_header {
	int64_t id;
	int32_t error_code;
	int32_t batch_size;
	size_t num_entries_per_period;
	size_t results_size_bytes;
};

struct lwis_periodic_io_result {
	int64_t timestamp_ns;
	struct lwis_io_result io_result;
};

struct lwis_dpm_clk_settings {
	struct lwis_clk_setting *settings;
	size_t num_settings;
};

struct lwis_qos_setting {
	// Frequency in hz.
	int64_t frequency_hz;
	// Device id for this vote.
	int32_t device_id;
	// Target clock family.
	int32_t clock_family;
	// read BW
	int64_t read_bw;
	// write BW
	int64_t write_bw;
	// peak BW
	int64_t peak_bw;
	// RT BW (total peak)
	int64_t rt_bw;
};

struct lwis_qos_setting_v2 {
	// Frequency in hz.
	int64_t frequency_hz;
	// Device id for this vote.
	int32_t device_id;
	// Target clock family.
	int32_t clock_family;
	// read BW
	int64_t read_bw;
	// write BW
	int64_t write_bw;
	// peak BW
	int64_t peak_bw;
	// RT BW (total peak)
	int64_t rt_bw;
	// Bts client name
	char bts_block_name[LWIS_MAX_NAME_STRING_LEN];
};

struct lwis_dpm_qos_requirements {
	// qos entities from user.
	struct lwis_qos_setting *qos_settings;
	// number of qos_settings.
	size_t num_settings;
};

struct lwis_dpm_qos_requirements_v2 {
	// qos entities from user.
	struct lwis_qos_setting_v2 *qos_settings;
	// number of qos_settings.
	size_t num_settings;
};

enum lwis_cmd_id {
	LWIS_CMD_ID_ECHO = 0x100,
	LWIS_CMD_ID_TIME_QUERY = 0x200,

	LWIS_CMD_ID_GET_DEVICE_INFO = 0x10000,
	LWIS_CMD_ID_DEVICE_ENABLE = 0x10100,
	LWIS_CMD_ID_DEVICE_DISABLE = 0x10200,
	LWIS_CMD_ID_DEVICE_RESET = 0x10300,
	LWIS_CMD_ID_DEVICE_SUSPEND = 0x10400,
	LWIS_CMD_ID_DEVICE_RESUME = 0x10500,
	LWIS_CMD_ID_DUMP_DEBUG_STATE = 0x10600,

	LWIS_CMD_ID_DMA_BUFFER_ENROLL = 0x20000,
	LWIS_CMD_ID_DMA_BUFFER_DISENROLL = 0x20100,
	LWIS_CMD_ID_DMA_BUFFER_CPU_ACCESS = 0x20200,
	LWIS_CMD_ID_DMA_BUFFER_ALLOC = 0x20300,
	LWIS_CMD_ID_DMA_BUFFER_FREE = 0x20400,

	LWIS_CMD_ID_REG_IO = 0x30000,

	LWIS_CMD_ID_EVENT_CONTROL_GET = 0x40000,
	LWIS_CMD_ID_EVENT_CONTROL_SET = 0x40100,
	LWIS_CMD_ID_EVENT_DEQUEUE = 0x40200,

	LWIS_CMD_ID_TRANSACTION_SUBMIT = 0x50000,
	LWIS_CMD_ID_TRANSACTION_SUBMIT_V2,
	LWIS_CMD_ID_TRANSACTION_CANCEL = 0x50100,
	LWIS_CMD_ID_TRANSACTION_REPLACE = 0x50200,
	LWIS_CMD_ID_TRANSACTION_REPLACE_V2,

	LWIS_CMD_ID_PERIODIC_IO_SUBMIT = 0x60000,
	LWIS_CMD_ID_PERIODIC_IO_CANCEL = 0x60100,

	LWIS_CMD_ID_DPM_CLK_UPDATE = 0x70000,
	LWIS_CMD_ID_DPM_QOS_UPDATE = 0x70100,
	LWIS_CMD_ID_DPM_QOS_UPDATE_V2,
	LWIS_CMD_ID_DPM_GET_CLOCK = 0x70200,

	LWIS_CMD_ID_FENCE_CREATE = 0x80000,

	LWIS_CMD_ID_EVENT_INJECTION = 0x90000
};

struct lwis_cmd_pkt {
	uint32_t cmd_id;
	int32_t ret_code;
	struct lwis_cmd_pkt *next;
};

struct lwis_cmd_echo {
	struct lwis_cmd_pkt header;
	struct lwis_echo msg;
};

struct lwis_cmd_time_query {
	struct lwis_cmd_pkt header;
	int64_t timestamp_ns;
};

struct lwis_cmd_device_info {
	struct lwis_cmd_pkt header;
	struct lwis_device_info info;
};

struct lwis_cmd_io_entries {
	struct lwis_cmd_pkt header;
	struct lwis_io_entries io;
};

struct lwis_cmd_dma_buffer_enroll {
	struct lwis_cmd_pkt header;
	struct lwis_buffer_info info;
};

struct lwis_cmd_dma_buffer_disenroll {
	struct lwis_cmd_pkt header;
	struct lwis_enrolled_buffer_info info;
};

struct lwis_cmd_dma_buffer_cpu_access {
	struct lwis_cmd_pkt header;
	struct lwis_buffer_cpu_access_op op;
};

struct lwis_cmd_dma_buffer_alloc {
	struct lwis_cmd_pkt header;
	struct lwis_alloc_buffer_info info;
};

struct lwis_cmd_dma_buffer_free {
	struct lwis_cmd_pkt header;
	int32_t fd;
};

struct lwis_cmd_event_control_get {
	struct lwis_cmd_pkt header;
	struct lwis_event_control ctl;
};

struct lwis_cmd_event_control_set {
	struct lwis_cmd_pkt header;
	struct lwis_event_control_list list;
};

struct lwis_cmd_event_dequeue {
	struct lwis_cmd_pkt header;
	struct lwis_event_info info;
};

struct lwis_cmd_transaction_info {
	struct lwis_cmd_pkt header;
	struct lwis_transaction_info info;
};

struct lwis_cmd_transaction_info_v2 {
	struct lwis_cmd_pkt header;
	struct lwis_transaction_info_v2 info;
};

struct lwis_cmd_transaction_cancel {
	struct lwis_cmd_pkt header;
	int64_t id;
};

struct lwis_cmd_periodic_io_info {
	struct lwis_cmd_pkt header;
	struct lwis_periodic_io_info info;
};

struct lwis_cmd_periodic_io_cancel {
	struct lwis_cmd_pkt header;
	int64_t id;
};

struct lwis_cmd_dpm_clk_update {
	struct lwis_cmd_pkt header;
	struct lwis_dpm_clk_settings settings;
};

struct lwis_cmd_dpm_qos_update {
	struct lwis_cmd_pkt header;
	struct lwis_dpm_qos_requirements reqs;
};

struct lwis_cmd_dpm_qos_update_v2 {
	struct lwis_cmd_pkt header;
	struct lwis_dpm_qos_requirements_v2 reqs;
};

struct lwis_cmd_dpm_clk_get {
	struct lwis_cmd_pkt header;
	struct lwis_qos_setting setting;
};

struct lwis_cmd_fence_create {
	struct lwis_cmd_pkt header;
	int32_t fd;
};

/*
 *  IOCTL Commands
 */

#define LWIS_IOC_TYPE 'L'
#define LWIS_CMD_PACKET _IOWR(LWIS_IOC_TYPE, 100, struct lwis_cmd_pkt)

/*
 * Event payloads
 */

/* For LWIS_ERROR_EVENT_ID_MEMORY_PAGE_FAULT */
struct lwis_mem_page_fault_event_payload {
	uint64_t fault_address;
	uint64_t fault_flags;
};

#pragma pack(pop)

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* LWIS_COMMANDS_H_ */
