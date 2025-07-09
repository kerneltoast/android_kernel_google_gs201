/* SPDX-License-Identifier: GPL-2.0-only */
/* BEGIN-INTERNAL */
/*
 * Google LWIS IOCTL Commands and Data Structures
 *
 * Copyright (c) 2018 Google, LLC
 */
/* END-INTERNAL */

/*
 * Since we require backward compatibility, we need to be able to handle
 * several versions of the same command. In this file, we keep all the versions
 * we want to handle. When we need changes to a command, we need to create a
 * new version of the command.
 *
 * As convention, we will only give a version number to the old versions of a
 * structure. For instance, if we have a structure `a`, we could have:
 *
 *	struct a_v1 {
 *		int x;
 *	};
 *	struct a {
 *		int x;
 *		int y;
 *	};
 *
 * Here, structure `a` is the latest version and `a_v1` is the old version
 * without the changes required in `a`. If we want a new version of `a`, we'll do:
 *
 *	struct a_v1 {
 *		int x;
 *	};
 *	struct a_v2 {
 *		int x;
 *		int y;
 *	};
 *	struct a {
 *		int x;
 *		int y;
 *		int z;
 *	};
 *
 * Having version numbers only for the old versions have two main advantages:
 * (1) We don't need to change the code everywhere when creating a new version
 * of a strucure because the symbol name will stay the same and (2) reviews
 * will clearly show what changed in the new version.
 *
 * Another advantage is that since versioned structures are the old structures,
 * they will show only whenever we need to handle old versions/command/APIs.
 *
 * While we transition to this new convention, the latest version of a
 * structure may be a version number. We should move to this new convention as
 * we make new changes to the structures.
 */

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
 * spi  : for controlling spi devices
 */
enum lwis_device_types {
	DEVICE_TYPE_UNKNOWN = -1,
	DEVICE_TYPE_TOP,
	DEVICE_TYPE_I2C,
	DEVICE_TYPE_IOREG,
	DEVICE_TYPE_SLC,
	DEVICE_TYPE_DPM,
	DEVICE_TYPE_TEST,
	DEVICE_TYPE_SPI,
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
	LWIS_IO_ENTRY_READ_ASSERT,
	LWIS_IO_ENTRY_POLL_SHORT,
	LWIS_IO_ENTRY_WAIT,
	LWIS_IO_ENTRY_WRITE_TO_BUFFER,
	LWIS_IO_ENTRY_READ_V2,
	LWIS_IO_ENTRY_READ_BATCH_V2,
	LWIS_IO_ENTRY_WRITE_V2,
	LWIS_IO_ENTRY_WRITE_BATCH_V2,
	LWIS_IO_ENTRY_IGNORE
};

// For io_entry read and write types.
struct lwis_io_entry_rw {
	int32_t bid;
	uint64_t offset;
	uint64_t val;
};

struct lwis_io_entry_rw_v2 {
	int32_t bid;
	uint64_t offset;
	uint64_t val;
	uint32_t speed_hz;
};

struct lwis_io_entry_rw_batch {
	int32_t bid;
	uint64_t offset;
	size_t size_in_bytes;
	uint8_t *buf;
	bool is_offset_fixed;
};

struct lwis_io_entry_rw_batch_v2 {
	int32_t bid;
	uint64_t offset;
	size_t size_in_bytes;
	uint8_t *buf;
	bool is_offset_fixed;
	uint32_t speed_hz;
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

struct pdma_buffer {
	/* kernel use only */
	void *io_sys_map;
	void *dma_buf;
};

// For io_entry write to buffer.
struct lwis_io_entry_write_to_buffer {
	union {
		int fd;
		struct pdma_buffer *buffer;
	};
	uint64_t offset;
	size_t size_in_bytes;
	uint8_t *bytes;
};

struct lwis_io_entry {
	int32_t type;
	union {
		struct lwis_io_entry_rw rw;
		struct lwis_io_entry_rw_batch rw_batch;
		struct lwis_io_entry_modify mod;
		struct lwis_io_entry_read_assert read_assert;
		uint64_t wait_us;
		struct lwis_io_entry_write_to_buffer write_to_buffer;
		struct lwis_io_entry_rw_v2 rw_v2;
		struct lwis_io_entry_rw_batch_v2 rw_batch_v2;
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
enum lwis_fence_v0_status {
	LWIS_FENCE_V0_STATUS_NOT_SIGNALED = -1,
};
enum lwis_fence_status {
	LWIS_FENCE_STATUS_NOT_SIGNALED = 0,
	LWIS_FENCE_STATUS_SUCCESSFULLY_SIGNALED = 1,
};

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

struct lwis_transaction_trigger_node_v5 {
	int32_t type; //lwis_transaction_trigger_node_types
	union {
		int32_t fence_fd;
		struct lwis_transaction_trigger_event event;
	};
};
struct lwis_transaction_trigger_node {
	int32_t type; //lwis_transaction_trigger_node_types
	union {
		struct {
			int32_t fence_fd;
			int32_t fence_signal_fd;
		};
		struct lwis_transaction_trigger_event event;
	};
};

enum lwis_transaction_trigger_node_operator {
	LWIS_TRIGGER_NODE_OPERATOR_INVALID = -1,
	LWIS_TRIGGER_NODE_OPERATOR_NONE,
	LWIS_TRIGGER_NODE_OPERATOR_AND,
	LWIS_TRIGGER_NODE_OPERATOR_OR,
};

#define LWIS_NESTED_TRANSACTION_MAX 8
#define LWIS_TRIGGER_NODES_MAX_NUM 16
struct lwis_transaction_trigger_condition_v5 {
	size_t num_nodes;
	int32_t operator_type; //lwis_transaction_trigger_node_operator
	struct lwis_transaction_trigger_node_v5 trigger_nodes[LWIS_TRIGGER_NODES_MAX_NUM];
};
struct lwis_transaction_trigger_condition {
	size_t num_nodes;
	int32_t operator_type; //lwis_transaction_trigger_node_operator
	struct lwis_transaction_trigger_node trigger_nodes[LWIS_TRIGGER_NODES_MAX_NUM];
};

// Status code for completion fences
#define LWIS_NO_COMPLETION_FENCE -1
#define LWIS_CREATE_COMPLETION_FENCE -2
#define LWIS_COMPLETION_FENCE_MAX 8

// Invalid ID for Transaction id and Periodic IO id
#define LWIS_ID_INVALID (-1LL)
#define LWIS_EVENT_COUNTER_ON_NEXT_OCCURRENCE (-1LL)
#define LWIS_EVENT_COUNTER_EVERY_TIME (-2LL)

// LWIS IO_ENTRY transaction overflow restriction
#define LWIS_IO_ENTRY_READ_RESTRICTION (SHRT_MAX - sizeof(struct lwis_transaction_response_header))
#define LWIS_IO_ENTRY_READ_OVERFLOW_BOUND                                                          \
	(LWIS_IO_ENTRY_READ_RESTRICTION / sizeof(struct lwis_io_entry))
struct lwis_transaction_info_v2 {
	// Input
	int64_t trigger_event_id;
	int64_t trigger_event_counter;
	struct lwis_transaction_trigger_condition_v5 trigger_condition;
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

struct lwis_transaction_info_v3 {
	// Input
	int64_t trigger_event_id;
	int64_t trigger_event_counter;
	struct lwis_transaction_trigger_condition_v5 trigger_condition;
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
	bool is_high_priority_transaction;
	char transaction_name[LWIS_MAX_NAME_STRING_LEN];
};

struct lwis_transaction_info_v4 {
	// Input
	int64_t trigger_event_id;
	int64_t trigger_event_counter;
	struct lwis_transaction_trigger_condition_v5 trigger_condition;
	int32_t completion_fence_fd;
	size_t num_io_entries;
	struct lwis_io_entry *io_entries;
	bool run_in_event_context;
	// Use reserved to keep the original interface
	bool reserved;
	int64_t emit_success_event_id;
	int64_t emit_error_event_id;
	bool is_level_triggered;
	bool is_high_priority_transaction;
	char transaction_name[LWIS_MAX_NAME_STRING_LEN];
	size_t num_nested_transactions;
	int64_t nested_transaction_ids[LWIS_NESTED_TRANSACTION_MAX];
	// Output
	int64_t id;
	// Only will be set if trigger_event_id is specified.
	// Otherwise, the value is -1.
	int64_t current_trigger_event_counter;
	int64_t submission_timestamp_ns;
};

struct lwis_transaction_info_v5 {
	// Input
	int64_t trigger_event_id;
	int64_t trigger_event_counter;
	struct lwis_transaction_trigger_condition_v5 trigger_condition;
	// Used to indicate a completion fence should be created for this transaction.
	// The created completion fence file descriptor is returned in this variable.
	int32_t create_completion_fence_fd;
	size_t num_io_entries;
	struct lwis_io_entry *io_entries;
	bool run_in_event_context;
	// Use reserved to keep the original interface
	bool reserved;
	int64_t emit_success_event_id;
	int64_t emit_error_event_id;
	bool is_level_triggered;
	bool is_high_priority_transaction;
	char transaction_name[LWIS_MAX_NAME_STRING_LEN];
	size_t num_nested_transactions;
	int64_t nested_transaction_ids[LWIS_NESTED_TRANSACTION_MAX];
	size_t num_completion_fences;
	int32_t completion_fence_fds[LWIS_COMPLETION_FENCE_MAX];
	// Output
	int64_t id;
	// Only will be set if trigger_event_id is specified.
	// Otherwise, the value is -1.
	int64_t current_trigger_event_counter;
	int64_t submission_timestamp_ns;
};

struct lwis_transaction_info {
	// Input
	int64_t trigger_event_id;
	int64_t trigger_event_counter;
	struct lwis_transaction_trigger_condition trigger_condition;
	// Used to indicate a completion fence should be created for this transaction.
	// The created completion fence file descriptor is returned in this variable.
	int32_t create_completion_fence_fd;
	int32_t create_completion_fence_signal_fd;
	size_t num_io_entries;
	struct lwis_io_entry *io_entries;
	bool run_in_event_context;
	// Use reserved to keep the original interface
	bool reserved;
	int64_t emit_success_event_id;
	int64_t emit_error_event_id;
	bool is_level_triggered;
	bool is_high_priority_transaction;
	char transaction_name[LWIS_MAX_NAME_STRING_LEN];
	size_t num_nested_transactions;
	int64_t nested_transaction_ids[LWIS_NESTED_TRANSACTION_MAX];
	size_t num_completion_fences;
	int32_t completion_fence_fds[LWIS_COMPLETION_FENCE_MAX];
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

struct lwis_qos_setting_v3 {
	// Frequency in hz either clock_family or qos_family_name is valid
	int64_t frequency_hz;
	// Device id for this vote.
	int32_t device_id;
	// Target clock family.
	int32_t clock_family;
	// The following Bandwidth in KBytes if clock_family is valid
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
	// The following Bandwidth in MBytes if qos_family_name is valid
	// read constraints
	uint32_t read_avg_bw;
	uint32_t read_peak_bw;
	uint32_t read_latency;
	// read latency tolerance value
	uint32_t read_ltv;
	uint8_t read_vc;
	// write BW
	uint32_t write_avg_bw;
	uint32_t write_peak_bw;
	uint32_t write_latency;
	// write latency tolerance value
	uint32_t write_ltv;
	uint8_t write_vc;
	// Target string qos family.
	char qos_family_name[LWIS_MAX_NAME_STRING_LEN];
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

struct lwis_dpm_qos_requirements_v3 {
	// qos entities from user.
	struct lwis_qos_setting_v3 *qos_settings;
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
	LWIS_CMD_ID_GET_DEVICE_ENABLE_STATE = 0x10700,

	LWIS_CMD_ID_DMA_BUFFER_ENROLL = 0x20000,
	LWIS_CMD_ID_DMA_BUFFER_DISENROLL = 0x20100,
	LWIS_CMD_ID_DMA_BUFFER_CPU_ACCESS = 0x20200,
	LWIS_CMD_ID_DMA_BUFFER_ALLOC = 0x20300,
	LWIS_CMD_ID_DMA_BUFFER_FREE = 0x20400,

	LWIS_CMD_ID_REG_IO = 0x30000,
	LWIS_CMD_ID_REG_IO_V2,

	LWIS_CMD_ID_EVENT_CONTROL_GET = 0x40000,
	LWIS_CMD_ID_EVENT_CONTROL_SET = 0x40100,
	LWIS_CMD_ID_EVENT_DEQUEUE = 0x40200,

	LWIS_CMD_ID_TRANSACTION_SUBMIT_V2 = 0x50001,
	LWIS_CMD_ID_TRANSACTION_SUBMIT_V3,
	LWIS_CMD_ID_TRANSACTION_SUBMIT_V4,
	LWIS_CMD_ID_TRANSACTION_SUBMIT_V5,
	LWIS_CMD_ID_TRANSACTION_SUBMIT,

	LWIS_CMD_ID_TRANSACTION_CANCEL = 0x50100,

	LWIS_CMD_ID_PERIODIC_IO_SUBMIT = 0x60000,
	LWIS_CMD_ID_PERIODIC_IO_CANCEL = 0x60100,

	LWIS_CMD_ID_DPM_CLK_UPDATE = 0x70000,
	LWIS_CMD_ID_DPM_QOS_UPDATE = 0x70100,
	LWIS_CMD_ID_DPM_QOS_UPDATE_V2,
	LWIS_CMD_ID_DPM_QOS_UPDATE_V3,
	LWIS_CMD_ID_DPM_GET_CLOCK = 0x70200,

	LWIS_CMD_ID_FENCE_CREATE_V0 = 0x80000,
	LWIS_CMD_ID_FENCE_CREATE,

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

enum lwis_device_enable_state {
	DEVICE_ENABLE_STATE_INVALID = -1,
	DEVICE_ENABLE_STATE_DISABLE,
	DEVICE_ENABLE_STATE_ENABLE,
	DEVICE_ENABLE_STATE_SUSPEND
};

struct lwis_cmd_get_device_enable_state {
	struct lwis_cmd_pkt header;
	int32_t state;
};

struct lwis_cmd_io_entries {
	struct lwis_cmd_pkt header;
	struct lwis_io_entries io;
};

struct lwis_cmd_io_entries_v2 {
	struct lwis_cmd_pkt header;
	struct lwis_io_entries io;
	bool skip_error;
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

struct lwis_cmd_transaction_info_v2 {
	struct lwis_cmd_pkt header;
	struct lwis_transaction_info_v2 info;
};

struct lwis_cmd_transaction_info_v3 {
	struct lwis_cmd_pkt header;
	struct lwis_transaction_info_v3 info;
};

struct lwis_cmd_transaction_info_v4 {
	struct lwis_cmd_pkt header;
	struct lwis_transaction_info_v4 info;
};

struct lwis_cmd_transaction_info_v5 {
	struct lwis_cmd_pkt header;
	struct lwis_transaction_info_v5 info;
};

struct lwis_cmd_transaction_info {
	struct lwis_cmd_pkt header;
	struct lwis_transaction_info info;
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

struct lwis_cmd_dpm_qos_update_v3 {
	struct lwis_cmd_pkt header;
	struct lwis_dpm_qos_requirements_v3 reqs;
};

struct lwis_cmd_dpm_clk_get {
	struct lwis_cmd_pkt header;
	struct lwis_qos_setting setting;
};

struct lwis_cmd_fence_create_v0 {
	struct lwis_cmd_pkt header;
	int32_t fd;
};

struct lwis_cmd_fence_create {
	struct lwis_cmd_pkt header;
	int32_t fd;
	int32_t signal_fd;
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
