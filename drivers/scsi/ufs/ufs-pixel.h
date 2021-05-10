/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2020 Google LLC
 *
 * Authors: Jaegeuk Kim <jaegeuk@google.com>
 */

#ifndef _UFS_PIXEL_H_
#define _UFS_PIXEL_H_

#include <asm/unaligned.h>
#include "ufshcd.h"

/* 1% lifetime C */
#define HEALTH_DESC_DEFAULT_PE_CYCLE		3000
#define HEALTH_DESC_PARAM_AVG_PE_CYCLE		0xD
#define HEALTH_DESC_PARAM_LIFE_TIME_EST_C	0x24

/* manual gc */
struct ufs_manual_gc {
	int state;
	bool hagc_support;
	struct hrtimer hrtimer;
	unsigned long delay_ms;
	struct work_struct hibern8_work;
	struct workqueue_struct *mgc_workq;
};

#define UFSHCD_MANUAL_GC_HOLD_HIBERN8		2000	/* 2 seconds */

#define QUERY_ATTR_IDN_MANUAL_GC_CONT		0x12
#define QUERY_ATTR_IDN_MANUAL_GC_STATUS		0x13

enum {
	MANUAL_GC_OFF = 0,
	MANUAL_GC_ON,
	MANUAL_GC_DISABLE,
	MANUAL_GC_ENABLE,
	MANUAL_GC_MAX,
};

extern void pixel_init_manual_gc(struct ufs_hba *hba);

/* defined request category on statistics */
enum req_type_stats {
	REQ_TYPE_VALID = 0,
	REQ_TYPE_READ = 1,
	REQ_TYPE_WRITE = 2,
	REQ_TYPE_FLUSH = 3,
	REQ_TYPE_DISCARD = 4,
	REQ_TYPE_SECURITY = 5,
	REQ_TYPE_OTHER = 6,
	REQ_TYPE_MAX = 7,
};

/* request statistic type on sysfs */
enum req_sysfs_stats {
	REQ_SYSFS_MIN = 0,
	REQ_SYSFS_MAX = 1,
	REQ_SYSFS_AVG = 2,
	REQ_SYSFS_SUM = 3,
};

/**
 * struct pixel_req_stats - statistics for request time measurement (usec)
 * @req_min: minimum time of request
 * @req_max: maximum time of request
 * @req_sum: sum of the total request time
 * @req_count: total request count
 */
struct pixel_req_stats {
	u64 req_min;
	u64 req_max;
	u64 req_sum;
	u64 req_count;
};

/* defined I/O amount on statistics */
enum io_type_stats {
	IO_TYPE_READ = 0,
	IO_TYPE_WRITE = 1,
	IO_TYPE_READ_WRITE = 2,
	IO_TYPE_MAX = 3,
};

/**
 * struct pixel_io_stats - statistics for I/O amount.
 * @req_count_started: total number of I/O requests, which were started.
 * @total_bytes_started: total I/O amount in bytes, which were started.
 * @req_count_completed: total number of I/O request, which were completed.
 * @total_bytes_completed: total I/O amount in bytes, which were completed.
 * @max_diff_req_count: MAX of 'req_count_started - req_count_completed'.
 * @max_diff_total_bytes: MAX of 'total_bytes_started - total_bytes_completed'.
 */
struct pixel_io_stats {
	u64 req_count_started;
	u64 total_bytes_started;
	u64 req_count_completed;
	u64 total_bytes_completed;
	u64 max_diff_req_count;
	u64 max_diff_total_bytes;
};

static inline char *parse_opcode(u8 opcode)
{
	/* string should be less than 12 byte-long */
	switch (opcode) {
	case READ_10:
		return "READ_10";
	case READ_16:
		return "READ_16";
	case WRITE_10:
		return "WRITE_10";
	case WRITE_16:
		return "WRITE_16";
	case UNMAP:
		return "UNMAP";
	case ZBC_IN:
		return "ZBC_IN";
	case ZBC_OUT:
		return "ZBC_OUT";
	case SYNCHRONIZE_CACHE:
		return "SYNC_CACHE";
	}
	return NULL;
}

#define PIXEL_MIN_SLOWIO_US            (1000)     /*  1 ms      */
#define PIXEL_DEFAULT_SLOWIO_READ_US   (5000000)  /*  5 seconds */
#define PIXEL_DEFAULT_SLOWIO_WRITE_US  (10000000) /* 10 seconds */
#define PIXEL_DEFAULT_SLOWIO_UNMAP_US  (30000000) /* 30 seconds */
#define PIXEL_DEFAULT_SLOWIO_SYNC_US   (10000000) /* 10 seconds */

/* UFS Slow I/O operation types */
enum pixel_slowio_optype {
	PIXEL_SLOWIO_READ = 0,
	PIXEL_SLOWIO_WRITE = 1,
	PIXEL_SLOWIO_UNMAP = 2,
	PIXEL_SLOWIO_SYNC = 3,
	PIXEL_SLOWIO_OP_MAX = 4,
};

/* UFS Slow I/O sysfs entry types */
enum pixel_slowio_systype {
	PIXEL_SLOWIO_US = 0,
	PIXEL_SLOWIO_CNT = 1,
	PIXEL_SLOWIO_SYS_MAX = 2,
};

struct slowio_attr {
	struct device_attribute attr;
	enum pixel_slowio_optype optype;
	enum pixel_slowio_systype systype;
};

extern void pixel_init_slowio(struct ufs_hba *hba);

/* UFS err_stats type */
enum pixel_err_systype {
	PIXEL_ERR_COUNT = 0,
	PIXEL_ERR_TIME,
};

/**
 * struct pixel_ufs_stats - statistics for ufs host parameters
 * @hibern8_total_us: the total time of staying hibern8 mode (us)
 * @hibern8_exit_cnt: the count of exiting hibern8 mode
 * @last_hibern8_enter_time: the last timing which entered hibern8 mode (us)
 * @last_hibern8_exit_time: the last timing which exited hibern8 mode (us)
 */
struct pixel_ufs_stats {
	bool hibern8_flag;
	u64 hibern8_total_us;
	u32 hibern8_exit_cnt;
	u64 last_hibern8_enter_time;
	u64 last_hibern8_exit_time;
};

extern int pixel_init(struct ufs_hba *hba);
extern void pixel_exit(struct ufs_hba *hba);
extern void pixel_ufs_record_hibern8(struct ufs_hba *hba, bool is_enter_h8);
extern void pixel_print_cmd_log(struct ufs_hba *hba);

enum pixel_event_type {
	EVENT_UNDEF = 0,
	EVENT_DME_SEND,
	EVENT_DME_COMPL,
	EVENT_SCSI_SEND,
	EVENT_SCSI_COMPL,
	EVENT_NOP_OUT,
	EVENT_NOP_IN,
	EVENT_QUERY_SEND,
	EVENT_QUERY_COMPL,
	EVENT_TM_SEND,
	EVENT_TM_ERR,
	EVENT_TM_COMPL,
	EVENT_INTR_FATAL_ERR,
	EVENT_INTR_UIC_ERR,
	EVENT_INTR_H8_ERR,
	EVENT_TYPE_MAX,
};

enum pixel_command_type {
	CMD_UNDEF = 0,
	/* dme cmd */
	CMD_DME_GET,
	CMD_DME_SET,
	CMD_DME_PWR_ON,
	CMD_DME_PWR_OFF,
	CMD_DME_RESET,
	CMD_DME_LINKSTARTUP,
	CMD_DME_H8_ENTER,
	CMD_DME_H8_EXIT,
	/* scsi cmd */
	CMD_SCSI_WRITE_10,
	CMD_SCSI_READ_10,
	CMD_SCSI_WRITE_16,
	CMD_SCSI_READ_16,
	CMD_SCSI_SYNC,
	CMD_SCSI_UNMAP,
	CMD_SCSI_SSU,
	CMD_SCSI_PROTOCOL_IN,
	CMD_SCSI_PROTOCOL_OUT,
	CMD_SCSI_ZBC_IN,
	CMD_SCSI_ZBC_OUT,
	/* query cmd */
	CMD_TYPE_MAX,
};

struct pixel_cmd_log_entry {
	u8  *event;
	u8  *cmd;
	u8  opcode;
	u8  lun;
	u8  idn;
	sector_t sector;
	s32 affected_bytes;
	u64 doorbell;
	u64 outstanding_reqs;
	u32 seq_num;
	s32 tag;
	u8  group_id;
	ktime_t tstamp;
	u64 error;
	u8 queue_eh_work;
};

#define MAX_CMD_ENTRY_NUM       200
#define MAX_EVENT_STR_LEN       16
#define MAX_CMD_STR_LEN         16

struct pixel_cmd_log {
	struct pixel_cmd_log_entry *entry;
	u32 head;
	u32 seq_cnt;
	u8 *event_str[EVENT_TYPE_MAX];
	u8 *cmd_str[CMD_TYPE_MAX];
};
#endif
