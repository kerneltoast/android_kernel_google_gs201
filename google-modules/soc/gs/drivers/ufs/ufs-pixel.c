// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2020 Google LLC
 *
 * Authors: Jaegeuk Kim <jaegeuk@google.com>
 */

#include <core/ufshcd-priv.h>
#include <linux/workqueue.h>
#include <misc/sbbm.h>
#include "ufs-pixel.h"
#include "ufs-pixel-crypto.h"
#if IS_ENABLED(CONFIG_SCSI_UFS_PIXEL_FIPS140)
#include "ufs-pixel-fips.h"
#endif

#define CREATE_TRACE_POINTS
#include <trace/events/ufs_pixel.h>

#undef CREATE_TRACE_POINTS
#include <trace/hooks/ufshcd.h>

/* UFSHCD error handling flags */
enum {
	UFSHCD_EH_IN_PROGRESS = (1 << 0),
};

#define ufshcd_eh_in_progress(h) \
	((h)->eh_flags & UFSHCD_EH_IN_PROGRESS)

static const char *const ufs_event_str[] = {
	[EVENT_UNDEF] = "event_undef",
	[EVENT_DME_SEND] = "dme_send",
	[EVENT_DME_COMPL] = "dme_compl",
	[EVENT_SCSI_SEND] = "scsi_send",
	[EVENT_SCSI_COMPL] = "scsi_compl",
	[EVENT_NOP_OUT] = "nop_out",
	[EVENT_NOP_IN] = "nop_in",
	[EVENT_QUERY_SEND] = "query_send",
	[EVENT_QUERY_COMPL] = "query_compl",
	[EVENT_TM_SEND] = "tm_send",
	[EVENT_TM_ERR] = "tm_err",
	[EVENT_TM_COMPL] = "tm_compl",
	[EVENT_INTR_FATAL_ERR] = "intr_fatal_err",
	[EVENT_INTR_UIC_ERR] = "intr_uic_err",
	[EVENT_INTR_H8_ERR] = "intr_h8_err",
};

static const char *const ufs_cmd_str[] = {
	[CMD_UNDEF] = "cmd_undef",
	[CMD_DME_GET] = "dme_get",
	[CMD_DME_SET] = "dme_set",
	[CMD_DME_PWR_ON] = "dme_pwr_on",
	[CMD_DME_PWR_OFF] = "dme_pwr_off",
	[CMD_DME_RESET] = "dme_reset",
	[CMD_DME_LINKSTARTUP] = "dme_linkstartup",
	[CMD_DME_H8_ENTER] = "dme_h8_enter",
	[CMD_DME_H8_EXIT] = "dme_h8_exit",
	[CMD_SCSI_WRITE_10] = "write_10",
	[CMD_SCSI_READ_10] = "read_10",
	[CMD_SCSI_WRITE_16] = "write_16",
	[CMD_SCSI_READ_16] = "read_16",
	[CMD_SCSI_SYNC] = "sync",
	[CMD_SCSI_UNMAP] = "unmap",
	[CMD_SCSI_SSU] = "ssu",
	[CMD_SCSI_PROTOCOL_IN] = "protocol_in",
	[CMD_SCSI_PROTOCOL_OUT] = "protocol_out",
	[CMD_SCSI_ZBC_IN] = "zbc_in: report_zone",
	[CMD_SCSI_ZBC_OUT] = "zbc_out: zone_reset",
};

void pixel_update_power_event(struct ufs_hba *hba,
			      enum pixel_power_event_type e)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	unsigned long flags;

	if (!ufs->power_event_mode)
		return;

	switch (e) {
	case PE_H8_EXIT:
		SBBM_SIGNAL_UPDATE(SBB_SIG_UFS_ACTIVE_IDLE, 1);
		break;
	case PE_H8_ENTER:
		SBBM_SIGNAL_UPDATE(SBB_SIG_UFS_ACTIVE_IDLE, 0);
		break;
	case PE_SYSTEM_RESUME:
		SBBM_SIGNAL_UPDATE(SBB_SIG_UFS_POWER_ON, 1);
		break;
	case PE_SYSTEM_SUSPEND:
		SBBM_SIGNAL_UPDATE(SBB_SIG_UFS_POWER_ON, 0);
		break;
	case PE_IO_ISSUE:
		spin_lock_irqsave(&ufs->power_event_lock, flags);
		ufs->outstanding_io++;
		if (ufs->outstanding_io == 1)
			SBBM_SIGNAL_UPDATE(SBB_SIG_UFS_IO_OUTSTANDING, 1);
		spin_unlock_irqrestore(&ufs->power_event_lock, flags);
		break;
	case PE_IO_COMPLETE:
		spin_lock_irqsave(&ufs->power_event_lock, flags);
		/*
		 * Handle case where feature was enabled while I/O were already in
		 * flight and unaccounted for
		 */
		if (ufs->outstanding_io)
			ufs->outstanding_io--;
		if (!ufs->outstanding_io)
			SBBM_SIGNAL_UPDATE(SBB_SIG_UFS_IO_OUTSTANDING, 0);
		spin_unlock_irqrestore(&ufs->power_event_lock, flags);
		break;
	default:
		dev_err(hba->dev, "Invalid power event %u\n", e);
	}
}

static void pixel_ufs_update_slowio_min_us(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	enum pixel_slowio_optype i;
	u64 us;

	ufs->slowio_min_us = ufs->slowio[PIXEL_SLOWIO_READ][PIXEL_SLOWIO_US];
	for (i = PIXEL_SLOWIO_WRITE; i < PIXEL_SLOWIO_OP_MAX; i++) {
		us = ufs->slowio[i][PIXEL_SLOWIO_US];
		if (us < ufs->slowio_min_us)
			ufs->slowio_min_us = us;
	}
}

void pixel_init_slowio(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);

	/* Set default slow io value. */
	ufs->slowio[PIXEL_SLOWIO_READ][PIXEL_SLOWIO_US] =
		PIXEL_DEFAULT_SLOWIO_READ_US;
	ufs->slowio[PIXEL_SLOWIO_WRITE][PIXEL_SLOWIO_US] =
		PIXEL_DEFAULT_SLOWIO_WRITE_US;
	ufs->slowio[PIXEL_SLOWIO_UNMAP][PIXEL_SLOWIO_US] =
		PIXEL_DEFAULT_SLOWIO_UNMAP_US;
	ufs->slowio[PIXEL_SLOWIO_SYNC][PIXEL_SLOWIO_US] =
		PIXEL_DEFAULT_SLOWIO_SYNC_US;
	pixel_ufs_update_slowio_min_us(hba);
}

static enum pixel_slowio_optype pixel_ufs_get_slowio_optype(u8 opcode)
{
	if (opcode == READ_10 || opcode == READ_16)
		return PIXEL_SLOWIO_READ;
	else if (opcode == WRITE_10 || opcode == WRITE_16)
		return PIXEL_SLOWIO_WRITE;
	else if (opcode == UNMAP)
		return PIXEL_SLOWIO_UNMAP;
	else if (opcode == SYNCHRONIZE_CACHE)
		return PIXEL_SLOWIO_SYNC;
	return PIXEL_SLOWIO_OP_MAX;
}

static void pixel_ufs_log_slowio(struct ufs_hba *hba,
		struct ufshcd_lrb *lrbp, s64 iotime_us)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	sector_t sector = ULONG_MAX;
	u32 affected_bytes = UINT_MAX;
	u8 opcode = 0xff;
	char opcode_str[16];
	u64 slowio_cnt = 0;
	enum pixel_slowio_optype optype;

	/* For common case */
	if (likely(iotime_us < ufs->slowio_min_us))
		return;

	if (lrbp->cmd) {
		opcode = (u8)(*lrbp->cmd->cmnd);
		optype = pixel_ufs_get_slowio_optype(opcode);
		if (optype < PIXEL_SLOWIO_OP_MAX) {
			if (iotime_us < ufs->slowio[optype][PIXEL_SLOWIO_US])
				return;
			slowio_cnt = ++ufs->slowio[optype][PIXEL_SLOWIO_CNT];
		}
		if (optype == PIXEL_SLOWIO_READ ||
			optype == PIXEL_SLOWIO_WRITE ||
			optype == PIXEL_SLOWIO_UNMAP) {
			struct request *rq = scsi_cmd_to_rq(lrbp->cmd);
			if (rq && rq->bio) {
				sector = blk_rq_pos(rq);
				affected_bytes = blk_rq_bytes(rq);
			}
		}
	}
	snprintf(opcode_str, 16, "%02x: %s", opcode, parse_opcode(opcode));
	dev_err_ratelimited(hba->dev,
		"Slow UFS (%lld): time = %lld us, opcode = %16s, sector = %lld, "
		"len = %u\n",
		slowio_cnt, iotime_us, opcode_str, sector, affected_bytes);
}

/* classify request type on statistics by scsi command opcode*/
static int pixel_ufs_get_cmd_type(struct ufshcd_lrb *lrbp,
				  enum req_type_stats *cmd_type)
{
	u8 scsi_op_code;

	if (!lrbp->cmd)
		return -EINVAL;

	scsi_op_code = (u8)(*lrbp->cmd->cmnd);
	switch (scsi_op_code) {
	case READ_10:
	case READ_16:
		*cmd_type = REQ_TYPE_READ;
		break;
	case WRITE_10:
	case WRITE_16:
		*cmd_type = REQ_TYPE_WRITE;
		break;
	case UNMAP:
		*cmd_type = REQ_TYPE_DISCARD;
		break;
	case SYNCHRONIZE_CACHE:
		*cmd_type = REQ_TYPE_FLUSH;
		break;
	case SECURITY_PROTOCOL_IN:
	case SECURITY_PROTOCOL_OUT:
		*cmd_type = REQ_TYPE_SECURITY;
		break;
	default:
		*cmd_type = REQ_TYPE_OTHER;
		break;
	}
	return 0;
}

/* record_ufs_stats() is following mm/mm_event.c style */
static const unsigned long period_ms = 3000;
static unsigned long next_period_ufs_stats;

static u64 pixel_ufs_prev_sum[REQ_TYPE_MAX];
static u64 pixel_ufs_prev_count[REQ_TYPE_MAX];

static inline void __sync_io_stats(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	int cpu;

	if (!ufs->io_stats)
		return;

	memcpy(&ufs->prev_io_stats, &ufs->curr_io_stats,
		sizeof(ufs->prev_io_stats));
	memset(&ufs->curr_io_stats, 0,
		sizeof(ufs->prev_io_stats));

	for_each_possible_cpu(cpu) {
		struct pixel_io_stats *ptr =
			per_cpu_ptr(ufs->io_stats, cpu);
		ufs->curr_io_stats.r_req_count_started +=
			ptr->r_req_count_started;
		ufs->curr_io_stats.r_req_count_completed +=
			 ptr->r_req_count_completed;
		ufs->curr_io_stats.r_total_bytes_started +=
			 ptr->r_total_bytes_started;
		ufs->curr_io_stats.r_total_bytes_completed +=
			 ptr->r_total_bytes_completed;
		ufs->curr_io_stats.w_req_count_started +=
			 ptr->w_req_count_started;
		ufs->curr_io_stats.w_req_count_completed +=
			 ptr->w_req_count_completed;
		ufs->curr_io_stats.w_total_bytes_started +=
			 ptr->w_total_bytes_started;
		ufs->curr_io_stats.w_total_bytes_completed +=
			 ptr->w_total_bytes_completed;
		ufs->curr_io_stats.rw_req_count_started +=
			 ptr->rw_req_count_started;
		ufs->curr_io_stats.rw_req_count_completed +=
			 ptr->rw_req_count_completed;
		ufs->curr_io_stats.rw_total_bytes_started +=
			 ptr->rw_total_bytes_started;
		ufs->curr_io_stats.rw_total_bytes_completed +=
			 ptr->rw_total_bytes_completed;
	}

	ufs->curr_io_stats.r_max_diff_req_count =
		ufs->curr_io_stats.r_req_count_started -
		ufs->curr_io_stats.r_req_count_completed;
	ufs->curr_io_stats.r_max_diff_total_bytes =
		ufs->curr_io_stats.r_total_bytes_started -
		ufs->curr_io_stats.r_total_bytes_completed;
	ufs->curr_io_stats.w_max_diff_req_count =
		ufs->curr_io_stats.w_req_count_started -
		ufs->curr_io_stats.w_req_count_completed;
	ufs->curr_io_stats.w_max_diff_total_bytes =
		ufs->curr_io_stats.w_total_bytes_started -
		ufs->curr_io_stats.w_total_bytes_completed;
	ufs->curr_io_stats.rw_max_diff_req_count =
		ufs->curr_io_stats.rw_req_count_started -
		ufs->curr_io_stats.rw_req_count_completed;
	ufs->curr_io_stats.rw_max_diff_total_bytes =
		ufs->curr_io_stats.rw_total_bytes_started -
		ufs->curr_io_stats.rw_total_bytes_completed;
	ufs->peak_queue_depth = ufs->curr_io_stats.rw_max_diff_req_count;
}

static inline void record_ufs_stats(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	int i;
	u64 avg_time[REQ_TYPE_MAX] = { 0, };

	if (time_is_after_jiffies(next_period_ufs_stats))
		return;
	next_period_ufs_stats = jiffies + msecs_to_jiffies(period_ms);

	for (i = 0; i < REQ_TYPE_MAX; i++) {
		u64 count_diff = ufs->req_stats[i].req_count
					- pixel_ufs_prev_count[i];

		if (count_diff) {
			u64 sum_diff = ufs->req_stats[i].req_sum
					- pixel_ufs_prev_sum[i];
			avg_time[i] = div64_u64(sum_diff, count_diff);
		}
	}

	__sync_io_stats(hba);
	trace_ufs_stats(ufs, avg_time);

	for (i = 0; i < REQ_TYPE_MAX; i++) {
		ufs->peak_reqs[i] = 0;
		pixel_ufs_prev_sum[i] = ufs->req_stats[i].req_sum;
		pixel_ufs_prev_count[i] = ufs->req_stats[i].req_count;
	}
}

void pixel_ufs_update_req_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	struct pixel_req_stats *rst;
	enum req_type_stats cmd_type;
	u64 delta = (u64)ktime_us_delta(lrbp->compl_time_stamp,
		lrbp->issue_time_stamp);

	/* Log for slow I/O */
	pixel_ufs_log_slowio(hba, lrbp, delta);

	if (pixel_ufs_get_cmd_type(lrbp, &cmd_type))
		return;

	/* Update request statistic if need */
	rst = &(ufs->req_stats[REQ_TYPE_VALID]);
	rst->req_count++;
	rst->req_sum += delta;
	if (rst->req_min == 0 || rst->req_min > delta)
		rst->req_min = delta;
	if (rst->req_max < delta)
		rst->req_max = delta;
	if (delta > ufs->peak_reqs[REQ_TYPE_VALID])
		ufs->peak_reqs[REQ_TYPE_VALID] = delta;

	rst = &(ufs->req_stats[cmd_type]);
	rst->req_count++;
	rst->req_sum += delta;
	if (rst->req_min == 0 || rst->req_min > delta)
		rst->req_min = delta;
	if (rst->req_max < delta)
		rst->req_max = delta;
	if (delta > ufs->peak_reqs[cmd_type])
		ufs->peak_reqs[cmd_type] = delta;

	record_ufs_stats(hba);
}

static inline void __update_io_stats(struct ufs_hba *hba,
				     bool is_write,
				     u32 affected_bytes, bool is_start)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	struct pixel_io_stats *s;

	if (!ufs->io_stats)
		return;

	s = get_cpu_ptr(ufs->io_stats);
	if (is_start) {
		s->rw_req_count_started++;
		s->rw_total_bytes_started += affected_bytes;

		if (!is_write) {
			s->r_req_count_started++;
			s->r_total_bytes_started += affected_bytes;
		} else {
			s->w_req_count_started++;
			s->w_total_bytes_started += affected_bytes;
		}
	} else {
		s->rw_req_count_completed++;
		s->rw_total_bytes_completed += affected_bytes;

		if (!is_write) {
			s->r_req_count_completed++;
			s->r_total_bytes_completed += affected_bytes;
		} else {
			s->w_req_count_completed++;
			s->w_total_bytes_completed += affected_bytes;
		}
	}
	put_cpu_ptr(s);
}

static void pixel_ufs_update_io_stats(struct ufs_hba *hba,
		struct ufshcd_lrb *lrbp, bool is_start)
{
	u32 affected_bytes;
	bool is_write;
	enum req_type_stats cmd_type;

	if (pixel_ufs_get_cmd_type(lrbp, &cmd_type))
		return;

	if (cmd_type != REQ_TYPE_READ && cmd_type != REQ_TYPE_WRITE)
		return;

	affected_bytes = blk_rq_bytes(scsi_cmd_to_rq(lrbp->cmd));

	/* Upload I/O amount on statistic */
	is_write = op_is_write(req_op(scsi_cmd_to_rq(lrbp->cmd)));
	__update_io_stats(hba, is_write, affected_bytes, is_start);

	record_ufs_stats(hba);

	if (is_start)
		pixel_update_power_event(hba, PE_IO_ISSUE);
	else
		pixel_update_power_event(hba, PE_IO_COMPLETE);
}

void pixel_ufs_record_hibern8(struct ufs_hba *hba, bool is_enter_h8)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	struct pixel_ufs_stats *ufs_stats = &ufs->ufs_stats;
	u64 curr_t = ktime_to_us(ktime_get());

	if (is_enter_h8) {
		ufs_stats->last_hibern8_enter_time = curr_t;
		ufs_stats->hibern8_flag = true;
		pixel_update_power_event(hba, PE_H8_ENTER);
	} else {
		ufs_stats->last_hibern8_exit_time = curr_t;
		if (ufs_stats->hibern8_flag) {
			/* calculate time & count when pair cases */
			ufs_stats->hibern8_total_us += curr_t -
				ufs_stats->last_hibern8_enter_time;
			ufs_stats->hibern8_exit_cnt++;
		}
		ufs_stats->hibern8_flag = false;
		pixel_update_power_event(hba, PE_H8_EXIT);
	}
}

static int pixel_ufs_init_cmd_log(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);

	memset(&ufs->cmd_log, 0, sizeof(struct pixel_cmd_log));

	ufs->cmd_log.entry = devm_kcalloc(ufs->dev, MAX_CMD_ENTRY_NUM,
					  sizeof(struct pixel_cmd_log_entry),
					  GFP_KERNEL);

	ufs->enable_cmd_log = 0;

	return 0;
}

static struct pixel_cmd_log_entry *__get_log_entry(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	struct pixel_cmd_log_entry *entry = NULL;

	if (!virt_addr_valid(ufs->cmd_log.entry))
		return entry;

	ufs->cmd_log.seq_cnt++;
	ufs->cmd_log.head = (ufs->cmd_log.head + 1) % MAX_CMD_ENTRY_NUM;
	entry = &ufs->cmd_log.entry[ufs->cmd_log.head];
	memset(entry, 0, sizeof(struct pixel_cmd_log_entry));
	entry->seq_num = ufs->cmd_log.seq_cnt;

	return entry;
}

static void __set_cmd_log_str(struct ufs_hba *hba, enum pixel_event_type event,
			      u8 opcode, struct pixel_cmd_log_entry *entry)
{
	enum pixel_command_type cmd_type = CMD_UNDEF;

	entry->event = ufs_event_str[event];
	switch (event) {
	case EVENT_DME_SEND:
	case EVENT_DME_COMPL:
		switch (opcode) {
		case 0x01:
			cmd_type = CMD_DME_GET; break;
		case 0x02:
			cmd_type = CMD_DME_SET; break;
		case 0x10:
			cmd_type = CMD_DME_PWR_ON; break;
		case 0x11:
			cmd_type = CMD_DME_PWR_OFF; break;
		case 0x14:
			cmd_type = CMD_DME_RESET; break;
		case 0x16:
			cmd_type = CMD_DME_LINKSTARTUP; break;
		case 0x17:
			cmd_type = CMD_DME_H8_ENTER; break;
		case 0x18:
			cmd_type = CMD_DME_H8_EXIT; break;
		}
		break;
	case EVENT_SCSI_SEND:
	case EVENT_SCSI_COMPL:
		switch (opcode) {
		case 0x1b:
			cmd_type = CMD_SCSI_SSU; break;
		case 0x28:
			cmd_type = CMD_SCSI_READ_10; break;
		case 0x2a:
			cmd_type = CMD_SCSI_WRITE_10; break;
		case 0x88:
			cmd_type = CMD_SCSI_READ_16; break;
		case 0x8a:
			cmd_type = CMD_SCSI_WRITE_16; break;
		case 0x35:
			cmd_type = CMD_SCSI_SYNC; break;
		case 0x42:
			cmd_type = CMD_SCSI_UNMAP; break;
		case 0xa2:
			cmd_type = CMD_SCSI_PROTOCOL_IN; break;
		case 0xb5:
			cmd_type = CMD_SCSI_PROTOCOL_OUT; break;
		case 0x95:
			cmd_type = CMD_SCSI_ZBC_IN; break;
		case 0x94:
			cmd_type = CMD_SCSI_ZBC_OUT; break;
		}
		break;
	case EVENT_NOP_OUT:
	case EVENT_NOP_IN:
	case EVENT_QUERY_SEND:
	case EVENT_QUERY_COMPL:
	case EVENT_TM_SEND:
	case EVENT_TM_ERR:
	case EVENT_TM_COMPL:
	case EVENT_INTR_FATAL_ERR:
	case EVENT_INTR_UIC_ERR:
	case EVENT_INTR_H8_ERR:
	case EVENT_UNDEF:
		break;
	}

	if (cmd_type > 0 && cmd_type < ARRAY_SIZE(ufs_cmd_str))
		entry->cmd = ufs_cmd_str[cmd_type];
	else
		entry->cmd = 0;
}

static void __store_cmd_log(struct ufs_hba *hba, enum pixel_event_type event,
		u8 lun, u8 opcode, u8 idn, sector_t sector, int affected_bytes,
		u8 group_id, int tag, u64 error, u8 queue_eh_work)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	struct pixel_cmd_log_entry *entry;

	if (!ufs->enable_cmd_log)
		return;

	entry = __get_log_entry(hba);
	if (!entry || event >= ARRAY_SIZE(ufs_event_str))
		return;

	__set_cmd_log_str(hba, event, opcode, entry);
	entry->lun = lun;
	entry->opcode = opcode;
	entry->idn = idn;
	entry->sector = sector;
	entry->affected_bytes = affected_bytes;
	entry->outstanding_reqs = hba->outstanding_reqs;
	entry->tag = tag;
	entry->group_id = group_id;
	entry->error = error;
	entry->queue_eh_work = queue_eh_work;
}

static void pixel_ufs_trace_fdeviceinit(struct ufs_hba *hba,
					struct ufshcd_lrb *lrbp, u8 opcode)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	struct latency_metrics *metric = NULL;

	if (opcode == UPIU_QUERY_OPCODE_SET_FLAG)
		metric = &ufs->power_stats.fdevinit_set;
	else if (opcode == UPIU_QUERY_OPCODE_READ_FLAG)
		metric = &ufs->power_stats.fdevinit_read;

	if (metric) {
		s64 delta = ktime_to_us(ktime_sub(lrbp->compl_time_stamp,
						  lrbp->issue_time_stamp));

		WARN_ON(delta < 0);

		metric->count++;
		metric->time_spent_us += delta;
		if (delta > metric->max_latency_us)
			metric->max_latency_us = delta;
	}
}

static void pixel_ufs_trace_upiu_cmd(struct ufs_hba *hba,
		struct ufshcd_lrb *lrbp, bool is_start)
{
	u8 lun = 0, opcode = 0, idn = 0, tag = 0, group_id = 0;
	enum pixel_event_type event = EVENT_UNDEF;
	sector_t sector = 0;
	int affected_bytes = 0;

	lun = lrbp->lun;
	tag = lrbp->task_tag;

	if (lrbp->cmd) {
		event = (is_start) ? EVENT_SCSI_SEND : EVENT_SCSI_COMPL;
		opcode = lrbp->cmd->cmnd[0];
		sector = blk_rq_pos(scsi_cmd_to_rq(lrbp->cmd));

		affected_bytes = blk_rq_bytes(scsi_cmd_to_rq(lrbp->cmd));
		if (opcode == WRITE_10 || opcode == READ_10)
			group_id = lrbp->cmd->cmnd[6] & 0x3f;
		else if (opcode == WRITE_16 || opcode == READ_16)
			group_id = lrbp->cmd->cmnd[14] & 0x3f;
	} else {
		if (hba->dev_cmd.type == DEV_CMD_TYPE_NOP)
			event = (is_start) ? EVENT_NOP_OUT : EVENT_NOP_IN;
		else if (hba->dev_cmd.type == DEV_CMD_TYPE_QUERY) {
			event = (is_start) ? EVENT_QUERY_SEND : EVENT_QUERY_COMPL;
			opcode = hba->dev_cmd.query.request.upiu_req.opcode;
			idn = hba->dev_cmd.query.request.upiu_req.idn;
			if (idn == QUERY_FLAG_IDN_FDEVICEINIT &&
			    event == EVENT_QUERY_COMPL)
				pixel_ufs_trace_fdeviceinit(hba, lrbp, opcode);
		}
	}

	__store_cmd_log(hba, event, lun, opcode, idn, sector,
		affected_bytes, group_id, tag, 0, 0);
}

static void pixel_ufs_send_uic_command(void *data, struct ufs_hba *hba,
				const struct uic_command *ucmd, int str_t)
{
	enum pixel_event_type event = EVENT_UNDEF;
	u8 opcode = (ucmd) ? ucmd->command : 0xFF;

	if (str_t == UFS_CMD_SEND)
		event = EVENT_DME_SEND;
	else if (str_t == UFS_CMD_COMP)
		event = EVENT_DME_COMPL;
	else
		event = EVENT_UNDEF;

	__store_cmd_log(hba, event, 0, opcode, 0, 0, 0, 0, 0, 0, 0);
}

static void pixel_ufs_send_tm_command(void *data, struct ufs_hba *hba,
				int tag, int str_t)
{
	enum pixel_event_type event = EVENT_UNDEF;

	if (str_t == UFS_TM_SEND)
		event = EVENT_TM_SEND;
	else if (str_t == UFS_TM_ERR)
		event = EVENT_TM_ERR;
	else if (str_t == UFS_TM_COMP)
		event = EVENT_TM_COMPL;
	else
		event = EVENT_UNDEF;

	__store_cmd_log(hba, event, 0, 0, 0, 0, 0, 0, tag,
			event == EVENT_TM_ERR, 0);
}

static void pixel_ufs_check_int_errors(void *data, struct ufs_hba *hba,
				bool queue_eh_work)
{
	enum pixel_event_type event = EVENT_UNDEF;
	u32 status = 0;

	if (!queue_eh_work)
		return;

	if (hba->errors & INT_FATAL_ERRORS)
		event = EVENT_INTR_FATAL_ERR;
	else if (hba->errors & UIC_ERROR) {
		event = EVENT_INTR_UIC_ERR;
		status = hba->uic_error;
	} else if (hba->errors & UFSHCD_UIC_HIBERN8_MASK) {
		event = EVENT_INTR_H8_ERR;
	}

	__store_cmd_log(hba, event, 0, 0, 0, 0, 0, 0, 0, hba->errors,
			queue_eh_work);
}

static void pixel_ufs_send_command(void *data, struct ufs_hba *hba,
					struct ufshcd_lrb *lrbp)
{
	pixel_ufs_update_io_stats(hba, lrbp, true);
	pixel_ufs_trace_upiu_cmd(hba, lrbp, true);
}

static inline int ufshcd_get_tr_ocs(struct ufshcd_lrb *lrbp)
{
	return le32_to_cpu(lrbp->utr_descriptor_ptr->header.dword_2) & MASK_OCS;
}

static inline int ufshcd_get_req_rsp(struct utp_upiu_rsp *ucd_rsp_ptr)
{
	return be32_to_cpu(ucd_rsp_ptr->header.dword_0) >> 24;
}

static inline int ufshcd_get_rsp_upiu_result(struct utp_upiu_rsp *ucd_rsp_ptr)
{
	return be32_to_cpu(ucd_rsp_ptr->header.dword_1) & MASK_RSP_UPIU_RESULT;
}

static void pixel_ufs_compl_command(void *data, struct ufs_hba *hba,
					struct ufshcd_lrb *lrbp)
{
	pixel_ufs_update_io_stats(hba, lrbp, false);
	pixel_ufs_update_req_stats(hba, lrbp);
	pixel_ufs_trace_upiu_cmd(hba, lrbp, false);
}

static void pixel_ufs_prepare_command(void *data, struct ufs_hba *hba,
			struct request *rq, struct ufshcd_lrb *lrbp, int *err)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	u8 opcode;

	*err = 0;

	if (ufs->set_gid == WB_GID_DISABLE)
		return;

	/*
	 * Set the group number to 0x11, if set_gid is,
	 *  1: WB_GID_SEL for REQ_META and REQ_IDLE requests,
	 *  2: WB_GID_ALL for all the requests.
	 */
	if (ufs->set_gid == WB_GID_SEL &&
	   !(rq->cmd_flags & (REQ_META | REQ_IDLE)))
		return;

	/* Do not set the group number for zoned logical units. */
	if (blk_queue_is_zoned(rq->q))
		return;

	/* Only set the group number for UFS versions 3.1 and later. */
	if (hba->dev_info.wspecversion <= 0x300)
		return;

	opcode = (u8)(*lrbp->cmd->cmnd);
	if (opcode == WRITE_10)
		lrbp->cmd->cmnd[6] = 0x11;
	else if (opcode == WRITE_16)
		lrbp->cmd->cmnd[14] = 0x11;
	ufs->need_wb_flush = true;
}

static ssize_t life_time_estimation_c_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	u8 value;
	int ret;

	if (ufshcd_eh_in_progress(hba))
		return -EBUSY;

	pm_runtime_get_sync(hba->dev);
	ret = ufshcd_read_desc_param(hba, QUERY_DESC_IDN_HEALTH, 0,
			HEALTH_DESC_PARAM_LIFE_TIME_EST_C, &value, 1);
	pm_runtime_put_sync(hba->dev);
	if (ret)
		return -EINVAL;

	return sprintf(buf, "0x%02X\n", value);
}

static DEVICE_ATTR_RO(life_time_estimation_c);

static struct attribute *ufs_sysfs_health_descriptor[] = {
	&dev_attr_life_time_estimation_c.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_health_descriptor_group = {
	.name = "health_descriptor",
	.attrs = ufs_sysfs_health_descriptor,
};

static ssize_t vendor_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%.8s\n", hba->ufs_device_wlun->vendor);
}

static ssize_t model_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%.16s\n", hba->ufs_device_wlun->model);
}

static ssize_t rev_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%.4s\n", hba->ufs_device_wlun->rev);
}

static ssize_t platform_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", hba->ufs_version);
}

/* for manual gc */
static ssize_t manual_gc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	u32 status = MANUAL_GC_OFF;

	if (ufs->manual_gc.state == MANUAL_GC_DISABLE)
		return scnprintf(buf, PAGE_SIZE, "%s", "disabled\n");

	if (ufs->manual_gc.hagc_support) {
		int err;

		if (!ufshcd_eh_in_progress(hba)) {
			pm_runtime_get_sync(hba->dev);
			err = ufshcd_query_attr_retry(hba,
				UPIU_QUERY_OPCODE_READ_ATTR,
				QUERY_ATTR_IDN_MANUAL_GC_STATUS, 0, 0, &status);
			pm_runtime_put_sync(hba->dev);
			ufs->manual_gc.hagc_support = err ? false: true;
		}
	}

	if (!ufs->manual_gc.hagc_support)
		return scnprintf(buf, PAGE_SIZE, "%s", "bkops\n");
	return scnprintf(buf, PAGE_SIZE, "%s",
			status == MANUAL_GC_OFF ? "off\n" : "on\n");
}

static int manual_gc_enable(struct ufs_hba *hba, u32 *value)
{
	int ret;

	if (ufshcd_eh_in_progress(hba))
		return -EBUSY;

	pm_runtime_get_sync(hba->dev);
	ret = ufshcd_query_attr_retry(hba,
				UPIU_QUERY_OPCODE_WRITE_ATTR,
				QUERY_ATTR_IDN_MANUAL_GC_CONT, 0, 0,
				value);
	pm_runtime_put_sync(hba->dev);
	return ret;
}

static ssize_t manual_gc_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	u32 value;
	int err = 0;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	if (value >= MANUAL_GC_MAX)
		return -EINVAL;

	if (ufshcd_eh_in_progress(hba))
		return -EBUSY;

	if (value == MANUAL_GC_DISABLE || value == MANUAL_GC_ENABLE) {
		ufs->manual_gc.state = value;
		return count;
	}
	if (ufs->manual_gc.state == MANUAL_GC_DISABLE)
		return count;

	if (ufs->manual_gc.hagc_support)
		ufs->manual_gc.hagc_support =
			manual_gc_enable(hba, &value) ? false : true;

	pm_runtime_get_sync(hba->dev);

	/* flush wb buffer */
	if (hba->dev_info.wspecversion >= 0x0310) {
		enum query_opcode opcode = (value == MANUAL_GC_ON) ?
						UPIU_QUERY_OPCODE_SET_FLAG:
						UPIU_QUERY_OPCODE_CLEAR_FLAG;
		u8 index = ufshcd_wb_get_query_index(hba);

		if (ufs->need_wb_flush) {
			ufshcd_query_flag_retry(hba, opcode,
				QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
				index, NULL);
			ufshcd_query_flag_retry(hba, opcode,
				QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN, index, NULL);
		}
	}

	if (!ufs->manual_gc.hagc_support) {
		err = ufshcd_bkops_ctrl(hba, (value == MANUAL_GC_ON) ?
					BKOPS_STATUS_NON_CRITICAL:
					BKOPS_STATUS_CRITICAL);
		if (!hba->auto_bkops_enabled)
			err = -EAGAIN;
	}

	if (err || hrtimer_active(&ufs->manual_gc.hrtimer)) {
		pm_runtime_put_sync(hba->dev);
		return count;
	} else {
		/* pm_runtime_put_sync in delay_ms */
		hrtimer_start(&ufs->manual_gc.hrtimer,
			ms_to_ktime(ufs->manual_gc.delay_ms),
			HRTIMER_MODE_REL);
	}
	return count;
}

static ssize_t manual_gc_hold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);

	return snprintf(buf, PAGE_SIZE, "%lu\n", ufs->manual_gc.delay_ms);
}

static ssize_t manual_gc_hold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	ufs->manual_gc.delay_ms = value;
	return count;
}

static enum hrtimer_restart pixel_mgc_hrtimer_handler(struct hrtimer *timer)
{
	struct pixel_ufs *ufs = container_of(timer, struct pixel_ufs,
					     manual_gc.hrtimer);

	queue_work(ufs->manual_gc.mgc_workq, &ufs->manual_gc.hibern8_work);
	return HRTIMER_NORESTART;
}

static void pixel_mgc_hibern8_work(struct work_struct *work)
{
	struct pixel_ufs *ufs = container_of(work, struct pixel_ufs,
					     manual_gc.hibern8_work);
	pm_runtime_put_sync(ufs->hba->dev);
	/* bkops will be disabled when power down */
}

void pixel_init_manual_gc(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	struct ufs_manual_gc *mgc = &ufs->manual_gc;
	char wq_name[sizeof("ufs_mgc_hibern8_work_#####")];

	mgc->state = MANUAL_GC_ENABLE;
	mgc->hagc_support = true;
	mgc->delay_ms = UFSHCD_MANUAL_GC_HOLD_HIBERN8;

	hrtimer_init(&mgc->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mgc->hrtimer.function = pixel_mgc_hrtimer_handler;

	INIT_WORK(&mgc->hibern8_work, pixel_mgc_hibern8_work);
	snprintf(wq_name, ARRAY_SIZE(wq_name), "ufs_mgc_hibern8_work_%d",
			hba->host->host_no);
	ufs->manual_gc.mgc_workq = create_singlethread_workqueue(wq_name);
}

static ssize_t host_capabilities_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", hba->caps);
}

static ssize_t slowio_store(struct device *dev, struct device_attribute *_attr,
		const char *buf, size_t count)
{
	struct slowio_attr *attr = (struct slowio_attr *)_attr;
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	unsigned long flags, value;

	if (kstrtol(buf, 0, &value))
		return -EINVAL;

	if (attr->systype == PIXEL_SLOWIO_CNT)
		value = 0;
	else if (value < PIXEL_MIN_SLOWIO_US)
		return -EINVAL;

	spin_lock_irqsave(hba->host->host_lock, flags);
	ufs->slowio[attr->optype][attr->systype] = value;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	if (attr->systype == PIXEL_SLOWIO_US)
		pixel_ufs_update_slowio_min_us(hba);
	return count;
}

static ssize_t slowio_show(struct device *dev, struct device_attribute *_attr,
		char *buf)
{
	struct slowio_attr *attr = (struct slowio_attr *)_attr;
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	return snprintf(buf, PAGE_SIZE, "%lld\n",
			ufs->slowio[attr->optype][attr->systype]);
}

#define __SLOWIO_ATTR(_name)					\
	__ATTR(slowio_##_name, 0644, slowio_show, slowio_store)

#define SLOWIO_ATTR_RW(_name, _optype)				\
static struct slowio_attr ufs_slowio_##_name##_us = {		\
	.attr = __SLOWIO_ATTR(_name##_us),			\
	.optype = _optype,					\
	.systype = PIXEL_SLOWIO_US,				\
};								\
								\
static struct slowio_attr ufs_slowio_##_name##_cnt = {		\
	.attr = __SLOWIO_ATTR(_name##_cnt),			\
	.optype = _optype,					\
	.systype = PIXEL_SLOWIO_CNT,				\
}

static const char *ufschd_ufs_dev_pwr_mode_to_string(
			enum ufs_dev_pwr_mode state)
{
	switch (state) {
	case UFS_ACTIVE_PWR_MODE:	return "ACTIVE";
	case UFS_SLEEP_PWR_MODE:	return "SLEEP";
	case UFS_POWERDOWN_PWR_MODE:	return "POWERDOWN";
	default:			return "UNKNOWN";
	}
}

static ssize_t curr_dev_pwr_mode_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", ufschd_ufs_dev_pwr_mode_to_string(
				hba->curr_dev_pwr_mode));
}

static const char *ufschd_uic_link_state_to_string(
			enum uic_link_state state)
{
	switch (state) {
	case UIC_LINK_OFF_STATE:	return "OFF";
	case UIC_LINK_ACTIVE_STATE:	return "ACTIVE";
	case UIC_LINK_HIBERN8_STATE:	return "HIBERN8";
	default:			return "UNKNOWN";
	}
}

static ssize_t uic_link_state_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", ufschd_uic_link_state_to_string(
				hba->uic_link_state));
}

static const char * const wb_gid_str[] = {
	[WB_GID_DISABLE]	= "disable",
	[WB_GID_SEL]		= "selective",
	[WB_GID_ALL]		= "all",
};

static ssize_t set_gid_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);

	return snprintf(buf, PAGE_SIZE, "%s\n", wb_gid_str[ufs->set_gid]);
}

static ssize_t set_gid_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	enum query_opcode opcode;
	int i;
	u8 index;

	i = sysfs_match_string(wb_gid_str, buf);
	if (i < 0)
		return i;

	if (i == ufs->set_gid)
		return count;

	/* 0: disable, 1: enable selectively, 2: always */
	opcode = i != WB_GID_DISABLE ? UPIU_QUERY_OPCODE_SET_FLAG :
			   UPIU_QUERY_OPCODE_CLEAR_FLAG;
	index = ufshcd_wb_get_query_index(hba);
	ufshcd_query_flag_retry(hba, opcode,
				QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
				index, NULL);
	ufs->set_gid = i;
	return count;
}

static DEVICE_ATTR_RO(vendor);
static DEVICE_ATTR_RO(model);
static DEVICE_ATTR_RO(rev);
static DEVICE_ATTR_RO(platform_version);
static DEVICE_ATTR_RW(manual_gc);
static DEVICE_ATTR_RW(manual_gc_hold);
static DEVICE_ATTR_RO(host_capabilities);
static DEVICE_ATTR_RO(curr_dev_pwr_mode);
static DEVICE_ATTR_RO(uic_link_state);
static DEVICE_ATTR_RW(set_gid);
SLOWIO_ATTR_RW(read, PIXEL_SLOWIO_READ);
SLOWIO_ATTR_RW(write, PIXEL_SLOWIO_WRITE);
SLOWIO_ATTR_RW(unmap, PIXEL_SLOWIO_UNMAP);
SLOWIO_ATTR_RW(sync, PIXEL_SLOWIO_SYNC);

#if IS_ENABLED(CONFIG_SCSI_UFS_PIXEL_FIPS140)
#define PIXEL_FIPS_STATS_ATTR(_name)	\
static ssize_t _name##_show(struct device *dev,				\
	struct device_attribute *attr, char *buf)			\
{									\
	struct ufs_hba *hba = dev_get_drvdata(dev);			\
	const struct ufs_pixel_fips_info *fips_info;			\
									\
	pm_runtime_get_sync(hba->dev);					\
	fips_info = ufs_pixel_fips_get_info(hba);			\
	pm_runtime_put_sync(hba->dev);					\
									\
	return sprintf(buf, "%u\n", fips_info->_name);			\
}									\
static DEVICE_ATTR_RO(_name)

PIXEL_FIPS_STATS_ATTR(hmac_self_test_attempted);
PIXEL_FIPS_STATS_ATTR(hmac_self_test_passed);
PIXEL_FIPS_STATS_ATTR(self_integrity_test_attempted);
PIXEL_FIPS_STATS_ATTR(self_integrity_test_passed);
PIXEL_FIPS_STATS_ATTR(encryption_test_attempted);
PIXEL_FIPS_STATS_ATTR(encryption_test_passed);
PIXEL_FIPS_STATS_ATTR(decryption_test_attempted);
PIXEL_FIPS_STATS_ATTR(decryption_test_passed);

static ssize_t ise_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	const struct ufs_pixel_fips_info *fips_info;

	pm_runtime_get_sync(hba->dev);
	fips_info = ufs_pixel_fips_get_info(hba);
	pm_runtime_put_sync(hba->dev);

	return sprintf(buf, "%u.%u.%u\n",
		       fips_info->ise_version_major,
		       fips_info->ise_version_minor,
		       fips_info->ise_version_revision);
}
static DEVICE_ATTR_RO(ise_version);

static ssize_t key_delivery_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	const struct ufs_pixel_fips_info *fips_info;

	pm_runtime_get_sync(hba->dev);
	fips_info = ufs_pixel_fips_get_info(hba);
	pm_runtime_put_sync(hba->dev);

	return sprintf(buf, "%s\n",
		       fips_info->key_delivery_mode == KEY_DELIVERY_SW ?
		       "software" : "hardware");
}
static DEVICE_ATTR_RO(key_delivery_mode);

static struct attribute *pixel_sysfs_fips[] = {
	&dev_attr_hmac_self_test_attempted.attr,
	&dev_attr_hmac_self_test_passed.attr,
	&dev_attr_self_integrity_test_attempted.attr,
	&dev_attr_self_integrity_test_passed.attr,
	&dev_attr_encryption_test_attempted.attr,
	&dev_attr_encryption_test_passed.attr,
	&dev_attr_decryption_test_attempted.attr,
	&dev_attr_decryption_test_passed.attr,
	&dev_attr_ise_version.attr,
	&dev_attr_key_delivery_mode.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_fips_group = {
	.name = "fips",
	.attrs = pixel_sysfs_fips,
};
#endif

static struct attribute *pixel_sysfs_ufshcd_attrs[] = {
	&dev_attr_vendor.attr,
	&dev_attr_model.attr,
	&dev_attr_rev.attr,
	&dev_attr_platform_version.attr,
	&dev_attr_manual_gc.attr,
	&dev_attr_manual_gc_hold.attr,
	&dev_attr_host_capabilities.attr,
	&dev_attr_curr_dev_pwr_mode.attr,
	&dev_attr_uic_link_state.attr,
	&dev_attr_set_gid.attr,
	&ufs_slowio_read_us.attr.attr,
	&ufs_slowio_read_cnt.attr.attr,
	&ufs_slowio_write_us.attr.attr,
	&ufs_slowio_write_cnt.attr.attr,
	&ufs_slowio_unmap_us.attr.attr,
	&ufs_slowio_unmap_cnt.attr.attr,
	&ufs_slowio_sync_us.attr.attr,
	&ufs_slowio_sync_cnt.attr.attr,
	NULL
};

static const struct attribute_group pixel_sysfs_default_group = {
	.attrs = pixel_sysfs_ufshcd_attrs,
};

#define PIXEL_ATTRIBUTE_RW(_name, _uname)				\
static ssize_t _name##_show(struct device *dev,				\
	struct device_attribute *attr, char *buf)			\
{									\
	struct ufs_hba *hba = dev_get_drvdata(dev);			\
	u32 value;							\
	int err;							\
	pm_runtime_get_sync(hba->dev);					\
	err = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR,	\
		QUERY_ATTR_IDN##_uname, 0, 0, &value);			\
	pm_runtime_put_sync(hba->dev);					\
	if (err)							\
		return err;						\
	return snprintf(buf, PAGE_SIZE, "0x%08X\n", value);		\
}									\
static ssize_t _name##_store(struct device *dev,			\
	struct device_attribute *attr, const char *buf, size_t count)	\
{									\
	struct ufs_hba *hba = dev_get_drvdata(dev);			\
	u32 value;							\
	int err;							\
	if (kstrtouint(buf, 0, &value))					\
		return -EINVAL;						\
	pm_runtime_get_sync(hba->dev);					\
	err = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_WRITE_ATTR,\
		QUERY_ATTR_IDN##_uname, 0, 0, &value);			\
	pm_runtime_put_sync(hba->dev);					\
	if (err)							\
		return err;						\
	return count;							\
}									\
static DEVICE_ATTR_RW(_name)

PIXEL_ATTRIBUTE_RW(boot_lun_enabled, _BOOT_LU_EN);

static ssize_t print_cmd_log_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	u32 value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;
	if (value == 1)
		pixel_print_cmd_log(hba);
	return count;
}
static DEVICE_ATTR_WO(print_cmd_log);

static ssize_t power_event_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	return sprintf(buf, "%u\n", ufs->power_event_mode);
}

static ssize_t power_event_mode_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	u32 value;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	if (value != ufs->power_event_mode) {
		ufs->power_event_mode = value;
		if (ufs->power_event_mode &&
		    !hba->ufs_device_wlun->sdev_gendev.power.is_suspended)
			pixel_update_power_event(hba, PE_SYSTEM_RESUME);
	}
	return count;
}
static DEVICE_ATTR_RW(power_event_mode);

static ssize_t enable_pixel_ufs_logging_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);

	return sysfs_emit(buf, "%u\n", ufs->enable_cmd_log);
}

static ssize_t enable_pixel_ufs_logging_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	u32 value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;
	if (value != ufs->enable_cmd_log)
		WRITE_ONCE(ufs->enable_cmd_log, value);
	return count;
}
static DEVICE_ATTR_RW(enable_pixel_ufs_logging);

static struct attribute *pixel_sysfs_pixel_attrs[] = {
	&dev_attr_boot_lun_enabled.attr,
	&dev_attr_print_cmd_log.attr,
	&dev_attr_power_event_mode.attr,
	&dev_attr_enable_pixel_ufs_logging.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_group = {
	.name = "pixel",
	.attrs = pixel_sysfs_pixel_attrs,
};

#define PIXEL_REQ_STATS_ATTR(_name, _type_name, _type_show)		\
static ssize_t _name##_show(struct device *dev,				\
	struct device_attribute *attr, char *buf)			\
{									\
	struct ufs_hba *hba = dev_get_drvdata(dev);			\
	struct pixel_ufs *ufs = to_pixel_ufs(hba);			\
	unsigned long flags;						\
	u64 val;							\
	spin_lock_irqsave(hba->host->host_lock, flags);			\
	switch (_type_show) {						\
	case REQ_SYSFS_MIN:						\
		val = ufs->req_stats[_type_name].req_min;		\
		break;							\
	case REQ_SYSFS_MAX:						\
		val = ufs->req_stats[_type_name].req_max;		\
		break;							\
	case REQ_SYSFS_AVG:						\
		val = div64_u64(ufs->req_stats[_type_name].req_sum,	\
				ufs->req_stats[_type_name].req_count);	\
		break;							\
	case REQ_SYSFS_SUM:						\
		val = ufs->req_stats[_type_name].req_sum;		\
		break;							\
	default:							\
		val = 0;						\
		break;							\
	}								\
	spin_unlock_irqrestore(hba->host->host_lock, flags);		\
	return sprintf(buf, "%llu\n", val);				\
}									\
static DEVICE_ATTR_RO(_name)

static inline void pixel_init_req_stats(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);

	memset(ufs->req_stats, 0, sizeof(ufs->req_stats));
	memset(pixel_ufs_prev_sum, 0, sizeof(pixel_ufs_prev_sum));
	memset(pixel_ufs_prev_count, 0, sizeof(pixel_ufs_prev_count));
}

static ssize_t reset_req_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t reset_req_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	unsigned long value;

	if (kstrtoul(buf, 0, &value)) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(hba->host->host_lock, flags);
	pixel_init_req_stats(hba);
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return count;
}

PIXEL_REQ_STATS_ATTR(all_min, REQ_TYPE_VALID, REQ_SYSFS_MIN);
PIXEL_REQ_STATS_ATTR(all_max, REQ_TYPE_VALID, REQ_SYSFS_MAX);
PIXEL_REQ_STATS_ATTR(all_avg, REQ_TYPE_VALID, REQ_SYSFS_AVG);
PIXEL_REQ_STATS_ATTR(all_sum, REQ_TYPE_VALID, REQ_SYSFS_SUM);
PIXEL_REQ_STATS_ATTR(read_min, REQ_TYPE_READ, REQ_SYSFS_MIN);
PIXEL_REQ_STATS_ATTR(read_max, REQ_TYPE_READ, REQ_SYSFS_MAX);
PIXEL_REQ_STATS_ATTR(read_avg, REQ_TYPE_READ, REQ_SYSFS_AVG);
PIXEL_REQ_STATS_ATTR(read_sum, REQ_TYPE_READ, REQ_SYSFS_SUM);
PIXEL_REQ_STATS_ATTR(write_min, REQ_TYPE_WRITE, REQ_SYSFS_MIN);
PIXEL_REQ_STATS_ATTR(write_max, REQ_TYPE_WRITE, REQ_SYSFS_MAX);
PIXEL_REQ_STATS_ATTR(write_avg, REQ_TYPE_WRITE, REQ_SYSFS_AVG);
PIXEL_REQ_STATS_ATTR(write_sum, REQ_TYPE_WRITE, REQ_SYSFS_SUM);
PIXEL_REQ_STATS_ATTR(flush_min, REQ_TYPE_FLUSH, REQ_SYSFS_MIN);
PIXEL_REQ_STATS_ATTR(flush_max, REQ_TYPE_FLUSH, REQ_SYSFS_MAX);
PIXEL_REQ_STATS_ATTR(flush_avg, REQ_TYPE_FLUSH, REQ_SYSFS_AVG);
PIXEL_REQ_STATS_ATTR(flush_sum, REQ_TYPE_FLUSH, REQ_SYSFS_SUM);
PIXEL_REQ_STATS_ATTR(discard_min, REQ_TYPE_DISCARD, REQ_SYSFS_MIN);
PIXEL_REQ_STATS_ATTR(discard_max, REQ_TYPE_DISCARD, REQ_SYSFS_MAX);
PIXEL_REQ_STATS_ATTR(discard_avg, REQ_TYPE_DISCARD, REQ_SYSFS_AVG);
PIXEL_REQ_STATS_ATTR(discard_sum, REQ_TYPE_DISCARD, REQ_SYSFS_SUM);
PIXEL_REQ_STATS_ATTR(security_min, REQ_TYPE_SECURITY, REQ_SYSFS_MIN);
PIXEL_REQ_STATS_ATTR(security_max, REQ_TYPE_SECURITY, REQ_SYSFS_MAX);
PIXEL_REQ_STATS_ATTR(security_avg, REQ_TYPE_SECURITY, REQ_SYSFS_AVG);
PIXEL_REQ_STATS_ATTR(security_sum, REQ_TYPE_SECURITY, REQ_SYSFS_SUM);
DEVICE_ATTR_RW(reset_req_status);

static struct attribute *ufs_sysfs_req_stats[] = {
	&dev_attr_all_min.attr,
	&dev_attr_all_max.attr,
	&dev_attr_all_avg.attr,
	&dev_attr_all_sum.attr,
	&dev_attr_read_min.attr,
	&dev_attr_read_max.attr,
	&dev_attr_read_avg.attr,
	&dev_attr_read_sum.attr,
	&dev_attr_write_min.attr,
	&dev_attr_write_max.attr,
	&dev_attr_write_avg.attr,
	&dev_attr_write_sum.attr,
	&dev_attr_flush_min.attr,
	&dev_attr_flush_max.attr,
	&dev_attr_flush_avg.attr,
	&dev_attr_flush_sum.attr,
	&dev_attr_discard_min.attr,
	&dev_attr_discard_max.attr,
	&dev_attr_discard_avg.attr,
	&dev_attr_discard_sum.attr,
	&dev_attr_security_min.attr,
	&dev_attr_security_max.attr,
	&dev_attr_security_avg.attr,
	&dev_attr_security_sum.attr,
	&dev_attr_reset_req_status.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_req_stats_group = {
	.name = "req_stats",
	.attrs = ufs_sysfs_req_stats,
};

#define PIXEL_IO_STATS_ATTR(_name, _type_show)			\
static ssize_t _name##_show(struct device *dev,			\
	struct device_attribute *attr, char *buf)		\
{								\
	struct ufs_hba *hba = dev_get_drvdata(dev);		\
	struct pixel_ufs *ufs = to_pixel_ufs(hba);		\
	unsigned long flags;					\
	u64 val;						\
	spin_lock_irqsave(hba->host->host_lock, flags);		\
	val = ufs->curr_io_stats._type_show;		\
	spin_unlock_irqrestore(hba->host->host_lock, flags);	\
	return sprintf(buf, "%llu\n", val);			\
}								\
static DEVICE_ATTR_RO(_name)


void pixel_init_io_stats(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	int cpu;

	memset(&ufs->curr_io_stats, 0, sizeof(ufs->curr_io_stats));
	memset(&ufs->prev_io_stats, 0, sizeof(ufs->prev_io_stats));

	ufs->io_stats = alloc_percpu(struct pixel_io_stats);
	if (!ufs->io_stats) {
		dev_err(hba->dev, "%s: failed on io_stats alloc_percpu()\n",
			__func__);
		return;
	}

	for_each_possible_cpu(cpu)
		memset(per_cpu_ptr(ufs->io_stats, cpu), 0,
			sizeof(struct pixel_io_stats));
}

static ssize_t reset_io_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t reset_io_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	int cpu;
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	if (ufs->io_stats)
		for_each_possible_cpu(cpu)
			memset(per_cpu_ptr(ufs->io_stats, cpu), 0,
				sizeof(struct pixel_io_stats));
	memset(&ufs->curr_io_stats, 0, sizeof(ufs->curr_io_stats));
	memset(&ufs->prev_io_stats, 0, sizeof(ufs->prev_io_stats));
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return count;
}

PIXEL_IO_STATS_ATTR(rcnt_start, r_req_count_started);
PIXEL_IO_STATS_ATTR(rcnt_complete, r_req_count_completed);
PIXEL_IO_STATS_ATTR(rcnt_maxdiff, r_max_diff_req_count);
PIXEL_IO_STATS_ATTR(rbyte_start, r_total_bytes_started);
PIXEL_IO_STATS_ATTR(rbyte_complete, r_total_bytes_completed);
PIXEL_IO_STATS_ATTR(rbyte_maxdiff, r_max_diff_total_bytes);
PIXEL_IO_STATS_ATTR(wcnt_start, w_req_count_started);
PIXEL_IO_STATS_ATTR(wcnt_complete, w_req_count_completed);
PIXEL_IO_STATS_ATTR(wcnt_maxdiff, w_max_diff_req_count);
PIXEL_IO_STATS_ATTR(wbyte_start, w_total_bytes_started);
PIXEL_IO_STATS_ATTR(wbyte_complete, w_total_bytes_completed);
PIXEL_IO_STATS_ATTR(wbyte_maxdiff, w_max_diff_total_bytes);
PIXEL_IO_STATS_ATTR(rwcnt_start, rw_req_count_started);
PIXEL_IO_STATS_ATTR(rwcnt_complete, rw_req_count_completed);
PIXEL_IO_STATS_ATTR(rwcnt_maxdiff, rw_max_diff_req_count);
PIXEL_IO_STATS_ATTR(rwbyte_start, rw_total_bytes_started);
PIXEL_IO_STATS_ATTR(rwbyte_complete, rw_total_bytes_completed);
PIXEL_IO_STATS_ATTR(rwbyte_maxdiff, rw_max_diff_total_bytes);
DEVICE_ATTR_RW(reset_io_status);

static struct attribute *ufs_sysfs_io_stats[] = {
	&dev_attr_rcnt_start.attr,
	&dev_attr_rcnt_complete.attr,
	&dev_attr_rcnt_maxdiff.attr,
	&dev_attr_rbyte_start.attr,
	&dev_attr_rbyte_complete.attr,
	&dev_attr_rbyte_maxdiff.attr,
	&dev_attr_wcnt_start.attr,
	&dev_attr_wcnt_complete.attr,
	&dev_attr_wcnt_maxdiff.attr,
	&dev_attr_wbyte_start.attr,
	&dev_attr_wbyte_complete.attr,
	&dev_attr_wbyte_maxdiff.attr,
	&dev_attr_rwcnt_start.attr,
	&dev_attr_rwcnt_complete.attr,
	&dev_attr_rwcnt_maxdiff.attr,
	&dev_attr_rwbyte_start.attr,
	&dev_attr_rwbyte_complete.attr,
	&dev_attr_rwbyte_maxdiff.attr,
	&dev_attr_reset_io_status.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_io_stats_group = {
	.name = "io_stats",
	.attrs = ufs_sysfs_io_stats,
};

#define PIXEL_ERR_STATS_ATTR(_name, _err_name, _type)			\
static ssize_t _name##_show(struct device *dev,				\
		struct device_attribute *attr, char *buf)		\
{									\
	struct ufs_hba *hba = dev_get_drvdata(dev);			\
	struct ufs_event_hist *e = &hba->ufs_stats.event[_err_name];	\
	unsigned long flags;						\
	u64 val = 0;							\
	int p;								\
	spin_lock_irqsave(hba->host->host_lock, flags);			\
	switch (_type) {						\
	case PIXEL_ERR_COUNT:						\
		val = e->cnt;						\
		break;							\
	case PIXEL_ERR_TIME:						\
		p = (e->pos + UFS_EVENT_HIST_LENGTH - 1) %		\
				UFS_EVENT_HIST_LENGTH;			\
		val = ktime_to_us(e->tstamp[p]);			\
		break;							\
	}								\
	spin_unlock_irqrestore(hba->host->host_lock, flags);		\
	return snprintf(buf, PAGE_SIZE, "%llu\n", val);			\
}									\
static DEVICE_ATTR_RO(_name)

static ssize_t reset_err_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t reset_err_status_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	memset(hba->ufs_stats.event, 0,
			sizeof(struct ufs_event_hist) * UFS_EVT_CNT);
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return count;
}

PIXEL_ERR_STATS_ATTR(pa_err_count, UFS_EVT_PA_ERR, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(pa_err_lasttime, UFS_EVT_PA_ERR, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(dl_err_count, UFS_EVT_DL_ERR, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(dl_err_lasttime, UFS_EVT_DL_ERR, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(nl_err_count, UFS_EVT_NL_ERR, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(nl_err_lasttime, UFS_EVT_NL_ERR, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(tl_err_count, UFS_EVT_TL_ERR, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(tl_err_lasttime, UFS_EVT_TL_ERR, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(dme_err_count, UFS_EVT_DME_ERR, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(dme_err_lasttime, UFS_EVT_DME_ERR, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(auto_hibern8_err_count, UFS_EVT_AUTO_HIBERN8_ERR,
		PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(auto_hibern8_err_lasttime, UFS_EVT_AUTO_HIBERN8_ERR,
		PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(fatal_err_count, UFS_EVT_FATAL_ERR, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(fatal_err_lasttime, UFS_EVT_FATAL_ERR, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(link_startup_err_count, UFS_EVT_LINK_STARTUP_FAIL,
		PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(link_startup_err_lasttime, UFS_EVT_LINK_STARTUP_FAIL,
		PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(resume_err_count, UFS_EVT_RESUME_ERR, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(resume_err_lasttime, UFS_EVT_RESUME_ERR, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(suspend_err_count, UFS_EVT_SUSPEND_ERR, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(suspend_err_lasttime, UFS_EVT_SUSPEND_ERR, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(dev_reset_count, UFS_EVT_DEV_RESET, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(dev_reset_lasttime, UFS_EVT_DEV_RESET, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(host_reset_count, UFS_EVT_HOST_RESET, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(host_reset_lasttime, UFS_EVT_HOST_RESET, PIXEL_ERR_TIME);
PIXEL_ERR_STATS_ATTR(task_abort_count, UFS_EVT_ABORT, PIXEL_ERR_COUNT);
PIXEL_ERR_STATS_ATTR(task_abort_lasttime, UFS_EVT_ABORT, PIXEL_ERR_TIME);
DEVICE_ATTR_RW(reset_err_status);

static struct attribute *ufs_sysfs_err_stats[] = {
	&dev_attr_pa_err_count.attr,
	&dev_attr_pa_err_lasttime.attr,
	&dev_attr_dl_err_count.attr,
	&dev_attr_dl_err_lasttime.attr,
	&dev_attr_nl_err_count.attr,
	&dev_attr_nl_err_lasttime.attr,
	&dev_attr_tl_err_count.attr,
	&dev_attr_tl_err_lasttime.attr,
	&dev_attr_dme_err_count.attr,
	&dev_attr_dme_err_lasttime.attr,
	&dev_attr_auto_hibern8_err_count.attr,
	&dev_attr_auto_hibern8_err_lasttime.attr,
	&dev_attr_fatal_err_count.attr,
	&dev_attr_fatal_err_lasttime.attr,
	&dev_attr_link_startup_err_count.attr,
	&dev_attr_link_startup_err_lasttime.attr,
	&dev_attr_resume_err_count.attr,
	&dev_attr_resume_err_lasttime.attr,
	&dev_attr_suspend_err_count.attr,
	&dev_attr_suspend_err_lasttime.attr,
	&dev_attr_dev_reset_count.attr,
	&dev_attr_dev_reset_lasttime.attr,
	&dev_attr_host_reset_count.attr,
	&dev_attr_host_reset_lasttime.attr,
	&dev_attr_task_abort_count.attr,
	&dev_attr_task_abort_lasttime.attr,
	&dev_attr_reset_err_status.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_err_stats_group = {
	.name = "err_stats",
	.attrs = ufs_sysfs_err_stats,
};

#define PIXEL_UFS_STATS_ATTR(_name)				\
static ssize_t _name##_show(struct device *dev,			\
	struct device_attribute *attr, char *buf)		\
{								\
	struct ufs_hba *hba = dev_get_drvdata(dev);		\
	struct pixel_ufs *ufs = to_pixel_ufs(hba);		\
	unsigned long flags;					\
	u64 val;						\
	spin_lock_irqsave(hba->host->host_lock, flags);		\
	val = ufs->ufs_stats._name;				\
	spin_unlock_irqrestore(hba->host->host_lock, flags);	\
	return sprintf(buf, "%llu\n", val);			\
}								\
static DEVICE_ATTR_RO(_name)

PIXEL_UFS_STATS_ATTR(hibern8_total_us);
PIXEL_UFS_STATS_ATTR(hibern8_exit_cnt);
PIXEL_UFS_STATS_ATTR(last_hibern8_enter_time);
PIXEL_UFS_STATS_ATTR(last_hibern8_exit_time);

static struct attribute *ufs_sysfs_ufs_stats[] = {
	&dev_attr_hibern8_total_us.attr,
	&dev_attr_hibern8_exit_cnt.attr,
	&dev_attr_last_hibern8_enter_time.attr,
	&dev_attr_last_hibern8_exit_time.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_ufs_stats_group = {
	.name = "ufs_stats",
	.attrs = ufs_sysfs_ufs_stats,
};

#define PIXEL_HC_REG_ATTR(_name, _uname)			\
static ssize_t _name##_show(struct device *dev,			\
	struct device_attribute *attr, char *buf)		\
{			\
	struct ufs_hba *hba = dev_get_drvdata(dev);			\
	u32 value;						\
	pm_runtime_get_sync(hba->dev);			\
	ufshcd_hold(hba, false);			\
	value = ufshcd_readl(hba, REG##_uname);			\
	ufshcd_release(hba);			\
	pm_runtime_put(hba->dev);			\
	return sprintf(buf, "0x%08X\n", value);			\
}			\
static DEVICE_ATTR_RO(_name)

PIXEL_HC_REG_ATTR(cap, _CONTROLLER_CAPABILITIES);
PIXEL_HC_REG_ATTR(ver, _UFS_VERSION);
PIXEL_HC_REG_ATTR(hcpid, _CONTROLLER_DEV_ID);
PIXEL_HC_REG_ATTR(hcmid, _CONTROLLER_PROD_ID);
PIXEL_HC_REG_ATTR(ahit, _AUTO_HIBERNATE_IDLE_TIMER);
PIXEL_HC_REG_ATTR(is, _INTERRUPT_STATUS);
PIXEL_HC_REG_ATTR(ie, _INTERRUPT_ENABLE);
PIXEL_HC_REG_ATTR(hcs, _CONTROLLER_STATUS);
PIXEL_HC_REG_ATTR(hce, _CONTROLLER_ENABLE);
PIXEL_HC_REG_ATTR(uecpa, _UIC_ERROR_CODE_PHY_ADAPTER_LAYER);
PIXEL_HC_REG_ATTR(uecdl, _UIC_ERROR_CODE_DATA_LINK_LAYER);
PIXEL_HC_REG_ATTR(uecn, _UIC_ERROR_CODE_NETWORK_LAYER);
PIXEL_HC_REG_ATTR(uect, _UIC_ERROR_CODE_TRANSPORT_LAYER);
PIXEL_HC_REG_ATTR(uecdme, _UIC_ERROR_CODE_DME);

static struct attribute *pixel_sysfs_hc_reg_ifc[] = {
	&dev_attr_cap.attr,
	&dev_attr_ver.attr,
	&dev_attr_hcpid.attr,
	&dev_attr_hcmid.attr,
	&dev_attr_ahit.attr,
	&dev_attr_is.attr,
	&dev_attr_ie.attr,
	&dev_attr_hcs.attr,
	&dev_attr_hce.attr,
	&dev_attr_uecpa.attr,
	&dev_attr_uecdl.attr,
	&dev_attr_uecn.attr,
	&dev_attr_uect.attr,
	&dev_attr_uecdme.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_hc_register_ifc_group = {
	.name = "hc_register_ifc",
	.attrs = pixel_sysfs_hc_reg_ifc,
};

#define PIXEL_POWER_INFO_ATTR(_name)			\
static ssize_t _name##_show(struct device *dev,			\
	struct device_attribute *attr, char *buf)	\
{			\
	struct ufs_hba *hba = dev_get_drvdata(dev);			\
	return sprintf(buf, "%u\n", hba->pwr_info._name);	\
}			\
static DEVICE_ATTR_RO(_name)

PIXEL_POWER_INFO_ATTR(gear_rx);
PIXEL_POWER_INFO_ATTR(gear_tx);
PIXEL_POWER_INFO_ATTR(lane_rx);
PIXEL_POWER_INFO_ATTR(lane_tx);
PIXEL_POWER_INFO_ATTR(pwr_rx);
PIXEL_POWER_INFO_ATTR(pwr_tx);
PIXEL_POWER_INFO_ATTR(hs_rate);

static struct attribute *pixel_sysfs_power_info[] = {
	&dev_attr_gear_rx.attr,
	&dev_attr_gear_tx.attr,
	&dev_attr_lane_rx.attr,
	&dev_attr_lane_tx.attr,
	&dev_attr_pwr_rx.attr,
	&dev_attr_pwr_tx.attr,
	&dev_attr_hs_rate.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_power_info_group = {
	.name = "power_info",
	.attrs = pixel_sysfs_power_info,
};

#define PIXEL_POWER_STATS_ATTR(_name, _metric, _var)			\
static ssize_t _name##_show(struct device *dev,				\
	struct device_attribute *attr, char *buf)			\
{									\
	struct ufs_hba *hba = dev_get_drvdata(dev);			\
	struct pixel_ufs *ufs = to_pixel_ufs(hba);			\
	return sprintf(buf, "%llu\n", ufs->power_stats._metric._var);	\
}									\
static DEVICE_ATTR_RO(_name)

PIXEL_POWER_STATS_ATTR(fdevinit_set_count, fdevinit_set, count);
PIXEL_POWER_STATS_ATTR(fdevinit_set_time_spent_us, fdevinit_set, time_spent_us);
PIXEL_POWER_STATS_ATTR(fdevinit_set_max_latency_us, fdevinit_set,
		       max_latency_us);
PIXEL_POWER_STATS_ATTR(fdevinit_read_count, fdevinit_read, count);
PIXEL_POWER_STATS_ATTR(fdevinit_read_time_spent_us, fdevinit_read,
		       time_spent_us);
PIXEL_POWER_STATS_ATTR(fdevinit_read_max_latency_us, fdevinit_read,
		       max_latency_us);

static struct attribute *pixel_sysfs_power_stats[] = {
	&dev_attr_fdevinit_set_count.attr,
	&dev_attr_fdevinit_set_time_spent_us.attr,
	&dev_attr_fdevinit_set_max_latency_us.attr,
	&dev_attr_fdevinit_read_count.attr,
	&dev_attr_fdevinit_read_time_spent_us.attr,
	&dev_attr_fdevinit_read_max_latency_us.attr,
	NULL,
};

static const struct attribute_group pixel_sysfs_power_stats_group = {
	.name = "power_stats",
	.attrs = pixel_sysfs_power_stats,
};

static const struct attribute_group *pixel_ufs_sysfs_groups[] = {
	&pixel_sysfs_group,
	&pixel_sysfs_req_stats_group,
	&pixel_sysfs_io_stats_group,
	&pixel_sysfs_err_stats_group,
	&pixel_sysfs_ufs_stats_group,
	&pixel_sysfs_hc_register_ifc_group,
	&pixel_sysfs_power_info_group,
	&pixel_sysfs_power_stats_group,
#if IS_ENABLED(CONFIG_SCSI_UFS_PIXEL_FIPS140)
	&pixel_sysfs_fips_group,
#endif
	NULL,
};

static void pixel_ufs_update_sysfs_work(struct work_struct *work)
{
	struct pixel_ufs *ufs = container_of(work, struct pixel_ufs,
					     update_sysfs_work);
	struct ufs_hba *hba = ufs->hba;
	int err;

	err = sysfs_create_groups(&hba->dev->kobj, pixel_ufs_sysfs_groups);
	if (err)
		dev_err(hba->dev,
			"%s: sysfs groups creation failed (err = %d)\n",
			__func__, err);

	err = sysfs_update_group(&hba->dev->kobj,
				&pixel_sysfs_health_descriptor_group);
	if (err)
		dev_err(hba->dev, "%s: Failed to add a pixel group\n",
				__func__);

	err = sysfs_update_group(&hba->dev->kobj,
				&pixel_sysfs_default_group);
	if (err)
		dev_err(hba->dev, "%s: Failed to add a pixel group\n",
				__func__);
}

static void pixel_ufs_update_sysfs(void *data, struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);

	queue_work(system_highpri_wq, &ufs->update_sysfs_work);
}

static void pixel_ufs_update_sdev(void *data, struct scsi_device *sdev)
{
	/* do not use slow FUA */
	sdev->broken_fua = 1;
}

void pixel_print_cmd_log(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	struct pixel_cmd_log_entry *entry = NULL;
	int i;

	if (!ufs->enable_cmd_log)
		return;

	for (i = 1; i <= MAX_CMD_ENTRY_NUM; i++) {
		entry = &ufs->cmd_log.entry[(ufs->cmd_log.head + i) %
						MAX_CMD_ENTRY_NUM];
		if (!entry->seq_num)
			break;
		dev_err(hba->dev, "%u: %s tag: %d cmd: %s sector: %llu len: 0x%x outstanding: 0x%llx GID: 0x%x\n",
			entry->seq_num, entry->event,
			entry->tag, entry->cmd,
			entry->sector, entry->affected_bytes,
			entry->outstanding_reqs, entry->group_id);
	}
}

/* Initialize struct pixel_ufs except the crypto_ops, hba and dev members. */
static void pixel_ufs_init(struct pixel_ufs *ufs)
{
	memset(&ufs->ufs_stats, 0, sizeof(struct pixel_ufs_stats));
	memset(&ufs->power_stats, 0, sizeof(struct pixel_power_stats));
	ufs->ufs_stats.hibern8_flag = false;
	ufs->set_gid = WB_GID_SEL;

	/* init power event monitoring */
	spin_lock_init(&ufs->power_event_lock);
}

int pixel_init(struct ufs_hba *hba, struct device *pdev,
	       const struct pixel_crypto_ops *crypto_ops)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	int ret;

	ufs->crypto_ops = crypto_ops;
	ufs->hba = hba;
	ufs->dev = pdev;

	ret = pixel_ufs_crypto_init(hba);
	if (ret)
		return ret;

	pixel_ufs_init(ufs);

	ret = register_trace_android_vh_ufs_prepare_command(
				pixel_ufs_prepare_command, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_ufs_update_sysfs(
				pixel_ufs_update_sysfs, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_ufs_send_command(
				pixel_ufs_send_command, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_ufs_compl_command(
				pixel_ufs_compl_command, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_ufs_send_uic_command(
				pixel_ufs_send_uic_command, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_ufs_send_tm_command(
				pixel_ufs_send_tm_command, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_ufs_check_int_errors(
				pixel_ufs_check_int_errors, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_ufs_update_sdev(
				pixel_ufs_update_sdev, NULL);
	if (ret)
		return ret;

	pixel_ufs_init_cmd_log(hba);

	INIT_WORK(&ufs->update_sysfs_work, pixel_ufs_update_sysfs_work);

	pixel_init_manual_gc(hba);

	pixel_init_slowio(hba);

	pixel_init_io_stats(hba);

	return 0;
}

void pixel_exit(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);

	devm_kfree(ufs->dev, ufs->cmd_log.entry);
}
