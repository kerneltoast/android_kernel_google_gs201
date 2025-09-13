/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/ktime.h>
#include <linux/io.h>
#include <linux/sched/clock.h>

#include <soc/google/exynos-bcm_dbg.h>
#include <soc/google/exynos-bcm_dbg-dump.h>

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
static char file_name[128];
#endif

int exynos_bcm_dbg_print_accumulators(struct exynos_bcm_dbg_data *data,
	bool klog, char *out_buff, size_t *out_buff_len, loff_t off,
	size_t size)
{
	u32 row_cnt = data->bcm_ip_nr;
	struct exynos_bcm_dump_info *dump_info = NULL;
	struct exynos_bcm_accumulator_data *acc_data = NULL;
	u32 defined_event, ip_index;
	u64 count = 0, printed = 0;
	u64 pos, len;
	char buf[256];

	if (out_buff == NULL) {
		/* user doesn't want to copy the data */
		return 0;
	}

	if (!data->dump_addr.p_addr) {
		BCM_ERR("%s: No memory region for dump\n", __func__);
		return -ENOMEM;
	}

	if (in_interrupt()) {
		BCM_INFO("%s: skip dump in interrupt context\n", __func__);
		return 0;
	}

	dump_info = (void *)(data->dump_addr.v_addr + EXYNOS_BCM_KTIME_SIZE);

	len = scnprintf(buf, sizeof(buf),
			"seq_no, ip_index, define_event, time, ccnt, "
			"pmcnt0, pmcnt1, pmcnt2, pmcnt3, "
			"pmcnt4, pmcnt5, pmcnt6, pmcnt7\n");

	if (len > off) {
		pos = off;
		if (len - pos > size)
			len = size + pos;

		memcpy(out_buff + printed, buf + pos, len - pos);
		printed = len - pos;

		if (printed >= size)
			goto out;
	}
	count += len;

	while (row_cnt > 0) {
		defined_event = BCM_CMD_GET(dump_info->dump_header,
			BCM_EVT_PRE_DEFINE_MASK, BCM_DUMP_PRE_DEFINE_SHIFT);
		ip_index = BCM_CMD_GET(dump_info->dump_header, BCM_IP_MASK, 0);

		acc_data = (void *)(dump_info + 1);

		len = scnprintf(buf, sizeof(buf),
				"%u, %u, %u, %llu, %llu, %llu, %llu, %llu, "
				"%llu, %llu, %llu, %llu, %llu\n",
				dump_info->dump_seq_no, ip_index, defined_event,
				acc_data->measure_time, acc_data->ccnt,
				acc_data->pmcnt[0], acc_data->pmcnt[1],
				acc_data->pmcnt[2], acc_data->pmcnt[3],
				acc_data->pmcnt[4], acc_data->pmcnt[5],
				acc_data->pmcnt[6], acc_data->pmcnt[7]);

		if (count + len > off) {
			pos = (count > off) ? 0 : off - count;
			if (printed + len - pos > size)
				len = size - printed + pos;

			memcpy(out_buff + printed, buf + pos, len - pos);
			printed += len - pos;

			if (printed >= size)
				break;
		}
		count += len;

		dump_info = (void *)(acc_data + 1);
		row_cnt--;
	}

out:
	if (out_buff_len)
		*out_buff_len = printed;

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_bcm_dbg_print_accumulators);

int exynos_bcm_dbg_buffer_dump(struct exynos_bcm_dbg_data *data)
{
	void __iomem *v_addr = data->dump_addr.v_addr;
	u32 buff_size = data->dump_addr.buff_size - EXYNOS_BCM_KTIME_SIZE;
	u32 buff_cnt = 0;
	u32 dump_entry_size = sizeof(struct exynos_bcm_dump_info)
		+ sizeof(struct exynos_bcm_out_data);
	struct exynos_bcm_dump_info *dump_info = NULL;
	struct exynos_bcm_out_data *out_data = NULL;
	u32 defined_event, ip_index;
	char *result;
	ssize_t str_size;
	u32 tmp_ktime[2];
	u64 last_ktime;
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
	struct file *fp = NULL;
	mm_segment_t old_fs = get_fs();
#endif

	if (!data->dump_addr.p_addr) {
		BCM_ERR("%s: No memory region for dump\n", __func__);
		return -ENOMEM;
	}

	if (in_interrupt()) {
		BCM_INFO("%s: skip file dump in interrupt context\n", __func__);
		return 0;
	}

	result = kzalloc(sizeof(char) * BCM_DUMP_MAX_STR, GFP_KERNEL);
	if (result == NULL)
		return -ENOMEM;

	tmp_ktime[0] = __raw_readl(v_addr);
	tmp_ktime[1] = __raw_readl(v_addr + 0x4);
	last_ktime = (((u64)tmp_ktime[1] << EXYNOS_BCM_32BIT_SHIFT) &
			EXYNOS_BCM_U64_HIGH_MASK) |
			((u64)tmp_ktime[0] & EXYNOS_BCM_U64_LOW_MASK);

	dump_info = (struct exynos_bcm_dump_info *)(v_addr +
				EXYNOS_BCM_KTIME_SIZE);

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
	if (data->dump_file) {
		str_size = snprintf(file_name, sizeof(file_name),
				    "/data/result_bcm_%llu.csv",
				    cpu_clock(raw_smp_processor_id()));

		set_fs(KERNEL_DS);

		fp = filp_open(file_name, O_WRONLY|O_CREAT|O_APPEND, 0);
		if (IS_ERR(fp)) {
			BCM_ERR("%s: name: %s filp_open fail\n",
				__func__, file_name);
			set_fs(old_fs);
			kfree(result);
			return IS_ERR(fp);
		}

		str_size = snprintf(result, BCM_DUMP_MAX_STR,
				    "last kernel time, %llu\n", last_ktime);
		vfs_write(fp, result, str_size, &fp->f_pos);
	}
#endif

	str_size = snprintf(result, BCM_DUMP_MAX_STR,
	    "seq_no, ip_index, define_event, time, ccnt, pmcnt0, pmcnt1, pmcnt2, pmcnt3, pmcnt4, pmcnt5, pmcnt6, pmcnt7\n");

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
	if (data->dump_file)
		vfs_write(fp, result, str_size, &fp->f_pos);
#endif
	if (data->dump_klog)
		pr_info("%s", result);

	while ((buff_size - buff_cnt) > dump_entry_size) {
		defined_event = BCM_CMD_GET(dump_info->dump_header,
			BCM_EVT_PRE_DEFINE_MASK, BCM_DUMP_PRE_DEFINE_SHIFT);
		ip_index = BCM_CMD_GET(dump_info->dump_header, BCM_IP_MASK, 0);

		out_data = (struct exynos_bcm_out_data *)((u8 *)dump_info
				+ sizeof(struct exynos_bcm_dump_info));

		str_size = snprintf(result, BCM_DUMP_MAX_STR,
				"%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n",
				dump_info->dump_seq_no, ip_index, defined_event,
				out_data->measure_time, out_data->ccnt,
				out_data->pmcnt[0], out_data->pmcnt[1],
				out_data->pmcnt[2], out_data->pmcnt[3],
				out_data->pmcnt[4], out_data->pmcnt[5],
				out_data->pmcnt[6], out_data->pmcnt[7]);

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
		if (data->dump_file)
			vfs_write(fp, result, str_size, &fp->f_pos);
#endif
		if (data->dump_klog)
			pr_info("%s", result);

		dump_info = (struct exynos_bcm_dump_info *)((u8 *)dump_info
				+ dump_entry_size);
		buff_cnt += dump_entry_size;
	}

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
	if (data->dump_file) {
		filp_close(fp, NULL);
		set_fs(old_fs);
	}
#endif
	kfree(result);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_bcm_dbg_buffer_dump);

int exynos_bcm_dbg_dump(struct exynos_bcm_dbg_data *data, char *buff, size_t buff_size, loff_t off)
{
	void __iomem *ktime_ptr;
	struct exynos_bcm_dump_entry *dump_start, *dump_end, *dump_entry;
	u32 defined_event, ip_index;

	u64 last_ktime;
	char *line;
	size_t full_size, buffered_size, line_start_off;
	size_t line_size, line_done_size, line_todo_size;

	if (!data->dump_addr.p_addr) {
		BCM_ERR("%s: No memory region for dump\n", __func__);
		return -ENOMEM;
	}

	line = kzalloc(sizeof(char) * BCM_DUMP_MAX_LINE_SIZE, GFP_KERNEL);
	if (line == NULL)
		return -ENOMEM;

	ktime_ptr = data->dump_addr.v_addr;
	dump_start = (struct exynos_bcm_dump_entry *)(ktime_ptr + EXYNOS_BCM_KTIME_SIZE);
	dump_end = dump_start + data->dump_addr.buff_size / sizeof(struct exynos_bcm_dump_entry);

	full_size = 0;
	buffered_size = 0;

	/* Start with the meta data [time stamp] & heading */
	last_ktime = __raw_readq(ktime_ptr);

	line_size = scnprintf(line, BCM_DUMP_MAX_LINE_SIZE, "last kernel time, %llu\n"
			"seq_no, ip_index, define_event, time, ccnt, pmcnt0, pmcnt1, pmcnt2, "
			"pmcnt3, pmcnt4, pmcnt5, pmcnt6, pmcnt7\n", last_ktime);
	line_start_off = full_size;
	full_size += line_size;

	if (full_size > off) {
		line_done_size = max((int)(off - line_start_off), 0);
		line_todo_size = min(line_size - line_done_size, buff_size - buffered_size);
		memcpy(buff + buffered_size, line + line_done_size, line_todo_size);
		buffered_size += line_todo_size;
		if (buffered_size == buff_size)
			goto exit;
	}

	for (dump_entry = dump_start; dump_entry < dump_end; dump_entry++) {
		defined_event = BCM_CMD_GET(dump_entry->dump_info.dump_header,
			BCM_EVT_PRE_DEFINE_MASK, BCM_DUMP_PRE_DEFINE_SHIFT);
		ip_index = BCM_CMD_GET(dump_entry->dump_info.dump_header, BCM_IP_MASK, 0);

		line_size = scnprintf(line, BCM_DUMP_MAX_LINE_SIZE,
			"%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n",
			dump_entry->dump_info.dump_seq_no, ip_index, defined_event,
			dump_entry->dump_data.measure_time, dump_entry->dump_data.ccnt,
			dump_entry->dump_data.pmcnt[0], dump_entry->dump_data.pmcnt[1],
			dump_entry->dump_data.pmcnt[2], dump_entry->dump_data.pmcnt[3],
			dump_entry->dump_data.pmcnt[4], dump_entry->dump_data.pmcnt[5],
			dump_entry->dump_data.pmcnt[6], dump_entry->dump_data.pmcnt[7]);
		line_start_off = full_size;
		full_size += line_size;

		if (full_size > off) {
			line_done_size = max((int)(off - line_start_off), 0);
			line_todo_size = min(line_size - line_done_size, buff_size - buffered_size);
			memcpy(buff + buffered_size, line + line_done_size, line_todo_size);
			buffered_size += line_todo_size;
			if (buffered_size == buff_size)
				break;
		}
	}

exit:

	kfree(line);

	return buffered_size;
}
EXPORT_SYMBOL_GPL(exynos_bcm_dbg_dump);

MODULE_LICENSE("GPL");
