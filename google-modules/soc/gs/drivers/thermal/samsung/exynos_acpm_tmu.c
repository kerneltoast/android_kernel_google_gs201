// SPDX-License-Identifier: GPL-2.0-only
/*
 * exynos_acpm_tmu.c - ACPM TMU plugin interface
 *
 * Copyright (C) 2017 Samsung Electronics
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/debugfs.h>
#include <linux/sched/clock.h>
#include <soc/google/acpm_ipc_ctrl.h>
#include <trace/events/power.h>
#include "exynos_acpm_tmu.h"

static unsigned int acpm_tmu_ch_num, acpm_tmu_size;

static bool acpm_tmu_test_mode;
static bool acpm_tmu_log;

bool exynos_acpm_tmu_is_test_mode(void)
{
	return acpm_tmu_test_mode;
}

void exynos_acpm_tmu_set_test_mode(bool mode)
{
	acpm_tmu_test_mode = mode;
}

void exynos_acpm_tmu_enable_log(bool mode)
{
	acpm_tmu_log = mode;
}

bool exynos_acpm_tmu_is_log_enabled(void)
{
	return acpm_tmu_log;
}

#define acpm_ipc_latency_check() \
	do { \
		if (acpm_tmu_log) { \
			pr_info_ratelimited("[acpm_tmu] type 0x%02x latency %llu ns ret %d\n", \
					message->req.type, latency, ret); \
		} \
	} while (0)

#define acpm_ipc_err_check() \
	do { \
		if (ret < 0) { \
			pr_warn("[acpm_tmu] IPC error! type 0x%02x latency %llu ns ret %d\n", \
					message->req.type, latency, ret); \
		} \
	} while (0)

static void exynos_acpm_tmu_ipc_send_data(union tmu_ipc_message *message)
{
	struct ipc_config config;
	int ret;
	unsigned long long before, after, latency;

	config.cmd = message->data;
	config.response = true;

	before = sched_clock();
	ret = acpm_ipc_send_data(acpm_tmu_ch_num, &config);
	after = sched_clock();
	latency = after - before;

	acpm_ipc_err_check();
	acpm_ipc_latency_check();

	memcpy(message->data, config.cmd, sizeof(message->data));
}

/*
 * TMU_IPC_INIT
 */
int exynos_acpm_tmu_set_init(struct acpm_tmu_cap *cap)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_INIT;

	exynos_acpm_tmu_ipc_send_data(&message);

	if (message.resp.ret & CAP_APM_IRQ)
		cap->acpm_irq = true;

	if (message.resp.ret & CAP_APM_DIVIDER)
		cap->acpm_divider = true;

	return 0;
}

/*
 * TMU_IPC_READ_TEMP
 *
 * - tz: thermal zone index registered in device tree
 */
int exynos_acpm_tmu_set_read_temp(int tz, int *temp, int *stat)
{
	union tmu_ipc_message message;

	if (acpm_tmu_test_mode)
		return -1;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_READ_TEMP;
	message.req.tzid = tz;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		int i;
		u8 *temp = &message.resp.rsvd0;
		pr_info_ratelimited("[acpm_tmu] tz %d temp 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d\n",
			tz,
			message.resp.rsvd0,
			message.resp.rsvd1,
			message.resp.rsvd2,
			message.resp.rsvd3,
			message.resp.rsvd4,
			message.resp.rsvd5,
			message.resp.rsvd6);
		for (i = 0; i < 7; i++) {
			char name[40];

			scnprintf(name, sizeof(name), "TMU%d_%d", tz, i);
			trace_clock_set_rate(name, temp[i], raw_smp_processor_id());
		}
	}
	*temp = message.resp.temp;
	*stat = message.resp.stat;

	return 0;
}

/*
 * TMU_IPC_AP_SUSPEND
 */
int exynos_acpm_tmu_set_suspend(int flag)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_AP_SUSPEND;
	message.req.rsvd = flag;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}

	return 0;
}

/*
 * TMU_IPC_CP_CALL
 */
int exynos_acpm_tmu_set_cp_call(void)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_CP_CALL;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}

	return 0;
}

/*
 * TMU_IPC_AP_RESUME
 */
int exynos_acpm_tmu_set_resume(void)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_AP_RESUME;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}

	pr_info_ratelimited("%s: acpm irq %d cold cnt %d stat %d\n",
		__func__, message.resp.rsvd2, message.resp.rsvd0, message.resp.stat);

	return 0;
}

int exynos_acpm_tmu_ipc_dump(int no, unsigned int dump[])
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_READ_TEMP;
	message.req.tzid = no;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu_dump] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}

	if (dump) {
		dump[0] = message.data[2];
		dump[1] = message.data[3];
	}

	return 0;
}

void exynos_acpm_tmu_set_threshold(int tz, unsigned char temp[])
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_THRESHOLD;
	message.req.tzid = tz;
	message.req.req_rsvd0 = temp[0];
	message.req.req_rsvd1 = temp[1];
	message.req.req_rsvd2 = temp[2];
	message.req.req_rsvd3 = temp[3];
	message.req.req_rsvd4 = temp[4];
	message.req.req_rsvd5 = temp[5];
	message.req.req_rsvd6 = temp[6];
	message.req.req_rsvd7 = temp[7];

	exynos_acpm_tmu_ipc_send_data(&message);
	pr_info_ratelimited("[acpm_tmu] tz %d threshold: 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d\n",
		tz, temp[0], temp[1], temp[2], temp[3], temp[4], temp[5],
		temp[6], temp[7]);
}

void exynos_acpm_tmu_set_hysteresis(int tz, unsigned char hyst[])
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_HYSTERESIS;
	message.req.tzid = tz;
	message.req.req_rsvd0 = hyst[0];
	message.req.req_rsvd1 = hyst[1];
	message.req.req_rsvd2 = hyst[2];
	message.req.req_rsvd3 = hyst[3];
	message.req.req_rsvd4 = hyst[4];
	message.req.req_rsvd5 = hyst[5];
	message.req.req_rsvd6 = hyst[6];
	message.req.req_rsvd7 = hyst[7];

	exynos_acpm_tmu_ipc_send_data(&message);
	pr_info_ratelimited("[acpm_tmu] tz %d hysteresis: 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d\n",
		tz, hyst[0], hyst[1], hyst[2], hyst[3], hyst[4], hyst[5],
		hyst[6], hyst[7]);
}

void exynos_acpm_tmu_set_interrupt_enable(int tz, unsigned char inten)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));
	message.req.type = TMU_IPC_INTEN;
	message.req.tzid = tz;
	message.req.req_rsvd0 = inten;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}
}

void exynos_acpm_tmu_tz_control(int tz, bool enable)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));
	message.req.type = TMU_IPC_TMU_CONTROL;
	message.req.tzid = tz;
	message.req.req_rsvd0 = ((enable) ? 1 : 0);

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}
}

int exynos_acpm_tmu_tz_trip_control(int tz, bool enable)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));
	message.req.type = TMU_IPC_TMU_TRIP_CONTROL;
	message.req.tzid = tz;
	message.req.req_rsvd0 = ((enable) ? 1 : 0);

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}
	return message.resp.ret;
}

void exynos_acpm_tmu_clear_tz_irq(int tz)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));
	message.req.type = TMU_IPC_IRQ_CLEAR;
	message.req.tzid = tz;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}
}

void exynos_acpm_tmu_set_emul_temp(int tz, unsigned char temp)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));
	message.req.type = TMU_IPC_EMUL_TEMP;
	message.req.tzid = tz;
	message.req.req_rsvd0 = temp;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}
}

void exynos_acpm_tmu_reg_read(u8 tmu_id, u16 offset, u32 *val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));
	message.req.type = TMU_IPC_REG_READ;
	message.req.rsvd = tmu_id;
	message.req.req_rsvd0 = (offset & TMU_REG_OFFSET_MASK_1);
	message.req.req_rsvd1 = ((offset & TMU_REG_OFFSET_MASK_2) >> TMU_REG_OFFSET_SHIFT_8);

	exynos_acpm_tmu_ipc_send_data(&message);
	*val = (message.resp.rsvd0 |
		(message.resp.rsvd1 << TMU_REG_VAL_SHIFT_8) |
		(message.resp.rsvd2 << TMU_REG_VAL_SHIFT_16) |
		(message.resp.rsvd3 << TMU_REG_VAL_SHIFT_24));

	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu_reg_read] tmu_id:0x%02x tmu_reg_offset:0x%04x tmu_reg_val:0x%08x\n",
			message.req.rsvd,
			(message.req.req_rsvd0 | (message.req.req_rsvd1 << TMU_REG_OFFSET_SHIFT_8)),
			*val);
	}
}

void exynos_acpm_tmu_reg_write(u8 tmu_id, u16 offset, u32 val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));
	message.req.type = TMU_IPC_REG_WRITE;
	message.req.rsvd = tmu_id;
	message.req.req_rsvd0 = (offset & TMU_REG_OFFSET_MASK_1);
	message.req.req_rsvd1 = ((offset & TMU_REG_OFFSET_MASK_2) >> TMU_REG_OFFSET_SHIFT_8);
	message.req.req_rsvd2 = (val & TMU_REG_VAL_MASK_1);
	message.req.req_rsvd3 = ((val & TMU_REG_VAL_MASK_2) >> TMU_REG_VAL_SHIFT_8);
	message.req.req_rsvd4 = ((val & TMU_REG_VAL_MASK_3) >> TMU_REG_VAL_SHIFT_16);
	message.req.req_rsvd5 = ((val & TMU_REG_VAL_MASK_4) >> TMU_REG_VAL_SHIFT_24);

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info_ratelimited("[acpm_tmu_reg_write] tmu_id:0x%02x tmu_reg_offset:0x%04x tmu_reg_val:0x%08x\n",
			message.req.rsvd,
			(message.req.req_rsvd0 | (message.req.req_rsvd1 << TMU_REG_OFFSET_SHIFT_8)),
			(message.req.req_rsvd2 |
			 (message.req.req_rsvd3 << TMU_REG_VAL_SHIFT_8) |
			 (message.req.req_rsvd4 << TMU_REG_VAL_SHIFT_16) |
			 (message.req.req_rsvd5 << TMU_REG_VAL_SHIFT_24)));
	}
}

void exynos_acpm_tmu_ipc_get_target_freq(int tz, u32 *freq)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TARGET_FREQ;
	message.req.tzid = tz;

	exynos_acpm_tmu_ipc_send_data(&message);
	*freq = (message.resp.rsvd3 << 24) | (message.resp.rsvd2 << 16) |
		(message.resp.rsvd1 << 8) | (message.resp.rsvd0 << 0);
}

int exynos_acpm_tmu_ipc_set_gov_config(int tz, u64 qword)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_GOV_CONFIG;
	message.req.tzid = tz;
	message.data_64b[1] = qword;

	exynos_acpm_tmu_ipc_send_data(&message);
	return message.resp.ret;
}

void exynos_acpm_tmu_ipc_set_gov_debug_tracing_mode(int debug_mode)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_GOV_DEBUG_TRACING_MODE;
	message.req.req_rsvd0 = (u8)(debug_mode & 0xff);

	exynos_acpm_tmu_ipc_send_data(&message);
}

int exynos_acpm_tmu_ipc_set_gov_time_windows(int timer_interval, int thermal_press_window)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_GOV_DEBUG_TIMER_INTERVAL;
	message.req.req_rsvd0 = (u8)(timer_interval & 0xff);
	message.req.req_rsvd1 = (u8)(thermal_press_window & 0xff);
	message.req.req_rsvd2 = (u8)((thermal_press_window>>8) & 0xff);

	exynos_acpm_tmu_ipc_send_data(&message);
	return message.resp.ret;
}

int exynos_acpm_tmu_ipc_set_gov_tz_time_windows(int tz, int timer_interval,
						int thermal_press_window)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.tzid = tz;
	message.req.type = TMU_IPC_SET_GOV_TZ_TIMER_INTERVAL;
	message.req.req_rsvd0 = (u8)(timer_interval & 0xff);
	message.req.req_rsvd1 = (u8)(thermal_press_window & 0xff);
	message.req.req_rsvd2 = (u8)((thermal_press_window >> 8) & 0xff);

	exynos_acpm_tmu_ipc_send_data(&message);
	return message.resp.ret;
}

int exynos_acpm_tmu_ipc_get_gov_tz_time_windows(int tz, int *timer_interval,
						int *thermal_press_window)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.tzid = tz;
	message.req.type = TMU_IPC_GET_GOV_TZ_TIMER_INTERVAL;

	exynos_acpm_tmu_ipc_send_data(&message);

	*timer_interval = (int)message.resp.rsvd0;
	*thermal_press_window = (int)((u16)message.resp.rsvd2 << 8) | (u16)message.resp.rsvd1;

	return message.resp.ret;
}

void exynos_acpm_tmu_ipc_get_trip_counter(int tz, int trip_id, u64 *trip_counter)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TRIP_CNT;
	message.req.tzid = tz;
	message.req.req_rsvd0 = (u8)(trip_id & 0xff);

	exynos_acpm_tmu_ipc_send_data(&message);

	*trip_counter = message.data_64b[1];
}

void exynos_acpm_tmu_ipc_reset_trip_counter(int tz)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_RESET_TRIP_CNT;
	message.req.tzid = tz;

	exynos_acpm_tmu_ipc_send_data(&message);
}

int exynos_acpm_tmu_ipc_set_pi_param(int tz, u8 param, u32 val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_PI_PARAM;
	message.req.tzid = tz;
	message.req.rsvd = param;
	message.data[2] = val;

	exynos_acpm_tmu_ipc_send_data(&message);
	return (int)message.resp.ret;
}

void exynos_acpm_tmu_ipc_get_pi_param(int tz, u8 param, u32 *val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_PI_PARAM;
	message.req.tzid = tz;
	message.req.rsvd = param;

	exynos_acpm_tmu_ipc_send_data(&message);

	*val = message.data[2];
}

void exynos_acpm_tmu_ipc_set_table(int tz, u8 index, int val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_TABLE;
	message.req.tzid = tz;
	message.req.rsvd = index;
	message.data[2] = (u32)val;

	exynos_acpm_tmu_ipc_send_data(&message);
}

void exynos_acpm_tmu_ipc_get_table(int tz, u8 index, int *val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TABLE;
	message.req.tzid = tz;
	message.req.rsvd = index;

	exynos_acpm_tmu_ipc_send_data(&message);

	*val = (int)message.data[2];
}

void exynos_acpm_tmu_ipc_set_power_status(int tz, bool val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_POWER_STATUS;
	message.req.tzid = tz;
	message.req.rsvd = val;

	exynos_acpm_tmu_ipc_send_data(&message);
}

void exynos_acpm_tmu_ipc_set_control_temp_step(int tz, u32 val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_CONTROL_TEMP_STEP;
	message.req.tzid = tz;
	message.data[2] = val;

	exynos_acpm_tmu_ipc_send_data(&message);
}

int exynos_acpm_tmu_ipc_reset_tr_stats(int tz)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_RESET_TR_STATS;
	message.req.tzid = tz;

	exynos_acpm_tmu_ipc_send_data(&message);

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_set_tr_num_thresholds(int tz, int num_of_threshold)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_TR_NUM_THRESHOLD;
	message.req.tzid = tz;
	message.data[2] = num_of_threshold;

	exynos_acpm_tmu_ipc_send_data(&message);

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_get_tr_num_thresholds(int tz, int *num_of_threshold)
{
	union tmu_ipc_message message;

	if (!num_of_threshold)
		return -EINVAL;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TR_NUM_THRESHOLD;
	message.req.tzid = tz;

	exynos_acpm_tmu_ipc_send_data(&message);

	*num_of_threshold = (int)message.data[2];
	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_set_tr_thresholds(int tz, u8 qword_index, u64 val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_TR_THRESHOLD;
	message.req.tzid = tz;
	message.req.rsvd = qword_index;
	message.data_64b[1] = val;

	exynos_acpm_tmu_ipc_send_data(&message);

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_get_tr_thresholds(int tz, u8 qword_index, u64 *val)
{
	union tmu_ipc_message message;

	if (!val)
		return -EINVAL;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TR_THRESHOLD;
	message.req.tzid = tz;
	message.req.rsvd = qword_index;

	exynos_acpm_tmu_ipc_send_data(&message);

	*val = message.data_64b[1];

	return (int)message.resp.ret;
}

enum tr_stat_type {
	TIME_IN_STATES = 0,
	MAX = 1,
	MIN = 2,
	START = 3,
	END = 4,
};

int exynos_acpm_tmu_ipc_get_tr_stats(int tz, int bucket_idx, u64 *bucket_stats)
{
	union tmu_ipc_message message;

	if (!bucket_stats)
		return -EINVAL;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TR_STATS;
	message.req.tzid = (u8)tz;
	message.req.rsvd = (u8)bucket_idx;
	message.req.rsvd2 = TIME_IN_STATES;

	exynos_acpm_tmu_ipc_send_data(&message);

	*bucket_stats = message.data_64b[1];

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_get_tr_stats_start(int tz)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TR_STATS;
	message.req.tzid = (u8)tz;
	message.req.rsvd2 = START;

	exynos_acpm_tmu_ipc_send_data(&message);

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_get_tr_stats_end(int tz)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TR_STATS;
	message.req.tzid = (u8)tz;
	message.req.rsvd2 = END;

	exynos_acpm_tmu_ipc_send_data(&message);

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_get_tr_stats_max(int tz, int *temp, u64 *timestamp)
{
	union tmu_ipc_message message;

	if (!temp || !timestamp)
		return -EINVAL;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TR_STATS;
	message.req.tzid = (u8)tz;
	message.req.rsvd2 = MAX;

	exynos_acpm_tmu_ipc_send_data(&message);

	*temp = message.resp.temp;
	*timestamp = message.data_64b[1];

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_get_tr_stats_min(int tz, int *temp, u64 *timestamp)
{
	union tmu_ipc_message message;

	if (!temp || !timestamp)
		return -EINVAL;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TR_STATS;
	message.req.tzid = (u8)tz;
	message.req.rsvd2 = MIN;

	exynos_acpm_tmu_ipc_send_data(&message);

	*temp = message.resp.temp;
	*timestamp = message.data_64b[1];

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_set_temp_lut(int tz, int temp, int state, int append)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_TEMP_STATE_LUT;
	message.req.tzid = tz;
	message.req.rsvd = (u8)append;
	message.data[2] = temp;
	message.data[3] = state;

	exynos_acpm_tmu_ipc_send_data(&message);

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_get_temp_lut(int tz, u8 index, int *temp, int *state)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_GET_TEMP_STATE_LUT;
	message.req.tzid = tz;
	message.req.rsvd = index;

	exynos_acpm_tmu_ipc_send_data(&message);

	*temp = message.data[2];
	*state = message.data[3];

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_set_mpmm_clr_throttle_level(int tz, u16 val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_MPMM_CLR_THROTTLE_LEVEL;
	message.req.tzid = tz;
	message.data[2] = (u32)val;

	exynos_acpm_tmu_ipc_send_data(&message);

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_set_mpmm_throttle_level(int tz, u16 val)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_MPMM_THROTTLE_LEVEL;
	message.req.tzid = tz;
	message.data[2] = (u32)val;

	exynos_acpm_tmu_ipc_send_data(&message);

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_ipc_set_mpmm_enable(int tz, u8 enable)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_SET_MPMM_ENABLE;
	message.req.tzid = tz;
	message.req.rsvd  = enable;

	exynos_acpm_tmu_ipc_send_data(&message);

	return (int)message.resp.ret;
}

int exynos_acpm_tmu_init(void)
{
	struct device_node *np;

	np = of_find_node_by_name(NULL, "acpm_tmu");
	if (!np)
		return -ENODEV;

	return acpm_ipc_request_channel(np, NULL, &acpm_tmu_ch_num, &acpm_tmu_size);
}

int exynos_acpm_tmu_cb_init(struct acpm_irq_callback *cb)
{
	struct device_node *np;
	struct device_node *sub_node;
	if (cb == NULL)
		return -EINVAL;

	np = of_find_node_by_name(NULL, "acpm_tmu");
	if (!np) {
		pr_err("GOV: No acpm_tmu node available\n");
		return -ENODEV;
	}

	sub_node = of_find_node_by_name(np, "async");
	if (!sub_node) {
		pr_err("GOV: No asynchronous CPM to AP node available\n");
		return -ENODEV;
	}

	if (acpm_ipc_request_channel(sub_node, cb->fn, &cb->ipc_ch, &cb->ipc_ch_size)) {
		pr_err("GOV: No asynchronous CPM to AP interrupt channel available\n");
		return -ENODEV;
	}
	pr_info("GOV: Asynchronous notification enabled\n");

	return 0;
}
