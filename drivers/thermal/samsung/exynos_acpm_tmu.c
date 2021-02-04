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

void exynos_acpm_tmu_log(bool mode)
{
	acpm_tmu_log = mode;
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

int exynos_acpm_tmu_init(void)
{
	struct device_node *np;

	np = of_find_node_by_name(NULL, "acpm_tmu");
	if (!np)
		return -ENODEV;

	return acpm_ipc_request_channel(np, NULL, &acpm_tmu_ch_num, &acpm_tmu_size);
}
