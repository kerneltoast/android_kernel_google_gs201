// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Google LLC
 *
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched/clock.h>
#include <linux/module.h>
#include <linux/workqueue.h>

#include "acpm.h"
#include "acpm_ipc.h"
#include "fw_header/framework.h"
#include "acpm_mbox_test.h"
#include <linux/delay.h>
#include <linux/random.h>

static int acpm_tmu_log;
static struct acpm_tmu_mbox_test *tmu_test;

#define CHIP_REV_TYPE_A0          0
#define CPU_NUM_NO_HERA           6
#define CPU_NUM_WITH_HERA         8
static unsigned int get_random_for_type(int type)
{
	unsigned int random;

	random = get_random_int();

	if (DELAY_MS == type) {
		return random % 100;
	} else if (TERMAL_ZONE_ID == type) {
		return random % TZ_END;
	} else if (CPU_ID == type) {
		if (CHIP_REV_TYPE_A0 == gs_chipid_get_type() &&
				CHIP_REV_TYPE_A0 == gs_chipid_get_revision()) {
			/* chipid: A0 */
			return random % CPU_NUM_NO_HERA;
		} else {
			/* chipid: A1/B0/... */
			return random % CPU_NUM_WITH_HERA;
		}
	}

	return 0;
}

#define acpm_ipc_latency_check() \
	do { \
		if (acpm_tmu_log) { \
			pr_info("[acpm_tmu] type 0x%02x latency %llu ns ret %d\n", \
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
	unsigned long long before, latency;

	config.cmd = message->data;
	config.response = true;

	before = sched_clock();
	ret = acpm_ipc_send_data(IPC_AP_TMU, &config);
	latency = sched_clock() - before;

	acpm_ipc_err_check();
	acpm_ipc_latency_check();

	memcpy(message->data, config.cmd, sizeof(message->data));
}

/*
 * TMU_IPC_READ_TEMP
 *
 * - tz: thermal zone index registered in device tree
 */
static int acpm_tmu_set_read_temp(int tz, int *temp, int *stat)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_READ_TEMP;
	message.req.tzid = tz;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}

	*temp = message.resp.temp;
	*stat = message.resp.stat;

	return 0;
}

/*
 * TMU_IPC_AP_SUSPEND
 */
static int acpm_tmu_set_suspend(int flag)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_AP_SUSPEND;
	message.req.rsvd = flag;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
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
static int acpm_tmu_set_resume(void)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));

	message.req.type = TMU_IPC_AP_RESUME;

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}

	pr_info("%s: acpm irq %d cold cnt %d stat %d\n",
		__func__, message.resp.rsvd2, message.resp.rsvd, message.resp.stat);

	return 0;
}

static void acpm_tmu_tz_control(int tz, bool enable)
{
	union tmu_ipc_message message;

	memset(&message, 0, sizeof(message));
	message.req.type = TMU_IPC_TMU_CONTROL;
	message.req.tzid = tz;
	message.req.req_rsvd0 = ((enable) ? 1 : 0);

	exynos_acpm_tmu_ipc_send_data(&message);
	if (acpm_tmu_log) {
		pr_info("[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			message.data[0],
			message.data[1],
			message.data[2],
			message.data[3]);
	}
}

static void acpm_debug_tmu_rd_tmp_random(struct work_struct *work)
{
	int temp, stat;
	u32 tzid;

	tzid = get_random_for_type(TERMAL_ZONE_ID);

	acpm_tmu_set_read_temp(tzid, &temp, &stat);
	pr_info("%s: thermal zone %d temp %d stat %d\n",
			__func__, tzid, temp, stat);
}

static void acpm_debug_tmu_rd_tmp_concur(struct work_struct *work)
{
	int temp, stat;
	u32 tzid;

	tzid = get_random_for_type(TERMAL_ZONE_ID);

	acpm_tmu_set_read_temp(tzid, &temp, &stat);
	pr_info("%s: thermal zone %d temp %d stat %d\n",
			__func__, tzid, temp, stat);
}

#define TMU_READ_TEMP_TRIGGER_DELAY      300
static void acpm_debug_tmu_rd_tmp_stress_trigger(struct work_struct *work)
{
	int i;

	for (i = 0; i < NUM_OF_WQ; i++) {
		queue_delayed_work_on(get_random_for_type(CPU_ID),
			tmu_test->rd_tmp_random_wq[i],
			&tmu_test->rd_tmp_random_wk[i],
			msecs_to_jiffies(get_random_for_type(DELAY_MS)));
		queue_delayed_work_on(get_random_for_type(CPU_ID),
			tmu_test->rd_tmp_concur_wq[i],
			&tmu_test->rd_tmp_concur_wk[i],
			msecs_to_jiffies(0));
	}
	queue_delayed_work_on(get_random_for_type(CPU_ID),
		tmu_test->rd_tmp_stress_trigger_wq,
		&tmu_test->rd_tmp_stress_trigger_wk,
		msecs_to_jiffies(TMU_READ_TEMP_TRIGGER_DELAY));
}

#define TMU_SUSPEND_RESUME_DELAY      100
static void acpm_debug_tmu_suspend(struct work_struct *work)
{
	u32 tzid;

	for (tzid = 0; tzid < TZ_END; tzid++)
		acpm_tmu_tz_control(tzid, false);

	acpm_tmu_set_suspend(false);
	pr_info("%s\n", __func__);

	queue_delayed_work_on(get_random_for_type(CPU_ID), tmu_test->resume_wq,
		&tmu_test->resume_work, msecs_to_jiffies(TMU_SUSPEND_RESUME_DELAY));
}
static void acpm_debug_tmu_resume(struct work_struct *work)
{
	int tzid, temp, stat;

	acpm_tmu_set_resume();

	for (tzid = 0; tzid < TZ_END; tzid++)
		acpm_tmu_tz_control(tzid, true);

	for (tzid = 0; tzid < TZ_END; tzid++) {
		acpm_tmu_set_read_temp(tzid, &temp, &stat);
		pr_info("%s: thermal zone %d temp %d stat %d\n",
			__func__, tzid, temp, stat);
	}
	pr_info("%s\n", __func__);

	queue_delayed_work_on(get_random_for_type(CPU_ID), tmu_test->suspend_wq,
		&tmu_test->suspend_work, msecs_to_jiffies(TMU_SUSPEND_RESUME_DELAY));
}

static void acpm_tmu_mbox_test_init(void)
{
	int i;
	char buf[32];

	if (!tmu_test->wq_init_done) {
		pr_info("%s\n", __func__);

		for (i = 0; i < NUM_OF_WQ; i++) {
			snprintf(buf, sizeof(buf), "acpm_tmu_rd_tmp_random_wq%d", i);
			tmu_test->rd_tmp_random_wq[i] = alloc_workqueue("%s",
					__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
					1, buf);
			snprintf(buf, sizeof(buf), "acpm_tmu_rd_tmp_concur_wq%d", i);
			tmu_test->rd_tmp_concur_wq[i] = alloc_workqueue("%s",
					__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
					1, buf);
		}
		tmu_test->rd_tmp_stress_trigger_wq = alloc_workqueue("%s",
				__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
				1, "acpm_tmu_rd_tmp_trigger_wq");
		tmu_test->suspend_wq = alloc_workqueue("%s",
				__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
				1, "acpm_tmu_mbox_suspend_wq");
		tmu_test->resume_wq = alloc_workqueue("%s",
				__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
				1, "acpm_tmu_mbox_resume_wq");

		for (i = 0; i < NUM_OF_WQ; i++) {
			INIT_DELAYED_WORK(&tmu_test->rd_tmp_random_wk[i],
				acpm_debug_tmu_rd_tmp_random);
			INIT_DELAYED_WORK(&tmu_test->rd_tmp_concur_wk[i],
				acpm_debug_tmu_rd_tmp_concur);
		}
		INIT_DELAYED_WORK(&tmu_test->rd_tmp_stress_trigger_wk,
				acpm_debug_tmu_rd_tmp_stress_trigger);
		INIT_DELAYED_WORK(&tmu_test->suspend_work, acpm_debug_tmu_suspend);
		INIT_DELAYED_WORK(&tmu_test->resume_work, acpm_debug_tmu_resume);

		tmu_test->wq_init_done = true;
		pr_info("%s done\n", __func__);
	}
}

static void acpm_framework_tmu_mbox_test(bool start)
{
	if (start) {
		acpm_tmu_mbox_test_init();
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				tmu_test->suspend_wq,
				&tmu_test->suspend_work,
				msecs_to_jiffies(TMU_SUSPEND_RESUME_DELAY));
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				tmu_test->rd_tmp_stress_trigger_wq,
				&tmu_test->rd_tmp_stress_trigger_wk,
				msecs_to_jiffies(TMU_READ_TEMP_TRIGGER_DELAY));
	} else {
		cancel_delayed_work_sync(&tmu_test->suspend_work);
		cancel_delayed_work_sync(&tmu_test->resume_work);
		cancel_delayed_work_sync(&tmu_test->rd_tmp_stress_trigger_wk);
	}
}

static int acpm_tmu_mbox_test_setting(struct acpm_info *acpm, u64 subcmd)
{
	if (subcmd >= ACPM_MBOX_TEST_CMD_MAX) {
		pr_err("%s, sub-cmd:%d, out of range!\n", __func__, subcmd);
		return -EINVAL;
	} else if (ACPM_MBOX_TMU_TEST_START == subcmd) {
		acpm_framework_tmu_mbox_test(true);
	} else if (ACPM_MBOX_TMU_TEST_STOP == subcmd) {
		acpm_framework_tmu_mbox_test(false);
	}

	return 0;
}

static int debug_acpm_tmu_mbox_test_set(void *data, u64 val)
{
	struct acpm_info *acpm = (struct acpm_info *)data;

	return acpm_tmu_mbox_test_setting(acpm, val);
}

DEFINE_SIMPLE_ATTRIBUTE(debug_acpm_tmu_mbox_test_fops,
		NULL, debug_acpm_tmu_mbox_test_set, "0x%016llx\n");

static void acpm_test_debugfs_init(struct acpm_tmu_mbox_test *tmu)
{
	struct dentry *den_tmu;

	den_tmu = debugfs_lookup("acpm_framework", NULL);
	debugfs_create_file("acpm_tmu_mbox_test", 0644, den_tmu, tmu,
			&debug_acpm_tmu_mbox_test_fops);
}

static int acpm_mbox_test_probe(struct platform_device *pdev)
{
	struct acpm_tmu_mbox_test *tmu;

	dev_info(&pdev->dev, "%s\n", __func__);

	tmu = devm_kzalloc(&pdev->dev, sizeof(struct acpm_tmu_mbox_test), GFP_KERNEL);
	if (IS_ERR(tmu))
		return PTR_ERR(tmu);
	if (!tmu)
		return -ENOMEM;

	acpm_test_debugfs_init(tmu);

	tmu_test = tmu;

	dev_info(&pdev->dev, "%s done\n", __func__);
	return 0;
}

static int acpm_mbox_test_remove(struct platform_device *pdev)
{
	int i;

	flush_workqueue(tmu_test->rd_tmp_stress_trigger_wq);
	flush_workqueue(tmu_test->suspend_wq);
	flush_workqueue(tmu_test->resume_wq);
	for (i = 0; i < NUM_OF_WQ; i++) {
		flush_workqueue(tmu_test->rd_tmp_random_wq[i]);
		flush_workqueue(tmu_test->rd_tmp_concur_wq[i]);
	}

	destroy_workqueue(tmu_test->rd_tmp_stress_trigger_wq);
	destroy_workqueue(tmu_test->suspend_wq);
	destroy_workqueue(tmu_test->resume_wq);
	for (i = 0; i < NUM_OF_WQ; i++) {
		destroy_workqueue(tmu_test->rd_tmp_random_wq[i]);
		destroy_workqueue(tmu_test->rd_tmp_concur_wq[i]);
	}

	kfree(tmu_test);
	pr_info("%s done.\n", __func__);
	return 0;
}

static const struct of_device_id acpm_mbox_test_match[] = {
	{ .compatible = "google,acpm-mbox-test" },
	{},
};
MODULE_DEVICE_TABLE(of, acpm_mbox_test_match);

static struct platform_driver acpm_mbox_test_driver = {
	.probe	= acpm_mbox_test_probe,
	.remove	= acpm_mbox_test_remove,
	.driver	= {
		.name = "acpm-mbox-test",
		.owner	= THIS_MODULE,
		.of_match_table	= acpm_mbox_test_match,
	},
};

static int __init exynos_acpm_mbox_test_init(void)
{
	platform_driver_register(&acpm_mbox_test_driver);
	return 0;
}
late_initcall_sync(exynos_acpm_mbox_test_init);

static void __exit exynos_acpm_mbox_test_exit(void)
{
	platform_driver_unregister(&acpm_mbox_test_driver);
}
module_exit(exynos_acpm_mbox_test_exit);
MODULE_LICENSE("GPL");

