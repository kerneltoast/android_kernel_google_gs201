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
#include <linux/pm_qos.h>
#include <linux/random.h>
#include <linux/cpufreq.h>
#include <soc/google/cal-if.h>
#include <dt-bindings/clock/gs101.h>
#include <soc/google/exynos-devfreq.h>
#include "../../../soc/google/cal-if/acpm_dvfs.h"

static int acpm_tmu_log;
static int acpm_dvfs_log;
static struct acpm_mbox_test *mbox;
static struct acpm_dvfs_test *dvfs_test;
static int init_done;

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
			mbox->tmu->rd_tmp_random_wq[i],
			&mbox->tmu->rd_tmp_random_wk[i],
			msecs_to_jiffies(get_random_for_type(DELAY_MS)));
		queue_delayed_work_on(get_random_for_type(CPU_ID),
			mbox->tmu->rd_tmp_concur_wq[i],
			&mbox->tmu->rd_tmp_concur_wk[i],
			msecs_to_jiffies(0));
	}
	queue_delayed_work_on(get_random_for_type(CPU_ID),
		mbox->tmu->rd_tmp_stress_trigger_wq,
		&mbox->tmu->rd_tmp_stress_trigger_wk,
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

	queue_delayed_work_on(get_random_for_type(CPU_ID), mbox->tmu->resume_wq,
		&mbox->tmu->resume_work, msecs_to_jiffies(TMU_SUSPEND_RESUME_DELAY));
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

	queue_delayed_work_on(get_random_for_type(CPU_ID), mbox->tmu->suspend_wq,
		&mbox->tmu->suspend_work, msecs_to_jiffies(TMU_SUSPEND_RESUME_DELAY));
}

static void acpm_mbox_test_tmu_init(void)
{
	int i;
	char buf[32];

	if (!mbox->tmu->wq_init_done) {
		pr_info("%s\n", __func__);

		for (i = 0; i < NUM_OF_WQ; i++) {
			snprintf(buf, sizeof(buf), "acpm_tmu_rd_tmp_random_wq%d", i);
			mbox->tmu->rd_tmp_random_wq[i] = alloc_workqueue("%s",
					__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
					1, buf);
			snprintf(buf, sizeof(buf), "acpm_tmu_rd_tmp_concur_wq%d", i);
			mbox->tmu->rd_tmp_concur_wq[i] = alloc_workqueue("%s",
					__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
					1, buf);
		}
		mbox->tmu->rd_tmp_stress_trigger_wq = alloc_workqueue("%s",
				__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
				1, "acpm_tmu_rd_tmp_trigger_wq");
		mbox->tmu->suspend_wq = alloc_workqueue("%s",
				__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
				1, "acpm_tmu_mbox_suspend_wq");
		mbox->tmu->resume_wq = alloc_workqueue("%s",
				__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND,
				1, "acpm_tmu_mbox_resume_wq");

		for (i = 0; i < NUM_OF_WQ; i++) {
			INIT_DELAYED_WORK(&mbox->tmu->rd_tmp_random_wk[i],
				acpm_debug_tmu_rd_tmp_random);
			INIT_DELAYED_WORK(&mbox->tmu->rd_tmp_concur_wk[i],
				acpm_debug_tmu_rd_tmp_concur);
		}
		INIT_DELAYED_WORK(&mbox->tmu->rd_tmp_stress_trigger_wk,
				acpm_debug_tmu_rd_tmp_stress_trigger);
		INIT_DELAYED_WORK(&mbox->tmu->suspend_work, acpm_debug_tmu_suspend);
		INIT_DELAYED_WORK(&mbox->tmu->resume_work, acpm_debug_tmu_resume);

		mbox->tmu->wq_init_done = true;
		pr_info("%s done\n", __func__);
	}
}

static void acpm_framework_mbox_test(bool start)
{
	if (start) {
		acpm_mbox_test_tmu_init();
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				mbox->tmu->suspend_wq,
				&mbox->tmu->suspend_work,
				msecs_to_jiffies(TMU_SUSPEND_RESUME_DELAY));
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				mbox->tmu->rd_tmp_stress_trigger_wq,
				&mbox->tmu->rd_tmp_stress_trigger_wk,
				msecs_to_jiffies(TMU_READ_TEMP_TRIGGER_DELAY));
	} else {
		cancel_delayed_work_sync(&mbox->tmu->suspend_work);
		cancel_delayed_work_sync(&mbox->tmu->resume_work);
		cancel_delayed_work_sync(&mbox->tmu->rd_tmp_stress_trigger_wk);
	}
}

static int acpm_mbox_test_setting(struct acpm_info *acpm, u64 subcmd)
{
	if (subcmd >= ACPM_MBOX_TEST_CMD_MAX) {
		pr_err("%s, sub-cmd:%d, out of range!\n", __func__, subcmd);
		return -EINVAL;
	} else if (ACPM_MBOX_TEST_START == subcmd) {
		acpm_framework_mbox_test(true);
	} else if (ACPM_MBOX_TEST_STOP == subcmd) {
		acpm_framework_mbox_test(false);
	}

	return 0;
}

static int debug_acpm_mbox_test_set(void *data, u64 val)
{
	struct acpm_info *acpm = (struct acpm_info *)data;

	return acpm_mbox_test_setting(acpm, val);
}

static int get_cpu_policy_num(unsigned int dm_id)
{
	int ret;

	switch (dm_id) {
	case DVFS_CPUCL0:
		ret = CPUCL0_POLICY;
		break;
	case DVFS_CPUCL1:
		ret =  CPUCL1_POLICY;
		break;
	case DVFS_CPUCL2:
		ret =  CPUCL2_POLICY;
		break;
	default:
		pr_err("%s, dm_id: %d not support\n", __func__, dm_id);
		ret =  -EINVAL;
		break;
	}

	return ret;
}

static unsigned int get_random_rate(unsigned int dm_id)
{
	unsigned int random, index;

	random = get_random_int();
	index = random % dvfs_test->dm[dm_id]->size;
	return dvfs_test->dm[dm_id]->table[index].freq;
}

static unsigned int acpm_dvfs_get_devfreq(unsigned int devfreq_type)
{
	unsigned int freq;

	freq = exynos_devfreq_get_domain_freq(devfreq_type);

	if (freq == 0)
		pr_err("%s: Failed get frequency\n", __func__);

	return freq;
}

static int acpm_dvfs_set_devfreq(unsigned int dm_id, unsigned int rate, int cycle)
{
	unsigned long long before, after, latency;
	unsigned int get_rate;
	int ret = 0;

	before = sched_clock();
	ret = exynos_devfreq_lock_freq(dm_id, rate);
	after = sched_clock();
	latency = after - before;

	if (ret < 0)
		pr_err("%s, ret=%d\n", ret);

	mdelay(100);

	get_rate = acpm_dvfs_get_devfreq(dm_id);
	if (cycle >= 0) {
		dvfs_test->dm[dm_id]->stats[cycle].latency = latency /*ns*/;
		dvfs_test->dm[dm_id]->stats[cycle].set_rate = rate;
		dvfs_test->dm[dm_id]->stats[cycle].get_rate = get_rate;
		dvfs_test->dm[dm_id]->total_cycle_cnt++;
	}

	pr_info("%s: domain[%s]set_rate: %d Hz, "
		"get_rate: %d Hz, latency: %llu ns, ret: %d\n",
		__func__, dvfs_test->dm[dm_id]->name, rate, get_rate, latency, ret);

	if (acpm_dvfs_log && cycle >= 0) {
		pr_info("%s: stats:[%s]set_rate: %d Hz, "
			"get_rate: %d Hz, latency: %llu ns, total_cnt:%d\n",
			__func__, dvfs_test->dm[dm_id]->name,
			dvfs_test->dm[dm_id]->stats[cycle].set_rate,
			dvfs_test->dm[dm_id]->stats[cycle].get_rate,
			dvfs_test->dm[dm_id]->stats[cycle].latency,
			dvfs_test->dm[dm_id]->total_cycle_cnt);
	}

	return ret;
}

static int acpm_dvfs_set_cpufreq(unsigned int dm_id, unsigned int rate, int cycle)
{
	struct cpufreq_policy *policy;
	unsigned long long before, after, latency;
	unsigned int cl0freq, cl1freq, cl2freq;
	int policy_id, ret = 0;

	policy_id = get_cpu_policy_num(dm_id);
	if (policy_id < 0)
		return -EINVAL;

	policy = cpufreq_cpu_get(policy_id);
	if (!policy)
		return -EINVAL;

	before = sched_clock();

	ret = cpufreq_driver_target(policy, rate, CPUFREQ_RELATION_C);
	if (ret < 0)
		pr_err("%s, cpufreq target failed, ret: %d\n", ret);

	after = sched_clock();
	latency = after - before;

	mdelay(100);

	if (cycle >= 0) {
		dvfs_test->dm[dm_id]->stats[cycle].latency = latency /*ns*/;
		dvfs_test->dm[dm_id]->stats[cycle].set_rate = rate;
		dvfs_test->dm[dm_id]->stats[cycle].get_rate = cpufreq_get(policy_id);
		dvfs_test->dm[dm_id]->total_cycle_cnt++;
	}

	cl0freq = cpufreq_get(CPUCL0_POLICY);
	cl1freq = cpufreq_get(CPUCL1_POLICY);
	cl2freq = cpufreq_get(CPUCL2_POLICY);
	pr_info("%s: domain[%s]set_rate: %d Hz, CL0: %d Hz, "
		"CL1: %d Hz, CL2: %d Hz, latency: %llu ns, ret: %d\n",
		__func__, dvfs_test->dm[dm_id]->name, rate,
		cl0freq, cl1freq, cl2freq, latency, ret);

	if (acpm_dvfs_log && cycle >= 0) {
		pr_info("%s: stats:[%s]set_rate: %d Hz, "
			"get_rate: %d Hz, latency: %llu ns, total_cnt: %d\n",
			__func__, dvfs_test->dm[dm_id]->name,
			dvfs_test->dm[dm_id]->stats[cycle].set_rate,
			dvfs_test->dm[dm_id]->stats[cycle].get_rate,
			dvfs_test->dm[dm_id]->stats[cycle].latency,
			dvfs_test->dm[dm_id]->total_cycle_cnt);
	}

	return 0;
}

static void acpm_dvfs_latency_summary(unsigned int dm_id, int cycle)
{
	unsigned int latency;
	int i;

	if (dvfs_test->dm[dm_id]->total_cycle_cnt && cycle >= 0) {
		latency = dvfs_test->dm[dm_id]->stats[cycle].latency;
		latency /= MICRO_SEC;

		for (i = (TIME_SCALES - 1); i >= 0; i--) {
			if (latency >= dvfs_test->dm[dm_id]->scales[i].limit) {
				dvfs_test->dm[dm_id]->scales[i].count++;
				break;
			}
		}
	}
}

static void acpm_dvfs_stats_dump(void)
{
	unsigned int cycle_cnt;
	int dm_id, cycle;

	for (dm_id = 0; dm_id < NUM_OF_DVFS_DOMAINS; dm_id++) {
		if (dvfs_test->dm[dm_id]->total_cycle_cnt) {
			pr_info("==================================="
				"==================================="
				"==================================="
				"===================================\n");
			cycle_cnt = dvfs_test->dm[dm_id]->total_cycle_cnt;

			pr_info("dm[%s] >10ms:%2d, 10ms~1ms:%2d, "
				"1ms~100us:%2d, 100us~80us:%2d, "
				"80us~60us:%2d, 60us~40us:%2d, "
				"40us~20us:%2d, 20us~10us:%2d, "
				"10us~1us:%2d, <1us:%2d\n",
				dvfs_test->dm[dm_id]->name,
				dvfs_test->dm[dm_id]->scales[9].count,
				dvfs_test->dm[dm_id]->scales[8].count,
				dvfs_test->dm[dm_id]->scales[7].count,
				dvfs_test->dm[dm_id]->scales[6].count,
				dvfs_test->dm[dm_id]->scales[5].count,
				dvfs_test->dm[dm_id]->scales[4].count,
				dvfs_test->dm[dm_id]->scales[3].count,
				dvfs_test->dm[dm_id]->scales[2].count,
				dvfs_test->dm[dm_id]->scales[1].count,
				dvfs_test->dm[dm_id]->scales[0].count);
			pr_info("dm[%s] >10ms:%2d%% 10ms~1ms:%2d%% "
				"1ms~100us:%2d%% 100us~80us:%2d%% "
				"80us~60us:%2d%% 60us~40us:%2d%% "
				"40us~20us:%2d%% 20us~10us:%2d%% "
				"10us~1us:%2d%%, <1us:%2d%%\n",
				dvfs_test->dm[dm_id]->name,
				dvfs_test->dm[dm_id]->scales[9].count * 100 / cycle_cnt,
				dvfs_test->dm[dm_id]->scales[8].count * 100 / cycle_cnt,
				dvfs_test->dm[dm_id]->scales[7].count * 100 / cycle_cnt,
				dvfs_test->dm[dm_id]->scales[6].count * 100 / cycle_cnt,
				dvfs_test->dm[dm_id]->scales[5].count * 100 / cycle_cnt,
				dvfs_test->dm[dm_id]->scales[4].count * 100 / cycle_cnt,
				dvfs_test->dm[dm_id]->scales[3].count * 100 / cycle_cnt,
				dvfs_test->dm[dm_id]->scales[2].count * 100 / cycle_cnt,
				dvfs_test->dm[dm_id]->scales[1].count * 100 / cycle_cnt,
				dvfs_test->dm[dm_id]->scales[0].count * 100 / cycle_cnt);

		pr_info("==================================="
			"==================================="
			"==================================="
			"===================================\n");
		for (cycle = 0; cycle < DVFS_TEST_CYCLE; cycle++)
			pr_info("%s: dm_id[%d], set_rate: %d Hz, "
				"get_rate: %d Hz, latency= %u ns\n",
				__func__, dm_id,
				dvfs_test->dm[dm_id]->stats[cycle].set_rate,
				dvfs_test->dm[dm_id]->stats[cycle].get_rate,
				dvfs_test->dm[dm_id]->stats[cycle].latency);
		pr_info("\n");
		}
	}
}

static int init_domain_freq_table(struct acpm_dvfs_test *dvfs, int cal_id, int dm_id);
static int acpm_dvfs_test_setting(struct acpm_info *acpm, u64 subcmd)
{
	unsigned int domain = (unsigned int)subcmd;
	static unsigned int pre_set_rate;
	unsigned int set_rate;
	int cycle = 0, cal_id, dm_id;
	int index, i;

	if (!dvfs_test) {
		pr_err("%s, dvfs_test is NULL\n", __func__);
		return -ENOMEM;
	}

	if (subcmd >= ACPM_DVFS_CMD_MAX) {
		pr_err("%s, sub-cmd:%d, out of range!\n", __func__, subcmd);
		return -EINVAL;
	}

	if (!init_done) {
		for (cal_id = ACPM_DVFS_MIF, dm_id = DVFS_MIF;
				cal_id <= ACPM_DVFS_CPUCL2; cal_id++, dm_id++)
			init_domain_freq_table(dvfs_test, cal_id, dm_id);
		if (acpm_dvfs_log) {
			for (i = DVFS_MIF; i < NUM_OF_DVFS_DOMAINS; i++) {
				for (index = 0; index < dvfs_test->dm[i]->size; index++)
					pr_info("%s: dvfs_test->dm[%d]->table[%d] = %d Hz\n",
						__func__, i, index,
						dvfs_test->dm[i]->table[index].freq);
			}
		}
		init_done = true;
	}

	switch (domain) {
	case ACPM_DVFS_TEST_MIF:
	case ACPM_DVFS_TEST_INT:
		while (cycle < DVFS_TEST_CYCLE) {
			set_rate = get_random_rate(domain);
			if (pre_set_rate == set_rate) {
				/*ignore the stats for duplicated freq request*/
				acpm_dvfs_set_devfreq(domain, set_rate, -1);
			} else {
				acpm_dvfs_set_devfreq(domain, set_rate, cycle);
				acpm_dvfs_latency_summary(domain, cycle);
				cycle++;
			}
			pre_set_rate = set_rate;
		}
		break;
	case ACPM_DVFS_TEST_CPUCL0:
		/* Lock CPUCL1/2 on min freq */
		/* to prevent CPUCL0 from being restricted to high by minlock */
		acpm_dvfs_set_cpufreq(DVFS_CPUCL1,
				      dvfs_test->dm[DVFS_CPUCL1]->min_freq, -1);
		acpm_dvfs_set_cpufreq(DVFS_CPUCL2,
				      dvfs_test->dm[DVFS_CPUCL2]->min_freq, -1);
	case ACPM_DVFS_TEST_CPUCL1:
	case ACPM_DVFS_TEST_CPUCL2:
		while (cycle < DVFS_TEST_CYCLE) {
			set_rate = get_random_rate(domain);
			if (pre_set_rate == set_rate) {
				/*ignore the stats for duplicated freq request*/
				acpm_dvfs_set_cpufreq(domain, set_rate, -1);
			} else {
				acpm_dvfs_set_cpufreq(domain, set_rate, cycle);
				acpm_dvfs_latency_summary(domain, cycle);
				cycle++;
			}
			pre_set_rate = set_rate;
		}
		break;
	case ACPM_DVFS_TEST_RESULT:
		acpm_dvfs_stats_dump();
		break;
	default:
		pr_err("%s, subcmd: %d not support\n", __func__, subcmd);
		return -EINVAL;
	}

	return 0;
}

static int debug_acpm_dvfs_test_set(void *data, u64 val)
{
	struct acpm_info *acpm = (struct acpm_info *)data;

	return acpm_dvfs_test_setting(acpm, val);
}

DEFINE_SIMPLE_ATTRIBUTE(debug_acpm_mbox_test_fops,
			NULL, debug_acpm_mbox_test_set, "0x%016llx\n");
DEFINE_SIMPLE_ATTRIBUTE(debug_acpm_dvfs_test_fops,
			NULL, debug_acpm_dvfs_test_set, "0x%016llx\n");

static void acpm_test_debugfs_init(struct acpm_mbox_test *mbox)
{
	struct dentry *den_mbox;

	den_mbox = debugfs_lookup("acpm_framework", NULL);
	debugfs_create_file("acpm_mbox_test", 0644, den_mbox, mbox,
			    &debug_acpm_mbox_test_fops);
	debugfs_create_file("acpm_dvfs_test", 0644, den_mbox, mbox,
			    &debug_acpm_dvfs_test_fops);
}

static int init_domain_freq_table(struct acpm_dvfs_test *dvfs, int cal_id, int dm_id)
{
	unsigned int orig_table_size;
	unsigned long *cal_freq_table;
	int index, r_index, ret, policy_id;
	struct acpm_dvfs_dm *dm;
	struct cpufreq_policy *policy;

	dm = kmalloc(sizeof(*dm), GFP_KERNEL);
	if (!dm) {
		pr_err("%s, dm alloc failed\n", __func__);
		return -ENOMEM;
	}
	/*
	 * Set min/max frequency.
	 * If max-freq property exists in device tree, max frequency is
	 * selected to smaller one between the value defined in device
	 * tree and CAL. In case of min-freq, min frequency is selected
	 * to bigger one.
	 */
	switch (cal_id) {
	case ACPM_DVFS_MIF:
	case ACPM_DVFS_INT:
		ret = exynos_devfreq_get_boundary(dm_id, &dvfs->max_freq, &dvfs->min_freq);
		if (ret < 0) {
			dvfs->max_freq = cal_dfs_get_max_freq(cal_id);
			dvfs->min_freq = cal_dfs_get_min_freq(cal_id);
			pr_warn("%s, cal_id: %d, get boundary failed, ret: %d\n",
				__func__, cal_id, ret);
		}
		break;
	case ACPM_DVFS_CPUCL0:
	case ACPM_DVFS_CPUCL1:
	case ACPM_DVFS_CPUCL2:
		policy_id = get_cpu_policy_num(dm_id);
		if (policy_id < 0)
			return -EINVAL;

		policy = cpufreq_cpu_get(policy_id);
		if (policy) {
			dvfs->max_freq = policy->cpuinfo.max_freq;
			dvfs->min_freq = policy->cpuinfo.min_freq;
		} else {
			dvfs->max_freq = cal_dfs_get_max_freq(cal_id);
			dvfs->min_freq = cal_dfs_get_min_freq(cal_id);
			pr_warn("%s, cal_id: %d, get boundary failed\n",
				__func__, cal_id);
		}
		break;
	default:
		pr_err("%s, cal_id: %d not support\n", __func__, cal_id);
		return -EINVAL;
	}

	if (acpm_dvfs_log)
		pr_info("%s: max_freq = %d, min_freq = %d\n",
			__func__, dvfs->max_freq, dvfs->min_freq);

	/*
	 * Allocate temporary frequency and voltage tables
	 * to get DVFS table from CAL.
	 */
	orig_table_size = cal_dfs_get_lv_num(cal_id);

	cal_freq_table = kcalloc(orig_table_size, sizeof(unsigned long), GFP_KERNEL);

	if (!cal_freq_table)
		return -ENOMEM;

	cal_dfs_get_rate_table(cal_id, cal_freq_table);

	/*
	 * Set frequency table size.
	 */
	dvfs->size = 0;
	for (index = 0; index < orig_table_size; index++) {
		if (cal_freq_table[index] > dvfs->max_freq) {
			if (acpm_dvfs_log)
				pr_info("%s: cal_freq_table[%d]: %d > max_freq: %d\n",
					__func__, index, cal_freq_table[index], dvfs->max_freq);
			continue;
		}
		if (cal_freq_table[index] < dvfs->min_freq) {
			if (acpm_dvfs_log)
				pr_info("%s: cal_freq_table[%d]: %d < min_freq: %d\n",
					__func__, index, cal_freq_table[index], dvfs->min_freq);
			continue;
		}
		dvfs->size++;

		if (acpm_dvfs_log)
			pr_info("%s: cal_freq_table[%d] = %d Hz, table_size = %d\n",
				__func__, index, cal_freq_table[index], dvfs->size);
	}

	/*
	 * Allocate frequency table.
	 * Last row of frequency table must be set to CPUFREQ_TABLE_END.
	 * Table size should be one larger than real table size.
	 */
	dm->table = kcalloc(dvfs->size + 1, sizeof(*dm->table), GFP_KERNEL);

	if (!dm->table) {
		kfree(cal_freq_table);
		return -ENOMEM;
	}

	/*
	 * Initialize frequency table.
	 * The frequency table obtained from ECT is in descending order, but
	 * the frequency table of domain is organized in ascending order for
	 * Android BatteryStat service.
	 */
	index = 0;
	r_index = orig_table_size;
	while (--r_index >= 0) {
		if (cal_freq_table[r_index] > dvfs->max_freq)
			continue;
		if (cal_freq_table[r_index] < dvfs->min_freq)
			continue;

		dm->table[index].freq = cal_freq_table[r_index];
		index++;
	}

	dm->table[index].freq = CPUFREQ_TABLE_END;

	dm->stats = kcalloc(DVFS_TEST_CYCLE, sizeof(struct acpm_dvfs_test_stats), GFP_KERNEL);

	dm->scales = kcalloc(TIME_SCALES, sizeof(struct stats_scale), GFP_KERNEL);
	memcpy(dm->scales, buckets, TIME_SCALES * sizeof(struct stats_scale));

	cal_id %= NUM_OF_DVFS_DOMAINS;
	dvfs->dm[cal_id] = dm;
	dvfs->dm[cal_id]->name = gs101_dvfs_domains[cal_id].name;
	dvfs->dm[cal_id]->max_freq = dvfs->max_freq;
	dvfs->dm[cal_id]->min_freq = dvfs->min_freq;
	dvfs->dm[cal_id]->size = dvfs->size;
	dvfs->dm[cal_id]->scales = dm->scales;

	kfree(cal_freq_table);

	return 0;
}

static int acpm_mbox_test_probe(struct platform_device *pdev)
{
	struct acpm_mbox_test *mbox_test;
	struct acpm_tmu_validity *tmu_validity;
	struct acpm_dvfs_test *dvfs;

	dev_info(&pdev->dev, "%s\n", __func__);

	mbox_test = devm_kzalloc(&pdev->dev, sizeof(struct acpm_mbox_test), GFP_KERNEL);
	if (IS_ERR(mbox_test))
		return PTR_ERR(mbox_test);
	if (!mbox_test)
		return -ENOMEM;

	tmu_validity = kmalloc(sizeof(*tmu_validity), GFP_KERNEL);
	if (!tmu_validity) {
		pr_err("%s, tmu_validity alloc failed\n", __func__);
		return -ENOMEM;
	}

	mbox_test->tmu = tmu_validity;

	dvfs = devm_kzalloc(&pdev->dev, sizeof(struct acpm_dvfs_test), GFP_KERNEL);

	if (IS_ERR(dvfs))
		return PTR_ERR(dvfs);
	if (!dvfs)
		return -ENOMEM;

	acpm_test_debugfs_init(mbox_test);

	mbox = mbox_test;
	dvfs_test = dvfs;

	dev_info(&pdev->dev, "%s done\n", __func__);
	return 0;
}

static int acpm_mbox_test_remove(struct platform_device *pdev)
{
	int i;

	flush_workqueue(mbox->tmu->rd_tmp_stress_trigger_wq);
	flush_workqueue(mbox->tmu->suspend_wq);
	flush_workqueue(mbox->tmu->resume_wq);
	for (i = 0; i < NUM_OF_WQ; i++) {
		flush_workqueue(mbox->tmu->rd_tmp_random_wq[i]);
		flush_workqueue(mbox->tmu->rd_tmp_concur_wq[i]);
	}

	destroy_workqueue(mbox->tmu->rd_tmp_stress_trigger_wq);
	destroy_workqueue(mbox->tmu->suspend_wq);
	destroy_workqueue(mbox->tmu->resume_wq);
	for (i = 0; i < NUM_OF_WQ; i++) {
		destroy_workqueue(mbox->tmu->rd_tmp_random_wq[i]);
		destroy_workqueue(mbox->tmu->rd_tmp_concur_wq[i]);
	}

	kfree(mbox);
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

