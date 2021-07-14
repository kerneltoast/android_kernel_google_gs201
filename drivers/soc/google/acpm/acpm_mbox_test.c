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
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/rtc-s2mpg10.h>
#include <soc/google/pt.h>
#include <linux/list.h>

static int acpm_tmu_log;
static int acpm_dvfs_log;
static struct acpm_mbox_test *mbox;
static struct acpm_dvfs_test *dvfs_test;
static u8 aged[NR_RTC_CNT_REGS];
static struct list_head *slc_pt_list;

#define CHIP_REV_TYPE_A0          0
#define CPU_NUM_NO_HERA           6
#define CPU_NUM_WITH_HERA         8
static unsigned int get_random_for_type(int type)
{
	unsigned int random;
	int ret;

	random = get_random_int();

	switch (type) {
	case DELAY_MS:
		ret = random % 100;
		break;
	case TERMAL_ZONE_ID:
		ret = random % TZ_END;
		break;
	case CPU_ID:
		if (CHIP_REV_TYPE_A0 == gs_chipid_get_type() &&
		    CHIP_REV_TYPE_A0 == gs_chipid_get_revision()) {
			/* chipid: A0 */
			ret = random % CPU_NUM_NO_HERA;
		} else {
			/* chipid: A1/B0/... */
			ret = random % CPU_NUM_WITH_HERA;
		}
		break;
	case DVFS_DOMAIN_ID:
		ret = random % NUM_OF_DVFS_DOMAINS;
		break;
	case GRANVILLE_M_REG:
	case GRANVILLE_S_REG:
		ret = random % PMIC_RANDOM_ADDR_RANGE;
		break;
	case SLC_REQ_TYPE:
		ret = random % NUM_OF_SLC_REQ_TYPE;
		break;
	default:
		dev_err(mbox->device, "%s, type: %d not support\n", __func__,
			type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

#define acpm_ipc_latency_check() \
	do { \
		if (acpm_tmu_log) { \
			dev_info(mbox->device, "[acpm_tmu] type 0x%02x latency %llu ns ret %d\n", \
					message->req.type, latency, ret); \
		} \
	} while (0)

#define acpm_ipc_err_check() \
	do { \
		if (ret < 0) { \
			dev_warn(mbox->device, "[acpm_tmu] IPC error! type 0x%02x latency %llu ns ret %d\n", \
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
		dev_info(mbox->device,
			 "[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			 message.data[0], message.data[1], message.data[2],
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
		dev_info(mbox->device,
			 "[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			 message.data[0], message.data[1], message.data[2],
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
		dev_info(mbox->device,
			 "[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			 message.data[0], message.data[1], message.data[2],
			 message.data[3]);
	}

	dev_info(mbox->device, "%s: acpm irq %d cold cnt %d stat %d\n",
		 __func__, message.resp.rsvd2, message.resp.rsvd,
		 message.resp.stat);

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
		dev_info(mbox->device,
			 "[acpm_tmu] data 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x\n",
			 message.data[0], message.data[1], message.data[2],
			 message.data[3]);
	}
}

static void acpm_debug_tmu_rd_tmp_random(struct work_struct *work)
{
	int temp, stat;
	u32 tzid;

	tzid = get_random_for_type(TERMAL_ZONE_ID);

	acpm_tmu_set_read_temp(tzid, &temp, &stat);
	dev_info(mbox->device, "%s: thermal zone %d temp %d stat %d\n",
		 __func__, tzid, temp, stat);
}

static void acpm_debug_tmu_rd_tmp_concur(struct work_struct *work)
{
	int temp, stat;
	u32 tzid;

	tzid = get_random_for_type(TERMAL_ZONE_ID);

	acpm_tmu_set_read_temp(tzid, &temp, &stat);
	dev_info(mbox->device, "%s: thermal zone %d temp %d stat %d\n",
		 __func__, tzid, temp, stat);
}

static void acpm_mbox_dvfs_rate_random_change(struct work_struct *work)
{
	u32 domain, set_rate;

	domain = get_random_for_type(DVFS_DOMAIN_ID);

	if (!dvfs_test->dm[domain]) {
		cancel_delayed_work_sync(&mbox->dvfs->mbox_stress_trigger_wk);
	} else {
		set_rate = get_random_rate(domain);

		if (domain >= DVFS_CPUCL0)
			acpm_dvfs_set_cpufreq(domain, set_rate, -1);
		else
			acpm_dvfs_set_devfreq(domain, set_rate, -1);
	}
}

static void acpm_dvfs_mbox_stress_trigger(struct work_struct *work)
{
	int i;

	for (i = 0; i < NUM_OF_WQ; i++) {
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->dvfs->rate_change_wq[i],
				      &mbox->dvfs->rate_change_wk[i],
				      msecs_to_jiffies(get_random_for_type
						       (DELAY_MS)));
	}
	queue_delayed_work_on(get_random_for_type(CPU_ID),
			      mbox->dvfs->mbox_stress_trigger_wq,
			      &mbox->dvfs->mbox_stress_trigger_wk,
			      msecs_to_jiffies(STRESS_TRIGGER_DELAY));
}

static int acpm_mfd_rtc_update(void)
{
	u8 data, reg;
	int ret;

	ret = s2mpg10_read_reg(mbox->mfd->rtc, S2MPG10_RTC_UPDATE, &data);
	if (ret < 0) {
		dev_err(mbox->device, "%s: fail to read update ret(%d,%u)\n",
			__func__, ret, data);
		return ret;
	}

	data |= mbox->mfd->update_reg;

	reg = BIT(RTC_RUDR_SHIFT);

	data &= ~reg;
	ret = s2mpg10_write_reg(mbox->mfd->rtc, S2MPG10_RTC_UPDATE, data);
	if (ret) {
		dev_err(mbox->device, "%s: fail to write update ret(%d,%u)\n",
			__func__, ret, data);
		return ret;
	}

	usleep_range(50, 51);

	data |= reg;
	ret = s2mpg10_write_reg(mbox->mfd->rtc, S2MPG10_RTC_UPDATE, data);
	if (ret)
		dev_err(mbox->device, "%s: fail to write update ret(%d,%u)\n",
			__func__, ret, data);
	else
		usleep_range(1000, 1001);

	return ret;
}

static int acpm_rtc_monotonic_chk(u8 *now, u8 *old)
{
	u64 now_time_secs = 0, old_time_secs = 0;

	/* Use the simplified format to switch the RTC time to secs */
	now_time_secs = ((u64) now[RTC_YEAR] * SECS_PER_YEAR) +
	    ((u64) now[RTC_MONTH] * SECS_PER_MONTH) +
	    ((u64) now[RTC_DATE] * SECS_PER_DAY) +
	    (((u64) now[RTC_HOUR] & 0x1f) * SECS_PER_HR) +
	    ((u64) now[RTC_MIN] * SECS_PER_MIN) + (u64) now[RTC_SEC];

	old_time_secs = ((u64) old[RTC_YEAR] * SECS_PER_YEAR) +
	    ((u64) old[RTC_MONTH] * SECS_PER_MONTH) +
	    ((u64) old[RTC_DATE] * SECS_PER_DAY) +
	    (((u64) old[RTC_HOUR] & 0x1f) * SECS_PER_HR) +
	    ((u64) old[RTC_MIN] * SECS_PER_MIN) + (u64) old[RTC_SEC];

	if (now_time_secs >= old_time_secs) {
		return true;
	} else {
		dev_err(mbox->device,
			"%s: [Aged]%d-%02d-%02d %02d:%02d:%02d(0x%02x)%s, "
			"now: %llus, old: %llus\n", __func__,
			old[RTC_YEAR] + 2000, old[RTC_MONTH], old[RTC_DATE],
			old[RTC_HOUR] & 0x1f, old[RTC_MIN], old[RTC_SEC],
			old[RTC_WEEKDAY],
			old[RTC_HOUR] & BIT(HOUR_PM_SHIFT) ? "PM" : "AM",
			now_time_secs, old_time_secs);
		return false;
	}
}

static int acpm_mfd_rtc_read_time(void)
{
	u8 now[NR_RTC_CNT_REGS];
	int ret;

	mutex_lock(&mbox->mfd->lock);
	ret = acpm_mfd_rtc_update();
	if (ret < 0) {
		dev_err(mbox->device, "%s: rtc update failed, ret: %d\n",
			__func__, ret);
		cancel_delayed_work_sync(&mbox->mfd->mbox_stress_trigger_wk);
		goto out;
	}

	ret = s2mpg10_bulk_read(mbox->mfd->rtc, S2MPG10_RTC_SEC, NR_RTC_CNT_REGS,
			      now);
	if (ret < 0) {
		dev_err(mbox->device, "%s: fail to read time reg(%d)\n",
			__func__, ret);
		cancel_delayed_work_sync(&mbox->mfd->mbox_stress_trigger_wk);
		goto out;
	}

	if (acpm_rtc_monotonic_chk(now, aged)) {
		memcpy(aged, now, sizeof(now));
	} else {
		cancel_delayed_work_sync(&mbox->mfd->mbox_stress_trigger_wk);
		dev_err(mbox->device, "%s: RTC abnormal, cancel mfd stress\n",
			__func__);
	}

	dev_info(mbox->device, "%s: %d-%02d-%02d %02d:%02d:%02d(0x%02x)%s\n",
		 __func__, now[RTC_YEAR] + 2000, now[RTC_MONTH],
		 now[RTC_DATE], now[RTC_HOUR] & 0x1f, now[RTC_MIN],
		 now[RTC_SEC], now[RTC_WEEKDAY],
		 now[RTC_HOUR] & BIT(HOUR_PM_SHIFT) ? "PM" : "AM");

out:
	mutex_unlock(&mbox->mfd->lock);
	return ret;
}

static void acpm_mbox_mfd_s2mpg10_random_read(struct work_struct *work)
{
	u32 addr;
	u8 val = 0;

	addr = get_random_for_type(GRANVILLE_M_REG);

	if (s2mpg10_read_reg(mbox->mfd->s2mpg10_pmic, addr, &val)) {
		dev_err(mbox->device, "%s: Failed to read S2MPG10\n", __func__);
		cancel_delayed_work_sync(&mbox->mfd->mbox_stress_trigger_wk);
	} else
		dev_info(mbox->device, "%s: [S2MPG10]addr: 0x%X, val: 0x%X\n",
			 __func__, addr, val);

	acpm_mfd_rtc_read_time();
}

static void acpm_mbox_mfd_s2mpg11_random_read(struct work_struct *work)
{
	u32 addr;
	u8 val = 0;

	addr = get_random_for_type(GRANVILLE_S_REG);

	if (s2mpg11_read_reg(mbox->mfd->s2mpg11_pmic, addr, &val)) {
		dev_err(mbox->device, "%s: Failed to read S2MPG11\n", __func__);
		cancel_delayed_work_sync(&mbox->mfd->mbox_stress_trigger_wk);
	} else
		dev_info(mbox->device, "%s: [S2MPG11]addr: 0x%X, val: 0x%X\n",
			 __func__, addr, val);
}

static int acpm_pmic_ctrlist_stress(void)
{
	int ret, i, err_result = 0;
	u16 addr;
	u8 value;

	for (i = 0; i < sizeof(def_lck_regs_m) / sizeof(u16); i++) {
		addr = def_lck_regs_m[i];

		ret = s2mpg10_read_reg(mbox->mfd->s2mpg10_pmic, addr, &value);
		if (ret) {
			dev_err(mbox->device, "%s: fail to read ret: %d\n",
				__func__, ret);
			return ret;
		}

		/* Verify PMIC ctrlist by writing the same setting */
		ret = s2mpg10_write_reg(mbox->mfd->s2mpg10_pmic, addr, value);
		if (ret == 0) {
			dev_err(mbox->device,
				"%s: ctrlist protection failed, ret: %d\n",
				__func__, ret);
			err_result |= 1 << i;
		}

		dev_info(mbox->device,
			 "%s: addr: 0x%X, value: 0x%X, err_result: 0x%X\n",
			 __func__, addr, value, err_result);
	}

	for (i = 0; i < sizeof(def_lck_regs_s) / sizeof(u16); i++) {
		addr = def_lck_regs_s[i];

		ret = s2mpg11_read_reg(mbox->mfd->s2mpg11_pmic, addr, &value);
		if (ret) {
			dev_err(mbox->device, "%s: fail to read ret: %d\n",
				__func__, ret);
			return ret;
		}

		/* Verify PMIC ctrlist by writing the same setting */
		ret = s2mpg11_write_reg(mbox->mfd->s2mpg11_pmic, addr, value);
		if (ret == 0) {
			dev_err(mbox->device,
				"%s: ctrlist protection failed, ret: %d\n",
				__func__, ret);
			err_result |= (1 << i) << 16;
		}

		dev_info(mbox->device,
			 "%s: addr: 0x%X, value: 0x%X, err_result: 0x%X\n",
			 __func__, addr, value, err_result);
	}

	if (err_result != 0)
		mbox->mfd->ctrlist_err_result = err_result;
	else
		mbox->mfd->ctrlist_err_result = 1;

	return 0;
}

static void acpm_mfd_mbox_stress_trigger(struct work_struct *work)
{
	int i;

	if ((!mbox->mfd->s2mpg10_pmic)
	    || (!mbox->mfd->s2mpg11_pmic)) {
		cancel_delayed_work_sync(&mbox->mfd->mbox_stress_trigger_wk);
	} else {
		for (i = 0; i < NUM_OF_WQ; i++) {
			queue_delayed_work_on(get_random_for_type(CPU_ID),
					      mbox->mfd->s2mpg10_mfd_read_wq[i],
					      &mbox->
					      mfd->s2mpg10_mfd_read_wk[i],
					      msecs_to_jiffies
					      (get_random_for_type(DELAY_MS)));

			queue_delayed_work_on(get_random_for_type(CPU_ID),
					      mbox->mfd->s2mpg11_mfd_read_wq[i],
					      &mbox->
					      mfd->s2mpg11_mfd_read_wk[i],
					      msecs_to_jiffies
					      (get_random_for_type(DELAY_MS)));
		}
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->mfd->mbox_stress_trigger_wq,
				      &mbox->mfd->mbox_stress_trigger_wk,
				      msecs_to_jiffies(STRESS_TRIGGER_DELAY));
	}
}

/*
 * Send a command to APM and get back the return value.
 */
static int acpm_slc_mbox(unsigned int command, unsigned int arg,
			 unsigned long arg1)
{
	struct ipc_config config;
	unsigned int cmd[8];
	int ret;

	config.cmd = cmd;
	config.response = true;
	config.cmd[0] = 0;
	config.cmd[1] = arg;
	config.cmd[2] = command;
	config.cmd[3] = arg1;

	ret = acpm_ipc_send_data(IPC_AP_SLC, &config);
	if (ret == 0)
		ret = config.cmd[1];

	return ret;
}

static void acpm_mbox_slc_request_send(struct work_struct *work)
{
	u32 req_type;
	int slc_version;
	int slc_state;
	int size, pbha, vptid, p_way, s_way;
	int i, ptid;

	req_type = get_random_for_type(SLC_REQ_TYPE);

	switch (req_type) {
	case VERSION:
		slc_version = acpm_slc_mbox(PT_VERSION, 0, 0);
		dev_info(mbox->device, "%s: valid acpm firmware %d.%d.\n",
			 __func__, slc_version >> 16, slc_version & 0xffff);
		break;
	case STATE:
		slc_state = acpm_slc_mbox(SLC_STATE, 0, 0);
		dev_info(mbox->device,
			 "%s: slc_state: 0x%X, sicd: %d, ap_on: %d, "
			 "slc_on: %d, mif_on: %d\n", __func__, slc_state,
			 slc_state & BIT(0), (slc_state & BIT(1)) >> 1,
			 (slc_state & BIT(2)) >> 2, (slc_state & BIT(3)) >> 3);
		break;
	case PT_INFO:
		if (slc_pt_list == NULL) {
			dev_err(mbox->device, "%s, NO slc_pt_list!\n",
				__func__);
			break;
		}
		for (i = 0; i < mbox->slc->client_cnt; i++) {
			ptid = mbox->slc->ptid[i];
			size = acpm_slc_mbox(SLC_PT_INFO, ptid | PT_SIZE, 0);
			pbha =
			    acpm_slc_mbox(SLC_PT_INFO, ptid | (PBHA << 8), 0);
			vptid =
			    acpm_slc_mbox(SLC_PT_INFO, ptid | (VPTID << 8), 0);
			p_way =
			    acpm_slc_mbox(SLC_PT_INFO,
					  ptid | (PRIMARY_WAYS << 8), 0);
			s_way =
			    acpm_slc_mbox(SLC_PT_INFO,
					  ptid | (SECONDARY_WAYS << 8), 0);
			dev_info(mbox->device,
				 "%s: name: %s, size: %X, pbha: %X, vpid: %X, "
				 "pri_way: %X, sec_way: %X\n", __func__,
				 mbox->slc->client_name[i], size, pbha, vptid,
				 p_way, s_way);

			if (size < 0) {
				/* Client pt is disabled */
				cancel_delayed_work_sync(&mbox->slc->mbox_stress_trigger_wk);
				dev_err(mbox->device,
					"%s, No client: %s, cancel slc mbox stress\n",
					__func__, mbox->slc->client_name[i]);
			}
		}
		break;
	default:
		dev_err(mbox->device, "%s, req %d not support\n", __func__,
			req_type);
		break;
	}
}

static void acpm_slc_mbox_stress_trigger(struct work_struct *work)
{
	int i;

	for (i = 0; i < NUM_OF_WQ; i++) {
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->slc->slc_request_wq[i],
				      &mbox->slc->slc_request_wk[i],
				      msecs_to_jiffies(get_random_for_type
						       (DELAY_MS)));
	}

	queue_delayed_work_on(get_random_for_type(CPU_ID),
			      mbox->slc->mbox_stress_trigger_wq,
			      &mbox->slc->mbox_stress_trigger_wk,
			      msecs_to_jiffies(STRESS_TRIGGER_DELAY));
}

static void acpm_debug_tmu_rd_tmp_stress_trigger(struct work_struct *work)
{
	int i;

	for (i = 0; i < NUM_OF_WQ; i++) {
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->tmu->rd_tmp_random_wq[i],
				      &mbox->tmu->rd_tmp_random_wk[i],
				      msecs_to_jiffies(get_random_for_type
						       (DELAY_MS)));
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->tmu->rd_tmp_concur_wq[i],
				      &mbox->tmu->rd_tmp_concur_wk[i],
				      msecs_to_jiffies(0));
	}
	queue_delayed_work_on(get_random_for_type(CPU_ID),
			      mbox->tmu->rd_tmp_stress_trigger_wq,
			      &mbox->tmu->rd_tmp_stress_trigger_wk,
			      msecs_to_jiffies(STRESS_TRIGGER_DELAY));
}

#define TMU_SUSPEND_RESUME_DELAY      100
static void acpm_debug_tmu_suspend(struct work_struct *work)
{
	u32 tzid;

	for (tzid = 0; tzid < TZ_END; tzid++)
		acpm_tmu_tz_control(tzid, false);

	acpm_tmu_set_suspend(false);
	dev_info(mbox->device, "%s\n", __func__);

	queue_delayed_work_on(get_random_for_type(CPU_ID), mbox->tmu->resume_wq,
			      &mbox->tmu->resume_work,
			      msecs_to_jiffies(TMU_SUSPEND_RESUME_DELAY));
}

static void acpm_debug_tmu_resume(struct work_struct *work)
{
	int tzid, temp, stat;

	acpm_tmu_set_resume();

	for (tzid = 0; tzid < TZ_END; tzid++)
		acpm_tmu_tz_control(tzid, true);

	for (tzid = 0; tzid < TZ_END; tzid++) {
		acpm_tmu_set_read_temp(tzid, &temp, &stat);
		dev_info(mbox->device, "%s: thermal zone %d temp %d stat %d\n",
			 __func__, tzid, temp, stat);
	}
	dev_info(mbox->device, "%s\n", __func__);

	queue_delayed_work_on(get_random_for_type(CPU_ID),
			      mbox->tmu->suspend_wq, &mbox->tmu->suspend_work,
			      msecs_to_jiffies(TMU_SUSPEND_RESUME_DELAY));
}

static int acpm_mfd_set_pmic(void)
{
	struct device_node *p_np;
	struct device_node *np = mbox->device->of_node;
	struct s2mpg10_dev *s2mpg10 = NULL;
	struct s2mpg11_dev *s2mpg11 = NULL;
	struct i2c_client *i2c_main;
	struct i2c_client *i2c_sub;
	u8 update_val;
	int ret;

	if (mbox->mfd->init_done) {
		return 0;
	}

	/* Configure for main pmic */
	p_np = of_parse_phandle(np, "main-pmic", 0);
	if (p_np) {
		i2c_main = of_find_i2c_device_by_node(p_np);
		if (!i2c_main) {
			dev_err(mbox->device, "%s: Cannot find main-pmic i2c\n",
				__func__);
			return -ENODEV;
		}
		s2mpg10 = i2c_get_clientdata(i2c_main);
	} else
		dev_err(mbox->device, "%s: Cannot find main-pmic\n", __func__);

	of_node_put(p_np);

	if (!s2mpg10) {
		dev_err(mbox->device, "%s: S2MPG10 device not found\n",
			__func__);
		return -ENODEV;
	}

	i2c_set_clientdata(s2mpg10->pmic, s2mpg10);
	mbox->mfd->s2mpg10_pmic = s2mpg10->pmic;
	i2c_set_clientdata(s2mpg10->rtc, s2mpg10);
	mbox->mfd->rtc = s2mpg10->rtc;

	/* Configure for RTC bulk_read */
	ret = s2mpg10_read_reg(mbox->mfd->rtc, S2MPG10_RTC_UPDATE, &update_val);
	if (ret < 0) {
		dev_err(mbox->device, "%s: Fail to read RTC_UPDATE ret: %d\n",
			__func__, ret);
		return ret;
	}

	mbox->mfd->update_reg = update_val & ~(BIT(RTC_FREEZE_SHIFT) |
					       BIT(RTC_RUDR_SHIFT));

	/* Configure for sub pmic */
	p_np = of_parse_phandle(np, "sub-pmic", 0);
	if (p_np) {
		i2c_sub = of_find_i2c_device_by_node(p_np);
		if (!i2c_sub) {
			dev_err(mbox->device, "%s: Cannot find sub-pmic i2c\n",
				__func__);
			return -ENODEV;
		}
		s2mpg11 = i2c_get_clientdata(i2c_sub);
	} else
		dev_err(mbox->device, "%s: Cannot find sub-pmic\n", __func__);

	of_node_put(p_np);

	if (!s2mpg11) {
		dev_err(mbox->device, "%s: S2MPG11 device not found\n",
			__func__);
		return -ENODEV;
	}

	i2c_set_clientdata(s2mpg11->pmic, s2mpg11);
	mbox->mfd->s2mpg11_pmic = s2mpg11->pmic;

	mbox->mfd->init_done = 1;

	return 0;
}

unsigned int acpm_pt_clients_enable(void)
{
	struct pt_handle *client = NULL;
	unsigned int client_cnt = 0;

	list_for_each_entry(client, slc_pt_list, list) {
		mbox->slc->client_name[client_cnt] = client->node->name;
		mbox->slc->ptid[client_cnt] = pt_client_enable(client, 0);
		dev_info(mbox->device, "%s: node name: %s, slc-ptid[%d]: %d\n",
			 __func__, client->node->name, client_cnt,
			 mbox->slc->ptid[client_cnt]);
		client_cnt++;
	}

	return client_cnt;
}

void acpm_pt_clients_disable(void)
{
	struct pt_handle *client = NULL;

	list_for_each_entry(client, slc_pt_list, list) {
		dev_info(mbox->device, "%s: node name: %s\n", __func__,
			 client->node->name);
		pt_client_disable(client, 0);
	}
	mbox->slc->client_cnt = 0;
	memset(mbox->slc->ptid, 0, sizeof(u32) * PT_PTID_MAX);
	memset(mbox->slc->client_name, 0, sizeof(char) * PT_PTID_MAX);
}

static void acpm_slc_pt_init(void)
{
	slc_pt_list = pt_get_handle_list();
}

static void acpm_mbox_test_init(void)
{
	int i, ret = 0;
	char buf[32];

	if (!mbox->wq_init_done) {
		dev_info(mbox->device, "%s\n", __func__);

		for (i = 0; i < NUM_OF_WQ; i++) {
			snprintf(buf, sizeof(buf),
				 "acpm_tmu_rd_tmp_random_wq%d", i);
			mbox->tmu->rd_tmp_random_wq[i] =
			    create_freezable_workqueue(buf);
			snprintf(buf, sizeof(buf),
				 "acpm_tmu_rd_tmp_concur_wq%d", i);
			mbox->tmu->rd_tmp_concur_wq[i] =
			    create_freezable_workqueue(buf);
			snprintf(buf, sizeof(buf), "acpm_dvfs_req_wq%d", i);
			mbox->dvfs->rate_change_wq[i] =
			    create_freezable_workqueue(buf);
			snprintf(buf, sizeof(buf), "acpm_s2mpg10_mfd_rd_wq%d",
				 i);
			mbox->mfd->s2mpg10_mfd_read_wq[i] =
			    create_freezable_workqueue(buf);
			snprintf(buf, sizeof(buf), "acpm_s2mpg11_mfd_rd_wq%d",
				 i);
			mbox->mfd->s2mpg11_mfd_read_wq[i] =
			    create_freezable_workqueue(buf);
			snprintf(buf, sizeof(buf), "acpm_slc_request_wq%d", i);
			mbox->slc->slc_request_wq[i] =
			    create_freezable_workqueue(buf);
		}
		mbox->tmu->rd_tmp_stress_trigger_wq =
		    create_freezable_workqueue("acpm_tmu_rd_tmp_trigger_wq");
		mbox->dvfs->mbox_stress_trigger_wq =
		    create_freezable_workqueue("acpm_dvfs_mbox_trigger_wq");
		mbox->mfd->mbox_stress_trigger_wq =
		    create_freezable_workqueue("acpm_mfd_mbox_trigger_wq");
		mbox->slc->mbox_stress_trigger_wq =
		    create_freezable_workqueue("acpm_slc_mbox_trigger_wq");
		mbox->tmu->suspend_wq =
		    create_freezable_workqueue("acpm_tmu_mbox_suspend_wq");
		mbox->tmu->resume_wq =
		    create_freezable_workqueue("acpm_tmu_mbox_resume_wq");

		for (i = 0; i < NUM_OF_WQ; i++) {
			INIT_DELAYED_WORK(&mbox->tmu->rd_tmp_random_wk[i],
					  acpm_debug_tmu_rd_tmp_random);
			INIT_DELAYED_WORK(&mbox->tmu->rd_tmp_concur_wk[i],
					  acpm_debug_tmu_rd_tmp_concur);
			INIT_DELAYED_WORK(&mbox->dvfs->rate_change_wk[i],
					  acpm_mbox_dvfs_rate_random_change);
			INIT_DELAYED_WORK(&mbox->mfd->s2mpg10_mfd_read_wk[i],
					  acpm_mbox_mfd_s2mpg10_random_read);
			INIT_DELAYED_WORK(&mbox->mfd->s2mpg11_mfd_read_wk[i],
					  acpm_mbox_mfd_s2mpg11_random_read);
			INIT_DELAYED_WORK(&mbox->slc->slc_request_wk[i],
					  acpm_mbox_slc_request_send);
		}
		INIT_DELAYED_WORK(&mbox->tmu->rd_tmp_stress_trigger_wk,
				  acpm_debug_tmu_rd_tmp_stress_trigger);
		INIT_DELAYED_WORK(&mbox->dvfs->mbox_stress_trigger_wk,
				  acpm_dvfs_mbox_stress_trigger);
		INIT_DELAYED_WORK(&mbox->mfd->mbox_stress_trigger_wk,
				  acpm_mfd_mbox_stress_trigger);
		INIT_DELAYED_WORK(&mbox->slc->mbox_stress_trigger_wk,
				  acpm_slc_mbox_stress_trigger);
		INIT_DELAYED_WORK(&mbox->tmu->suspend_work,
				  acpm_debug_tmu_suspend);
		INIT_DELAYED_WORK(&mbox->tmu->resume_work,
				  acpm_debug_tmu_resume);

		ret = dvfs_freq_table_init();
		if (ret < 0)
			dev_err(mbox->device,
				"%s: table init failed, ret: %d\n", __func__,
				ret);

		ret = acpm_mfd_set_pmic();
		if (ret < 0)
			dev_err(mbox->device, "%s: set pmic failed, ret: %d\n",
				__func__, ret);

		acpm_slc_pt_init();

		mbox->wq_init_done = true;

		dev_info(mbox->device, "%s done!\n", __func__);
	}
}

static int dvfs_freq_table_init(void)
{
	int cal_id, dm_id, ret;
	int index, i;

	if (!dvfs_test->init_done) {
		for (cal_id = ACPM_DVFS_MIF, dm_id = DVFS_MIF;
		     cal_id <= ACPM_DVFS_CPUCL2; cal_id++, dm_id++) {
			ret = init_domain_freq_table(dvfs_test, cal_id, dm_id);
			if (ret < 0) {
				dev_err(mbox->device, "%s failed, ret = %d\n",
					__func__, ret);
				return ret;
			}
		}
		if (acpm_dvfs_log) {
			for (i = DVFS_MIF; i < NUM_OF_DVFS_DOMAINS; i++) {
				for (index = 0; index < dvfs_test->dm[i]->size;
				     index++)
					dev_info(mbox->device,
						 "%s: dvfs_test->dm[%d]->table[%d] = %d Hz\n",
						 __func__, i, index,
						 dvfs_test->dm[i]->table[index].
						 freq);
			}
		}
		dvfs_test->init_done = true;
	}

	return 0;
}

static void acpm_framework_mbox_test(bool start)
{
	if (start) {
		acpm_mbox_test_init();
		mbox->slc->client_cnt = acpm_pt_clients_enable();

		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->tmu->suspend_wq,
				      &mbox->tmu->suspend_work,
				      msecs_to_jiffies
				      (TMU_SUSPEND_RESUME_DELAY));
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->tmu->rd_tmp_stress_trigger_wq,
				      &mbox->tmu->rd_tmp_stress_trigger_wk,
				      msecs_to_jiffies(STRESS_TRIGGER_DELAY));
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->dvfs->mbox_stress_trigger_wq,
				      &mbox->dvfs->mbox_stress_trigger_wk,
				      msecs_to_jiffies(STRESS_TRIGGER_DELAY));
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->mfd->mbox_stress_trigger_wq,
				      &mbox->mfd->mbox_stress_trigger_wk,
				      msecs_to_jiffies(STRESS_TRIGGER_DELAY));
		queue_delayed_work_on(get_random_for_type(CPU_ID),
				      mbox->slc->mbox_stress_trigger_wq,
				      &mbox->slc->mbox_stress_trigger_wk,
				      msecs_to_jiffies(STRESS_TRIGGER_DELAY));
	} else {
		if (mbox->wq_init_done) {
			cancel_delayed_work_sync(&mbox->tmu->suspend_work);
			cancel_delayed_work_sync(&mbox->tmu->resume_work);
			cancel_delayed_work_sync(&mbox->tmu->rd_tmp_stress_trigger_wk);
			cancel_delayed_work_sync(&mbox->dvfs->mbox_stress_trigger_wk);
			cancel_delayed_work_sync(&mbox->mfd->mbox_stress_trigger_wk);
			cancel_delayed_work_sync(&mbox->slc->mbox_stress_trigger_wk);
			acpm_pt_clients_disable();
		}
	}
}

static int acpm_mbox_test_setting(struct acpm_info *acpm, u64 subcmd)
{
	int ret = 0;

	if (subcmd >= ACPM_MBOX_TEST_CMD_MAX) {
		dev_err(mbox->device, "%s, sub-cmd:%d, out of range!\n",
			__func__, subcmd);
		return -EINVAL;
	} else if (ACPM_MBOX_TEST_START == subcmd) {
		acpm_framework_mbox_test(true);
	} else if (ACPM_MBOX_TEST_STOP == subcmd) {
		acpm_framework_mbox_test(false);
	} else if (ACPM_MBOX_CTRLIST == subcmd) {
		ret = acpm_mfd_set_pmic();
		if (ret < 0)
			dev_err(mbox->device, "%s: set pmic failed, ret: %d\n",
				__func__, ret);
		else
			acpm_pmic_ctrlist_stress();
	}

	return 0;
}

static int debug_acpm_mbox_test_get(void *data, unsigned long long *val)
{
	if (mbox->mfd->ctrlist_err_result)
		*val = mbox->mfd->ctrlist_err_result;
	else
		*val = 0;

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
		ret = CPUCL1_POLICY;
		break;
	case DVFS_CPUCL2:
		ret = CPUCL2_POLICY;
		break;
	default:
		dev_err(mbox->device, "%s, dm_id: %d not support\n", __func__,
			dm_id);
		ret = -EINVAL;
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
		dev_err(mbox->device, "%s: Failed get frequency\n", __func__);

	return freq;
}

static int acpm_dvfs_set_devfreq(unsigned int dm_id, unsigned int rate,
				 int cycle)
{
	unsigned long long before, after, latency;
	unsigned int get_rate;
	int ret = 0;

	before = sched_clock();
	ret = exynos_devfreq_lock_freq(dm_id, rate);
	after = sched_clock();
	latency = after - before;

	if (ret < 0)
		dev_err(mbox->device, "exynos_devfreq_lock_freq ret=%d\n", ret);

	mdelay(10);

	get_rate = acpm_dvfs_get_devfreq(dm_id);
	if (cycle >= 0) {
		dvfs_test->dm[dm_id]->stats[cycle].latency = latency /*ns */ ;
		dvfs_test->dm[dm_id]->stats[cycle].set_rate = rate;
		dvfs_test->dm[dm_id]->stats[cycle].get_rate = get_rate;
		dvfs_test->dm[dm_id]->total_cycle_cnt++;
	}

	dev_info(mbox->device, "%s: domain[%s]set_rate: %d Hz, "
		 "get_rate: %d Hz, latency: %llu ns, ret: %d\n",
		 __func__, dvfs_test->dm[dm_id]->name, rate, get_rate, latency,
		 ret);

	if (acpm_dvfs_log && cycle >= 0) {
		dev_info(mbox->device, "%s: stats:[%s]set_rate: %d Hz, "
			 "get_rate: %d Hz, latency: %llu ns, total_cnt:%d\n",
			 __func__, dvfs_test->dm[dm_id]->name,
			 dvfs_test->dm[dm_id]->stats[cycle].set_rate,
			 dvfs_test->dm[dm_id]->stats[cycle].get_rate,
			 dvfs_test->dm[dm_id]->stats[cycle].latency,
			 dvfs_test->dm[dm_id]->total_cycle_cnt);
	}

	return ret;
}

static int acpm_dvfs_set_cpufreq(unsigned int dm_id, unsigned int rate,
				 int cycle)
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
		dev_err(mbox->device, "cpufreq target failed, ret: %d\n",
			ret);

	after = sched_clock();
	latency = after - before;

	mdelay(10);

	if (cycle >= 0) {
		dvfs_test->dm[dm_id]->stats[cycle].latency = latency /*ns */ ;
		dvfs_test->dm[dm_id]->stats[cycle].set_rate = rate;
		dvfs_test->dm[dm_id]->stats[cycle].get_rate =
		    cpufreq_get(policy_id);
		dvfs_test->dm[dm_id]->total_cycle_cnt++;
	}

	cl0freq = cpufreq_get(CPUCL0_POLICY);
	cl1freq = cpufreq_get(CPUCL1_POLICY);
	cl2freq = cpufreq_get(CPUCL2_POLICY);
	dev_info(mbox->device, "%s: domain[%s]set_rate: %d Hz, CL0: %d Hz, "
		 "CL1: %d Hz, CL2: %d Hz, latency: %llu ns, ret: %d\n",
		 __func__, dvfs_test->dm[dm_id]->name, rate,
		 cl0freq, cl1freq, cl2freq, latency, ret);

	if (acpm_dvfs_log && cycle >= 0) {
		dev_info(mbox->device, "%s: stats:[%s]set_rate: %d Hz, "
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
			dev_info(mbox->device,
				 "==================================="
				 "==================================="
				 "==================================="
				 "===================================\n");
			cycle_cnt = dvfs_test->dm[dm_id]->total_cycle_cnt;

			dev_info(mbox->device,
				 "dm[%s] >10ms:%2d, 10ms~1ms:%2d, "
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
			dev_info(mbox->device,
				 "dm[%s] >10ms:%2d%% 10ms~1ms:%2d%% "
				 "1ms~100us:%2d%% 100us~80us:%2d%% "
				 "80us~60us:%2d%% 60us~40us:%2d%% "
				 "40us~20us:%2d%% 20us~10us:%2d%% "
				 "10us~1us:%2d%%, <1us:%2d%%\n",
				 dvfs_test->dm[dm_id]->name,
				 dvfs_test->dm[dm_id]->scales[9].count * 100 /
				 cycle_cnt,
				 dvfs_test->dm[dm_id]->scales[8].count * 100 /
				 cycle_cnt,
				 dvfs_test->dm[dm_id]->scales[7].count * 100 /
				 cycle_cnt,
				 dvfs_test->dm[dm_id]->scales[6].count * 100 /
				 cycle_cnt,
				 dvfs_test->dm[dm_id]->scales[5].count * 100 /
				 cycle_cnt,
				 dvfs_test->dm[dm_id]->scales[4].count * 100 /
				 cycle_cnt,
				 dvfs_test->dm[dm_id]->scales[3].count * 100 /
				 cycle_cnt,
				 dvfs_test->dm[dm_id]->scales[2].count * 100 /
				 cycle_cnt,
				 dvfs_test->dm[dm_id]->scales[1].count * 100 /
				 cycle_cnt,
				 dvfs_test->dm[dm_id]->scales[0].count * 100 /
				 cycle_cnt);

			dev_info(mbox->device,
				 "==================================="
				 "==================================="
				 "==================================="
				 "===================================\n");
			for (cycle = 0; cycle < DVFS_TEST_CYCLE; cycle++)
				dev_info(mbox->device,
					 "%s: dm_id[%d], set_rate: %d Hz, "
					 "get_rate: %d Hz, latency= %u ns\n",
					 __func__, dm_id,
					 dvfs_test->dm[dm_id]->stats[cycle].
					 set_rate,
					 dvfs_test->dm[dm_id]->stats[cycle].
					 get_rate,
					 dvfs_test->dm[dm_id]->stats[cycle].
					 latency);
			dev_info(mbox->device, "\n");
		}
	}
}

static int acpm_dvfs_test_setting(struct acpm_info *acpm, u64 subcmd)
{
	unsigned int domain = (unsigned int)subcmd;
	static unsigned int pre_set_rate;
	unsigned int set_rate;
	int cycle = 0;

	if (!dvfs_test) {
		dev_err(mbox->device, "%s, dvfs_test is NULL\n", __func__);
		return -ENOMEM;
	}

	if (subcmd >= ACPM_DVFS_CMD_MAX) {
		dev_err(mbox->device, "%s, sub-cmd:%d, out of range!\n",
			__func__, subcmd);
		return -EINVAL;
	}

	dvfs_freq_table_init();

	switch (domain) {
	case ACPM_DVFS_TEST_MIF:
	case ACPM_DVFS_TEST_INT:
		while (cycle < DVFS_TEST_CYCLE) {
			set_rate = get_random_rate(domain);
			if (pre_set_rate == set_rate) {
				/*ignore the stats for duplicated freq request */
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
				/*ignore the stats for duplicated freq request */
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
		dev_err(mbox->device, "%s, subcmd: %d not support\n", __func__,
			subcmd);
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
			debug_acpm_mbox_test_get, debug_acpm_mbox_test_set,
			"0x%016llx\n");
DEFINE_SIMPLE_ATTRIBUTE(debug_acpm_dvfs_test_fops, NULL,
			debug_acpm_dvfs_test_set, "0x%016llx\n");

static void acpm_test_debugfs_init(struct acpm_mbox_test *mbox)
{
	struct dentry *den_mbox;

	den_mbox = debugfs_lookup("acpm_framework", NULL);
	debugfs_create_file("acpm_mbox_test", 0644, den_mbox, mbox,
			    &debug_acpm_mbox_test_fops);
	debugfs_create_file("acpm_dvfs_test", 0644, den_mbox, mbox,
			    &debug_acpm_dvfs_test_fops);
}

static int init_domain_freq_table(struct acpm_dvfs_test *dvfs, int cal_id,
				  int dm_id)
{
	unsigned int orig_table_size;
	unsigned long *cal_freq_table;
	int index, r_index, ret, policy_id;
	struct acpm_dvfs_dm *dm;
	struct cpufreq_policy *policy;

	dm = kmalloc(sizeof(*dm), GFP_KERNEL);
	if (!dm) {
		dev_err(mbox->device, "%s, dm alloc failed\n", __func__);
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
		ret =
		    exynos_devfreq_get_boundary(dm_id, &dvfs->max_freq,
						&dvfs->min_freq);
		if (ret < 0) {
			dvfs->max_freq = cal_dfs_get_max_freq(cal_id);
			dvfs->min_freq = cal_dfs_get_min_freq(cal_id);
			dev_warn(mbox->device,
				 "%s, cal_id: %d, get boundary failed, ret: %d\n",
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
			dev_warn(mbox->device,
				 "%s, cal_id: %d, get boundary failed\n",
				 __func__, cal_id);
		}
		break;
	default:
		dev_err(mbox->device, "%s, cal_id: %d not support\n", __func__,
			cal_id);
		return -EINVAL;
	}

	if (acpm_dvfs_log)
		dev_info(mbox->device, "%s: max_freq = %d, min_freq = %d\n",
			 __func__, dvfs->max_freq, dvfs->min_freq);

	/*
	 * Allocate temporary frequency and voltage tables
	 * to get DVFS table from CAL.
	 */
	orig_table_size = cal_dfs_get_lv_num(cal_id);

	cal_freq_table =
	    kcalloc(orig_table_size, sizeof(unsigned long), GFP_KERNEL);

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
				dev_info(mbox->device,
					 "%s: cal_freq_table[%d]: %d > max_freq: %d\n",
					 __func__, index, cal_freq_table[index],
					 dvfs->max_freq);
			continue;
		}
		if (cal_freq_table[index] < dvfs->min_freq) {
			if (acpm_dvfs_log)
				dev_info(mbox->device,
					 "%s: cal_freq_table[%d]: %d < min_freq: %d\n",
					 __func__, index, cal_freq_table[index],
					 dvfs->min_freq);
			continue;
		}
		dvfs->size++;

		if (acpm_dvfs_log)
			dev_info(mbox->device,
				 "%s: cal_freq_table[%d] = %d Hz, table_size = %d\n",
				 __func__, index, cal_freq_table[index],
				 dvfs->size);
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

	dm->stats =
	    kcalloc(DVFS_TEST_CYCLE, sizeof(struct acpm_dvfs_test_stats),
		    GFP_KERNEL);

	dm->scales =
	    kcalloc(TIME_SCALES, sizeof(struct stats_scale), GFP_KERNEL);
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
	struct acpm_dvfs_validity *dvfs_validity;
	struct acpm_mfd_validity *mfd_validity;
	struct acpm_slc_validity *slc_validity;
	struct acpm_dvfs_test *dvfs;
	int ret = 0;

	dev_info(&pdev->dev, "%s\n", __func__);

	mbox_test =
	    devm_kzalloc(&pdev->dev, sizeof(struct acpm_mbox_test), GFP_KERNEL);
	if (!mbox_test) {
		dev_err(&pdev->dev, "mbox_test alloc failed\n");
		return -ENOMEM;
	}

	mbox_test->device = &pdev->dev;
	platform_set_drvdata(pdev, mbox_test);
	mbox_test->device->of_node = pdev->dev.of_node;

	tmu_validity = kmalloc(sizeof(*tmu_validity), GFP_KERNEL);
	if (!tmu_validity) {
		dev_err(&pdev->dev, "tmu_validity alloc failed\n");
		ret = -ENOMEM;
		goto err_tmu;
	}

	dvfs_validity = kmalloc(sizeof(*dvfs_validity), GFP_KERNEL);
	if (!dvfs_validity) {
		dev_err(&pdev->dev, "dvfs_validity alloc failed\n");
		ret = -ENOMEM;
		goto err_dvfs;
	}

	mfd_validity = kmalloc(sizeof(*mfd_validity), GFP_KERNEL);
	if (!mfd_validity) {
		dev_err(mbox->device, "%s, mfd_validity alloc failed\n",
			__func__);
		ret = -ENOMEM;
		goto err_mfd;
	}

	slc_validity = kmalloc(sizeof(*slc_validity), GFP_KERNEL);
	if (!slc_validity) {
		dev_err(mbox->device, "%s, slc_validity alloc failed\n",
			__func__);
		ret = -ENOMEM;
		goto err_slc;
	}

	mbox_test->tmu = tmu_validity;
	mbox_test->dvfs = dvfs_validity;
	mbox_test->mfd = mfd_validity;
	mbox_test->slc = slc_validity;

	dvfs =
	    devm_kzalloc(&pdev->dev, sizeof(struct acpm_dvfs_test), GFP_KERNEL);
	if (!dvfs) {
		dev_err(&pdev->dev, "dvfs alloc failed\n");
		ret = -ENOMEM;
		goto err_dvfs_latency;
	}

	acpm_test_debugfs_init(mbox_test);

	mutex_init(&mbox_test->mfd->lock);

	mbox = mbox_test;
	dvfs_test = dvfs;

	dev_info(&pdev->dev, "%s done\n", __func__);
	return 0;

err_dvfs_latency:
	kfree(slc_validity);
err_slc:
	kfree(mfd_validity);
err_mfd:
	kfree(dvfs_validity);
err_dvfs:
	kfree(tmu_validity);
err_tmu:
	kfree(mbox_test);
	return ret;
}

static int acpm_mbox_test_remove(struct platform_device *pdev)
{
	int i;

	flush_workqueue(mbox->tmu->rd_tmp_stress_trigger_wq);
	flush_workqueue(mbox->dvfs->mbox_stress_trigger_wq);
	flush_workqueue(mbox->mfd->mbox_stress_trigger_wq);
	flush_workqueue(mbox->slc->mbox_stress_trigger_wq);
	flush_workqueue(mbox->tmu->suspend_wq);
	flush_workqueue(mbox->tmu->resume_wq);
	for (i = 0; i < NUM_OF_WQ; i++) {
		flush_workqueue(mbox->tmu->rd_tmp_random_wq[i]);
		flush_workqueue(mbox->tmu->rd_tmp_concur_wq[i]);
		flush_workqueue(mbox->dvfs->rate_change_wq[i]);
		flush_workqueue(mbox->mfd->s2mpg10_mfd_read_wq[i]);
		flush_workqueue(mbox->mfd->s2mpg11_mfd_read_wq[i]);
		flush_workqueue(mbox->slc->slc_request_wq[i]);
	}

	destroy_workqueue(mbox->tmu->rd_tmp_stress_trigger_wq);
	destroy_workqueue(mbox->dvfs->mbox_stress_trigger_wq);
	destroy_workqueue(mbox->mfd->mbox_stress_trigger_wq);
	destroy_workqueue(mbox->slc->mbox_stress_trigger_wq);
	destroy_workqueue(mbox->tmu->suspend_wq);
	destroy_workqueue(mbox->tmu->resume_wq);
	for (i = 0; i < NUM_OF_WQ; i++) {
		destroy_workqueue(mbox->tmu->rd_tmp_random_wq[i]);
		destroy_workqueue(mbox->tmu->rd_tmp_concur_wq[i]);
		destroy_workqueue(mbox->dvfs->rate_change_wq[i]);
		destroy_workqueue(mbox->mfd->s2mpg10_mfd_read_wq[i]);
		destroy_workqueue(mbox->mfd->s2mpg11_mfd_read_wq[i]);
		destroy_workqueue(mbox->slc->slc_request_wq[i]);
	}

	mutex_destroy(&mbox->mfd->lock);

	kfree(mbox);
	dev_info(mbox->device, "%s done.\n", __func__);
	return 0;
}

static const struct of_device_id acpm_mbox_test_match[] = {
	{.compatible = "google,acpm-mbox-test" },
	{ },
};

MODULE_DEVICE_TABLE(of, acpm_mbox_test_match);

static struct platform_driver acpm_mbox_test_driver = {
	.probe = acpm_mbox_test_probe,
	.remove = acpm_mbox_test_remove,
	.driver = {
		   .name = "acpm-stress",
		   .owner = THIS_MODULE,
		   .of_match_table = acpm_mbox_test_match,
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
