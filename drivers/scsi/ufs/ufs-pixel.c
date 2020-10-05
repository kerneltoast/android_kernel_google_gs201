// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2020 Google LLC
 *
 * Authors: Jaegeuk Kim <jaegeuk@google.com>
 */

#include "ufs-pixel.h"
#include "ufs-exynos.h"

/* UFSHCD error handling flags */
enum {
	UFSHCD_EH_IN_PROGRESS = (1 << 0),
};

#define ufshcd_eh_in_progress(h) \
	((h)->eh_flags & UFSHCD_EH_IN_PROGRESS)

/* classify request type on statistics by scsi command opcode*/
static int pixel_ufs_get_cmd_type(struct ufshcd_lrb *lrbp, u8 *cmd_type)
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

void pixel_ufs_update_req_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct pixel_req_stats *rst;
	u8 cmd_type;
	u64 delta = (u64)ktime_us_delta(lrbp->compl_time_stamp,
		lrbp->issue_time_stamp);

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

	rst = &(ufs->req_stats[cmd_type]);
	rst->req_count++;
	rst->req_sum += delta;
	if (rst->req_min == 0 || rst->req_min > delta)
		rst->req_min = delta;
	if (rst->req_max < delta)
		rst->req_max = delta;
}

void pixel_ufs_compl_command(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	pixel_ufs_update_req_stats(hba, lrbp);
}

void pixel_ufs_prepare_command(struct ufs_hba *hba,
			struct request *rq, struct ufshcd_lrb *lrbp)
{
	u8 opcode = (u8)(*lrbp->cmd->cmnd);

	/* Assign correct RPMB lun */
	if (opcode == SECURITY_PROTOCOL_IN || opcode == SECURITY_PROTOCOL_OUT) {
		unsigned int lun = (SCSI_W_LUN_BASE |
			(UFS_UPIU_RPMB_WLUN & UFS_UPIU_MAX_UNIT_NUM_ID));

		lrbp->lun = ufshcd_scsi_to_upiu_lun(lun);
		return;
	}

	if (!(rq->cmd_flags & REQ_META))
		return;

	if (hba->dev_info.wspecversion <= 0x300)
		return;

	if (opcode == WRITE_10)
		lrbp->cmd->cmnd[6] = 0x11;
	else if (opcode == WRITE_16)
		lrbp->cmd->cmnd[14] = 0x11;
}

static int ufs_sysfs_emulate_health_est_c(struct ufs_hba *hba, u8 *value)
{
	u8 desc_buf[2] = {0};
	u32 avg_pe_cycle;
	int ret;

	if (ufshcd_eh_in_progress(hba))
		return -EBUSY;

	pm_runtime_get_sync(hba->dev);
	ret = ufshcd_read_desc_param(hba, QUERY_DESC_IDN_HEALTH, 0,
			HEALTH_DESC_PARAM_AVG_PE_CYCLE, desc_buf,
			sizeof(desc_buf));
	pm_runtime_put_sync(hba->dev);
	if (ret)
		return -EINVAL;

	avg_pe_cycle = get_unaligned_be16(desc_buf);
	*value = (u8)(avg_pe_cycle * 100 / HEALTH_DESC_DEFAULT_PE_CYCLE);

	return ret;
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

	if (value == 0 && ufs_sysfs_emulate_health_est_c(hba, &value))
		return  -EINVAL;
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

	return snprintf(buf, PAGE_SIZE, "%.8s\n", hba->sdev_ufs_device->vendor);
}

static ssize_t model_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%.16s\n", hba->sdev_ufs_device->model);
}

static ssize_t rev_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%.4s\n", hba->sdev_ufs_device->rev);
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
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
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
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
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

	if (!ufs->manual_gc.hagc_support) {
		enum query_opcode opcode = (value == MANUAL_GC_ON) ?
						UPIU_QUERY_OPCODE_SET_FLAG:
						UPIU_QUERY_OPCODE_CLEAR_FLAG;

		err = ufshcd_bkops_ctrl(hba, (value == MANUAL_GC_ON) ?
					BKOPS_STATUS_NON_CRITICAL:
					BKOPS_STATUS_CRITICAL);
		if (!hba->auto_bkops_enabled)
			err = -EAGAIN;

		/* flush wb buffer */
		if (hba->dev_info.wspecversion >= 0x0310) {
			u8 index = ufshcd_wb_get_query_index(hba);

			ufshcd_query_flag_retry(hba, opcode,
				QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
				index, NULL);
			ufshcd_query_flag_retry(hba, opcode,
				QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN, index, NULL);
		}
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
	struct exynos_ufs *ufs = to_exynos_ufs(hba);

	return snprintf(buf, PAGE_SIZE, "%lu\n", ufs->manual_gc.delay_ms);
}

static ssize_t manual_gc_hold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	ufs->manual_gc.delay_ms = value;
	return count;
}

static enum hrtimer_restart pixel_mgc_hrtimer_handler(struct hrtimer *timer)
{
	struct exynos_ufs *ufs = container_of(timer, struct exynos_ufs,
					manual_gc.hrtimer);

	queue_work(ufs->manual_gc.mgc_workq, &ufs->manual_gc.hibern8_work);
	return HRTIMER_NORESTART;
}

static void pixel_mgc_hibern8_work(struct work_struct *work)
{
	struct exynos_ufs *ufs = container_of(work, struct exynos_ufs,
					manual_gc.hibern8_work);
	pm_runtime_put_sync(ufs->hba->dev);
	/* bkops will be disabled when power down */
}

void pixel_init_manual_gc(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_manual_gc *mgc = &ufs->manual_gc;
	char wq_name[sizeof("ufs_mgc_hibern8_work")];

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

	return snprintf(buf, PAGE_SIZE, "0x%lx\n", hba->caps);
}

static DEVICE_ATTR_RO(vendor);
static DEVICE_ATTR_RO(model);
static DEVICE_ATTR_RO(rev);
static DEVICE_ATTR_RO(platform_version);
static DEVICE_ATTR_RW(manual_gc);
static DEVICE_ATTR_RW(manual_gc_hold);
static DEVICE_ATTR_RO(host_capabilities);

static struct attribute *pixel_sysfs_ufshcd_attrs[] = {
	&dev_attr_vendor.attr,
	&dev_attr_model.attr,
	&dev_attr_rev.attr,
	&dev_attr_platform_version.attr,
	&dev_attr_manual_gc.attr,
	&dev_attr_manual_gc_hold.attr,
	&dev_attr_host_capabilities.attr,
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

static struct attribute *pixel_sysfs_pixel_attrs[] = {
	&dev_attr_boot_lun_enabled.attr,
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
	struct exynos_ufs *ufs = to_exynos_ufs(hba);			\
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
	struct exynos_ufs *ufs = to_exynos_ufs(hba);

	memset(ufs->req_stats, 0, sizeof(ufs->req_stats));
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

static const struct attribute_group *pixel_ufs_sysfs_groups[] = {
	&pixel_sysfs_group,
	&pixel_sysfs_req_stats_group,
	NULL,
};

int pixel_ufs_update_sysfs(struct ufs_hba *hba)
{
	int err;

	err = sysfs_create_groups(&hba->dev->kobj, pixel_ufs_sysfs_groups);
	if (err) {
		dev_err(hba->dev,
			"%s: sysfs groups creation failed (err = %d)\n",
			__func__, err);
		return err;
	}

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

	return err;
}
