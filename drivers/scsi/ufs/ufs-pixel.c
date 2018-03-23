// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2020 Google LLC
 *
 * Authors: Jaegeuk Kim <jaegeuk@google.com>
 */

#include "ufs-pixel.h"

/* UFSHCD error handling flags */
enum {
	UFSHCD_EH_IN_PROGRESS = (1 << 0),
};

#define ufshcd_eh_in_progress(h) \
	((h)->eh_flags & UFSHCD_EH_IN_PROGRESS)

void pixel_ufs_prepare_command(struct ufs_hba *hba,
			struct request *rq, struct ufshcd_lrb *lrbp)
{
	u8 opcode;

	if (!(rq->cmd_flags & REQ_META))
		return;

	if (hba->dev_info.wspecversion <= 0x300)
		return;

	opcode = (u8)(*lrbp->cmd->cmnd);
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

static DEVICE_ATTR_RO(vendor);
static DEVICE_ATTR_RO(model);
static DEVICE_ATTR_RO(rev);
static DEVICE_ATTR_RO(platform_version);

static struct attribute *pixel_sysfs_ufshcd_attrs[] = {
	&dev_attr_vendor.attr,
	&dev_attr_model.attr,
	&dev_attr_rev.attr,
	&dev_attr_platform_version.attr,
	NULL
};

static const struct attribute_group pixel_sysfs_default_group = {
	.attrs = pixel_sysfs_ufshcd_attrs,
};

int pixel_ufs_update_sysfs(struct ufs_hba *hba)
{
	int err;

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
