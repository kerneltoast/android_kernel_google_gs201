// SPDX-License-Identifier: GPL-2.0-only
/*
 * GCIP firmware interface.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <gcip/gcip-firmware.h>
#include <gcip/gcip-pm.h>

char *gcip_fw_flavor_str(enum gcip_fw_flavor fw_flavor)
{
	switch (fw_flavor) {
	case GCIP_FW_FLAVOR_BL1:
		return "stage 2 bootloader";
	case GCIP_FW_FLAVOR_SYSTEST:
		return "test";
	case GCIP_FW_FLAVOR_PROD_DEFAULT:
		return "prod";
	case GCIP_FW_FLAVOR_CUSTOM:
		return "custom";
	case GCIP_FW_FLAVOR_UNKNOWN:
	default:
		return "unknown";
	}
}

static int gcip_firmware_tracing_active_get(void *data, u64 *val)
{
	struct gcip_fw_tracing *fw_tracing = data;

	mutex_lock(&fw_tracing->lock);
	*val = fw_tracing->active_level;
	mutex_unlock(&fw_tracing->lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_gcip_firmware_tracing_active, gcip_firmware_tracing_active_get, NULL,
			 "%llu\n");

static int gcip_firmware_tracing_request_get(void *data, u64 *val)
{
	struct gcip_fw_tracing *fw_tracing = data;

	mutex_lock(&fw_tracing->lock);
	*val = fw_tracing->request_level;
	mutex_unlock(&fw_tracing->lock);

	return 0;
}

static int gcip_firmware_tracing_set_level_lock(struct gcip_fw_tracing *fw_tracing)
{
	unsigned long active_level;
	int ret = fw_tracing->set_level(fw_tracing->data, fw_tracing->request_level, &active_level);

	if (ret)
		dev_warn(fw_tracing->dev, "Failed to set firmware tracing level to %lu: %d",
			 fw_tracing->request_level, ret);
	else
		fw_tracing->active_level =
			(fw_tracing->request_level & GCIP_FW_TRACING_DEFAULT_VOTE) ?
				GCIP_FW_TRACING_DEFAULT_VOTE :
				active_level;

	return ret;
}

static int gcip_firmware_tracing_request_set(void *data, u64 val)
{
	struct gcip_fw_tracing *fw_tracing = data;
	int ret = 0;

	mutex_lock(&fw_tracing->lock);

	fw_tracing->request_level = val;
	if (!gcip_pm_get_if_powered(fw_tracing->pm, false)) {
		ret = gcip_firmware_tracing_set_level_lock(fw_tracing);
		gcip_pm_put(fw_tracing->pm);
	}

	mutex_unlock(&fw_tracing->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_gcip_firmware_tracing_request, gcip_firmware_tracing_request_get,
			 gcip_firmware_tracing_request_set, "%llu\n");

struct gcip_fw_tracing *gcip_firmware_tracing_create(const struct gcip_fw_tracing_args *args)
{
	struct gcip_fw_tracing *fw_tracing;

	if (!args->dev || !args->set_level)
		return ERR_PTR(-EINVAL);

	fw_tracing = kzalloc(sizeof(*fw_tracing), GFP_KERNEL);
	if (!fw_tracing)
		return ERR_PTR(-ENOMEM);

	fw_tracing->dev = args->dev;
	fw_tracing->pm = args->pm;
	fw_tracing->set_level = args->set_level;
	fw_tracing->data = args->data;
	fw_tracing->active_level = GCIP_FW_TRACING_DEFAULT_VOTE;
	fw_tracing->request_level = GCIP_FW_TRACING_DEFAULT_VOTE;
	mutex_init(&fw_tracing->lock);

	fw_tracing->dentry = debugfs_create_dir("fw_tracing", args->dentry);
	if (IS_ERR(fw_tracing->dentry)) {
		dev_warn(args->dev, "Failed to create debug FS tracing");
		kfree(fw_tracing);

		return (struct gcip_fw_tracing *)fw_tracing->dentry;
	}

	debugfs_create_file("active", 0440, fw_tracing->dentry, fw_tracing,
			    &fops_gcip_firmware_tracing_active);
	debugfs_create_file("request", 0660, fw_tracing->dentry, fw_tracing,
			    &fops_gcip_firmware_tracing_request);

	return fw_tracing;
}

void gcip_firmware_tracing_destroy(struct gcip_fw_tracing *fw_tracing)
{
	if (!fw_tracing)
		return;

	debugfs_remove_recursive(fw_tracing->dentry);
	kfree(fw_tracing);
}

int gcip_firmware_tracing_restore_on_powering(struct gcip_fw_tracing *fw_tracing)
{
	int ret = 0;

	if (!fw_tracing)
		return 0;

	mutex_lock(&fw_tracing->lock);

	fw_tracing->active_level = GCIP_FW_TRACING_DEFAULT_VOTE;
	if (!(fw_tracing->request_level & GCIP_FW_TRACING_DEFAULT_VOTE))
		ret = gcip_firmware_tracing_set_level_lock(fw_tracing);

	mutex_unlock(&fw_tracing->lock);

	return ret;
}
