// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 Google LLC
 */

#define pr_fmt(fmt) "smra_procfs: " fmt

#include <linux/kobject.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "smra_core.h"
#include "smra_procfs.h"
#include "smra_sysfs.h"

extern struct proc_dir_entry *vendor_mm;
static struct proc_dir_entry *smra_parent;
static struct proc_dir_entry *smra;

extern struct mutex smra_sysfs_lock;

static void smra_footprint_show(struct seq_file *m,
				struct list_head *footprint)
{
	struct smra_metadata *metadata;

	list_for_each_entry(metadata, footprint, list) {
		seq_printf(m, "%lld,%s,%lu,%d\n",
			   metadata->time, metadata->path,
			   metadata->offset, metadata->nr_pages);
	}
}

static int smra_footprint_proc_show(struct seq_file *m, void *v)
{
	struct list_head footprints[SMRA_MAX_TARGET_CNT];
	int i, err;

	mutex_lock(&smra_sysfs_lock);
	if (!smra_config.buffer_has_trace) {
		pr_warn("No trace to be post processed\n");
		mutex_unlock(&smra_sysfs_lock);
		return 0;
	}

	for (i = 0; i < smra_config.nr_targets; i++)
		INIT_LIST_HEAD(&footprints[i]);

	err = smra_post_processing(smra_config.target_pids,
				   smra_config.nr_targets,
				   smra_config.merge_threshold,
				   footprints);
	if (err) {
		pr_warn("Post process failed, error %d\n", err);
		mutex_unlock(&smra_sysfs_lock);
		return err;
	}

	for (i = 0; i < smra_config.nr_targets; i++) {
		seq_printf(m, "<< IO footprint for pid %d >>\n",
			   smra_config.target_pids[i]);
		smra_footprint_show(m, &footprints[i]);
	}
	smra_post_processing_cleanup(footprints, smra_config.nr_targets);
	mutex_unlock(&smra_sysfs_lock);

	return 0;
}

int smra_procfs_init(void)
{

#if IS_ENABLED(CONFIG_PIXEL_STAT)
	smra_parent = vendor_mm;
#else
	smra_parent = proc_mkdir("smra", NULL);
#endif
	if (!smra_parent) {
		pr_err("Failed to find /proc/vendor_mm\n");
		return -ENOMEM;
	}

	smra = proc_mkdir("smra", smra_parent);
	if (!smra) {
		pr_err("Failed to create /proc/vendor_mm/smra\n");
		return -ENOMEM;
	}

	if (!proc_create_single("footprint", 0, smra, smra_footprint_proc_show)) {
		pr_err("Failed to create /proc/vendor_mm/smra/footprint\n");
		return -ENOMEM;
	}

	return 0;
}
