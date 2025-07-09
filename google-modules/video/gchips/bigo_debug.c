// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for BigOcean video accelerator
 *
 * Copyright 2021 Google LLC.
 *
 * Author: Ruofei Ma <ruofeim@google.com>
 */

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "bigo_debug.h"
#include "bigo_io.h"

static int avail_freq_show(struct seq_file *s, void *unused)
{
	struct bigo_core *core = s->private;
	struct bigo_opp *opp;

	list_for_each_entry (opp, &core->pm.opps, list)
		seq_printf(s, "%d\n", opp->freq_khz);

	return 0;
}

static int avail_freqs_open(struct inode *inode, struct file *file)
{
	return single_open(file, avail_freq_show, inode->i_private);
}

static const struct file_operations avail_freqs_fops = {
	.open = avail_freqs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void bigo_init_debugfs(struct bigo_core *core)
{
	struct bigo_debugfs *debugfs = &core->debugfs;

	debugfs->set_freq = 0;
	debugfs->trigger_ssr = 0;
	debugfs->timeout = JOB_COMPLETE_TIMEOUT_MS;

	debugfs->root = debugfs_create_dir("bigo", NULL);
	debugfs_create_file("avail_freqs", 0400, debugfs->root, core,
			&avail_freqs_fops);
	debugfs_create_u32("set_freq", 0200, debugfs->root, &debugfs->set_freq);
	debugfs_create_u32("trigger_ssr", 0600, debugfs->root,
			&debugfs->trigger_ssr);
	debugfs_create_u32("timeout", 0644, debugfs->root,
			&debugfs->timeout);
}

void bigo_uninit_debugfs(struct bigo_core *core)
{
	debugfs_remove_recursive(core->debugfs.root);
}